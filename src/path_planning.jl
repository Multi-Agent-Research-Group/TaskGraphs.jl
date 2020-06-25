module PathPlanning

using Parameters
using LightGraphs
using MetaGraphs
using DataStructures
using MathOptInterface, JuMP
using TOML
using JLD2, FileIO

using GraphUtils
using CRCBS

using ..TaskGraphs


export
    PC_TAPF

const State = PCCBS.State
const Action = PCCBS.Action

"""
    PC_TAPF{L<:LowLevelSolution}

Defines an instance of a Precedence-Constrained Multi-Agent Task
    Assignment and Path-Finding problem.
"""
struct PC_TAPF{L<:LowLevelSolution}
    env::GridFactoryEnvironment
    schedule::OperatingSchedule       # partial project schedule
    initial_route_plan::L             # initial condition
end

function CRCBS.get_initial_solution(schedule::OperatingSchedule,env::E) where {E<:PCCBS.LowLevelEnv}
    starts = State[]
    robot_ics = get_robot_ICs(schedule)
    for k in sort(collect(keys(robot_ics)))
        push!(starts, State(vtx = get_id(get_location_id(robot_ics[k])), t = 0))
    end
    cost_model = get_cost_model(env)
    paths = map(s->Path{State,Action,get_cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end

include("legacy/pc_tapf_solver.jl")


# export
#     AbstractSolverException,
#     SolverTimeOutException,
#     SolverMilpMaxOutException,
#     SolverAstarMaxOutException,
#     read_solver
#
# abstract type AbstractSolverException end
#
# @with_kw struct SolverTimeOutException <: AbstractSolverException
#     msg::String = ""
# end
# @with_kw struct SolverMilpMaxOutException <: AbstractSolverException
#     msg::String = ""
# end
# @with_kw struct SolverCBSMaxOutException <: AbstractSolverException
#     msg::String = ""
# end
# @with_kw struct SolverAstarMaxOutException <: AbstractSolverException
#     msg::String = ""
# end
# Helpers for printing
export
    log_info

function log_info(limit::Int,verbosity::Int,msg...)
    if verbosity > limit
        println("[ logger ]: ",msg...)
    end
end

export
    PlanningCache,
    isps_queue_cost,
    initialize_planning_cache,
    reset_cache!,
    update_planning_cache!,
    repair_solution!,
    plan_path!,
    plan_next_path!

@with_kw struct PlanningCache
    closed_set::Set{Int}                    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}                    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} = PriorityQueue{Int,Tuple{Int,Float64}}() # active nodes prioritized by slack
    t0::Vector{Int}                         = Vector{Int}()
    tF::Vector{Int}                         = Vector{Int}()
    slack::Vector{Vector{Float64}}          = Vector{Vector{Float64}}()
    local_slack::Vector{Vector{Float64}}    = Vector{Vector{Float64}}()
    max_deadline::Vector{Int}               = Vector{Int}() # Marks the time at whidh this node will begin accumulating a delay cost
end

function isps_queue_cost(schedule::OperatingSchedule,cache::PlanningCache,v::Int)
    path_spec = get_path_spec(schedule,v)
    return (Int(path_spec.plan_path), minimum(cache.slack[v]))
end

function initialize_planning_cache(schedule::OperatingSchedule;kwargs...)
    t0,tF,slack,local_slack = process_schedule(schedule;kwargs...);
    allowable_slack = map(i->minimum(i),slack) # soft deadline (can be tightened as necessary)
    allowable_slack = map(i->i==Inf ? typemax(Int) : Int(i), allowable_slack) # initialize deadline at infinity
    cache = PlanningCache(t0=t0,tF=tF,slack=slack,local_slack=local_slack,max_deadline=allowable_slack)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
        end
    end
    cache
end

"""
    `reset_cache!(cache,schedule)`

    Resets the cache so that a solution can be repaired (otherwise calling
    low_level_search!() will return immediately because the cache says it's
    complete)
"""
function reset_cache!(cache::PlanningCache,schedule::OperatingSchedule)
    t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
    cache.t0            .= t0
    cache.tF            .= tF
    cache.slack         .= slack
    cache.local_slack   .= local_slack
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
        end
    end
    cache
end

export
    SearchEnv,
    construct_search_env,
    update_env!

@with_kw struct SearchEnv{C,E<:AbstractLowLevelEnv{State,Action,C},S} <: AbstractLowLevelEnv{State,Action,C}
    schedule::OperatingSchedule     = OperatingSchedule()
    cache::PlanningCache            = PlanningCache()
    env::E                          = PCCBS.LowLevelEnv()
    problem_spec::ProblemSpec       = ProblemSpec()
    dist_function::DistMatrixMap    = env.graph.dist_function # DistMatrixMap(env.graph.vtx_map, env.graph.vtxs)
    cost_model::C                   = get_cost_model(env)
    num_agents::Int                 = length(get_robot_ICs(schedule))
    route_plan::S                = LowLevelSolution(
        paths = map(i->Path{State,Action,get_cost_type(cost_model)}(),
            1:num_agents),
        cost_model = cost_model,
        costs = map(i->get_initial_cost(cost_model), 1:num_agents),
        cost = get_infeasible_cost(cost_model),
        # cost  = aggregate_costs(cost_model,
        #     map(i->get_initial_cost(cost_model), 1:num_agents)),
        )
end
function CRCBS.get_start(env::SearchEnv,v::Int)
    start_vtx   = get_path_spec(env.schedule,v).start_vtx
    start_time  = env.cache.t0[v]
    PCCBS.State(start_vtx,start_time)
end
CRCBS.get_cost(env::SearchEnv) = get_cost(env.route_plan)

# function reverse_propagate_delay!(solver,cache,schedule,delay_vec)
#     buffer = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer[v] = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         for v2 in outneighbors(schedule,v)
#             delay_vec[v] = max(delay_vec[v], delay_vec[v2] - buffer[v2])
#             delay_vec[v] = max(0, delay_vec[v] - (cache.t0[v2] - cache.tF[v]))
#         end
#     end
#     delay_vec
# end
# function get_delay_vec(solver,cache,schedule,v0)
#     delay = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         delay[v] = (cache.tF[v] - (cache.t0[v] + Δt_min + minimum(cache.local_slack[v])))
#     end
#     # zero out all vtxs that are not predecessors of v
#     delay_vec = zeros(Int,nv(schedule))
#     for v in map(e->e.dst,collect(edges(bfs_tree(get_graph(schedule),v0;dir=:in))))
#         if cache.max_deadline[v] > 0
#             delay_vec[v] = min(delay[v],typemax(Int))
#         end
#     end
#     delay_vec
# end
# function backtrack_deadlines(solver,cache,schedule,v)
#     frontier = Set{Int}([v])
#     delay_cut = Set{Int}()
#     while length(frontier) > 0
#         v = pop!(frontier)
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         if get_path_spec(schedule,v).fixed
#             continue
#         elseif cache.max_deadline[v] > 0 # still has room for some delay
#             push!(delay_cut,v)
#         elseif indegree(schedule,v) == 0
#             # if v is a root_node, the deadlines cannot be tightened anymore
#             log_info(-1,solver.l3_verbosity,"ISPS: deadlines cannot be tightened any more.")
#             return Set{Int}()
#         else
#             for v2 in inneighbors(schedule,v)
#                 push!(frontier,v2)
#             end
#         end
#     end
#     return delay_cut
# end
# function tighten_deadline!(solver,env,route_plan,v,dt=1)
#     cache = env.cache
#     schedule = env.schedule
#     active_set = cache.active_set
#     closed_set = cache.closed_set
#     node_queue = cache.node_queue
#     graph = get_graph(schedule)
#     # adjust allowable_slack
#     cache.max_deadline[v] = max(0, cache.max_deadline[v]-1)
#     # remove from closed set
#     push!(active_set, v)
#     setdiff!(closed_set,v)
#     # Trim schedule at v.t0
#     agent_id = get_path_spec(schedule,v).agent_id
#     if agent_id != -1
#         cbs_env = typeof(env.env)(
#             graph = env.env.graph,
#             agent_idx = agent_id,
#             cost_model = get_cost_model(env.env),
#             heuristic = get_heuristic_model(env.env)
#         )
#         new_path = trim_path(cbs_env,get_paths(route_plan)[agent_id],cache.t0[v])
#         set_solution_path!(route_plan,new_path,agent_id)
#         set_path_cost!(route_plan,new_path.cost,agent_id)
#     end
#     # Update node queue
#     for v2 in map(e->e.dst,collect(edges(bfs_tree(graph,v;dir=:out))))
#         setdiff!(closed_set,v2)
#         setdiff!(active_set,v2)
#     end
#     empty!(node_queue)
#     for v2 in active_set
#         node_queue[v2] = isps_queue_cost(schedule,cache,v2)
#     end
#     env,route_plan
# end
function update_planning_cache!(solver,env::E,v::Int,path::P) where {E<:SearchEnv,P<:Path}
    cache = env.cache
    schedule = env.schedule
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)

    # update t0, tF, slack, local_slack
    Δt = get_final_state(path).t - cache.tF[v]
    if Δt > 0
        # delay = Δt - minimum(cache.slack[v])
        # if delay > 0
        #     log_info(-1,solver.l3_verbosity,"LOW LEVEL SEARCH: schedule delay of ",delay," time steps incurred by path for vertex v = ",v," - ",string(get_node_from_vtx(schedule,v)), " - tF = ",get_final_state(path).t)
        #     # Backtracking
        #     tightenable_set = backtrack_deadlines(solver,cache,schedule,v)
        #     delay_vec = get_delay_vec(solver,cache,schedule,v)
        #     # @show delay_vec
        #     if any(delay_vec .> 0)
        #         for v_ in topological_sort(graph)
        #             if delay_vec[v_] > 0
        #                 log_info(-1,solver,"ISPS: tightening at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
        #                 tighten_deadline!(solver,env,solution,v_)
        #                 break
        #             end
        #         end
        #     # if length(tightenable_set) > 0
        #     #     for v_ in tightenable_set
        #     #         log_info(-1,solver,"ISPS: backtracking at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
        #     #         # tighten_deadline!(solver,env,solution,v_)
        #     #     end
        #         t0,tF,slack,local_slack = process_schedule(schedule;
        #             t0=map(i->i in closed_set ? cache.t0[i] : 0, vertices(schedule)),
        #             tF=map(i->i in closed_set ? cache.tF[i] : 0, vertices(schedule)),
        #             # t0=map(i->is_root_node(schedule,v) ? cache.t0[i] : 0, vertices(schedule)),
        #             # tF=map(i->is_root_node(schedule,v) ? cache.tF[i] : 0, vertices(schedule)),
        #         )
        #         cache.t0            .= t0
        #         cache.tF            .= tF
        #         cache.slack         .= slack
        #         cache.local_slack   .= local_slack
        #         return cache
        #     end
        # end
        cache.tF[v] = get_final_state(path).t
        t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
        cache.t0            .= t0
        cache.tF            .= tF
        cache.slack         .= slack
        cache.local_slack   .= local_slack
    end
    # update closed_set
    activated_vtxs = Int[]
    push!(closed_set,v)
    # update active_set
    setdiff!(active_set, v)
    for v2 in outneighbors(graph,v)
        active = true
        for v1 in inneighbors(graph,v2)
            if !(v1 in closed_set)
                active = false
                break
            end
        end
        if active
            push!(activated_vtxs,v2)
            push!(active_set, v2)               # add to active set
        end
    end
    # update priority queue
    for v2 in active_set
        node_queue[v2] = isps_queue_cost(schedule,cache,v2)
    end
    log_info(2,solver,"moved ",v," to closed set, moved ",activated_vtxs," to active set")
    log_info(3,solver,string("cache.tF[v] = ",cache.tF))
    return cache
end

function CRCBS.SumOfMakeSpans(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    SumOfMakeSpans(
        cache.tF,
        schedule.root_nodes,
        map(k->schedule.weights[k], schedule.root_nodes),
        cache.tF[schedule.root_nodes])
end
function CRCBS.MakeSpan(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    MakeSpan(
        cache.tF,
        schedule.root_nodes,
        map(k->schedule.weights[k], schedule.root_nodes),
        cache.tF[schedule.root_nodes])
end

export
    construct_cost_model

function construct_cost_model end

function construct_search_env(solver, schedule, problem_spec, env_graph;
        primary_objective=SumOfMakeSpans(),
        extra_T=400,
        kwargs...
    )
    cache = initialize_planning_cache(schedule;kwargs...)
    N = problem_spec.N                                          # number of robots
    cost_model, heuristic_model = construct_cost_model(
        solver, schedule, cache, problem_spec, env_graph;
        primary_objective=primary_objective, extra_T=extra_T)
    low_level_env = PCCBS.LowLevelEnv(
        graph=env_graph, cost_model=cost_model, heuristic=heuristic_model)
    base_route_plan = get_initial_solution(schedule,low_level_env)

    search_env = SearchEnv(schedule=schedule, cache=cache, env=low_level_env,
        problem_spec=problem_spec, num_agents=N, route_plan=base_route_plan)
    return search_env
end
function construct_search_env(solver,schedule::OperatingSchedule,
        env::SearchEnv;
        primary_objective=env.problem_spec.cost_function,
        t0 = env.cache.t0,
        tF = env.cache.tF,
        kwargs...)
    construct_search_env(solver,schedule,env.problem_spec,env.env.graph;
        primary_objective=primary_objective,t0=t0,tF=tF,kwargs...)
end

update_cost_model!(model::C,env::S) where {C,S<:SearchEnv} = nothing
function update_cost_model!(model::C,env::S) where {C<:MultiDeadlineCost,S<:SearchEnv}
    model.tF .= env.cache.tF
end
function update_cost_model!(model::C,env::S) where {C<:CompositeCostModel,S<:SearchEnv}
    for m in model.cost_models
        update_cost_model!(m,env)
    end
end
update_cost_model!(env::S) where {S<:SearchEnv} = update_cost_model!(env.cost_model,env)

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver,env::SearchEnv,route_plan::L,v::Int,path::P,
        agent_id::Int=get_path_spec(env.schedule,v).agent_id
        ) where {P<:Path,L<:LowLevelSolution}
    cache = env.cache
    schedule = env.schedule

    # UPDATE CACHE
    update_planning_cache!(solver,env,v,path)
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env.env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
        partially_set_path!(get_cost_model(env.env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
    end

    env
end

function CRCBS.build_env(solver, env::E, node::N, schedule_node::T, v::Int, path_spec=get_path_spec(env.schedule, v);
        heuristic = get_heuristic_model(env.env),
        cost_model = get_cost_model(env.env)
    ) where {E<:SearchEnv,N<:ConstraintTreeNode,T}
    agent_id = path_spec.agent_id
    goal_vtx = path_spec.final_vtx
    goal_time = env.cache.tF[v]                             # time after which goal can be satisfied

    # deadline = env.cache.tF[v] .+ min.(env.cache.max_deadline[v],env.cache.slack[v]) # NOTE iterative deadline tightening was causing problems with slack running out before the goal time, so this line has been replaced by the original
    # deadline = env.cache.tF[v] .+ min.(max.(env.cache.local_slack[v], env.cache.max_deadline[v]),env.cache.slack[v]) # This is a potential fix that would allow iterative tightening to keep working
    deadline = env.cache.tF[v] .+ env.cache.slack[v]         # deadline for DeadlineCost
    # Adjust deadlines if necessary:
    if get_path_spec(env.schedule, v).tight == true
        goal_time += minimum(env.cache.local_slack[v])
    end
    for v_next in outneighbors(get_graph(env.schedule),v)
        if get_path_spec(env.schedule, v_next).static == true
            duration_next = get_path_spec(env.schedule,v_next).min_path_duration
            for c in get_constraints(node, agent_id).sorted_state_constraints
                if get_sp(get_path_node(c)).vtx == goal_vtx
                    if 0 < get_time_of(c) - goal_time < duration_next
                        log_info(1,solver,string("extending goal_time for node ",v," from ",goal_time," to ",get_time_of(c)," to avoid constraints"))
                        goal_time = max(goal_time, get_time_of(c))
                    end
                end
            end
        end
    end
    if (path_spec.free == true) && is_terminal_node(get_graph(env.schedule),v)
        goal_time = maximum(env.cache.tF)
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        log_info(3,solver,string("BUILD ENV: setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    cbs_env = PCCBS.LowLevelEnv(
        graph       = env.env.graph,
        schedule_node = schedule_node,
        node_id     = get_vtx_id(env.schedule,v),
        constraints = get_constraints(node, agent_id), # agent_id represents the whole path
        goal        = PCCBS.State(goal_vtx,goal_time),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        heuristic   = heuristic,
        cost_model  = cost_model
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    # retrieve base_path
    if !is_root_node(get_graph(env.schedule),v) # && agent_id != -1
        # log_info(0,solver,string("retrieving base path for node ",v," of type ",typeof(schedule_node)))
        base_path = get_paths(node.solution)[agent_id]
    else
        base_path = get_paths(node.solution)[agent_id]
        # log_info(0,solver,string("initializing base path for node ",v," of type ",typeof(schedule_node)))
        # start_vtx = path_spec.start_vtx
        # base_path = Path{PCCBS.State,PCCBS.Action,get_cost_type(cbs_env)}(
        #     s0=PCCBS.State(start_vtx, 0), # TODO fix start time
        #     cost=get_initial_cost(cbs_env)
        #     )
        # try
        #     @assert(PCCBS.State(path_spec.start_vtx, 0) == get_final_state(base_path),"State(path_spec.start_vtx, 0) = $((path_spec.start_vtx,0)), not equal to get_final_state(base_path) = $(get_final_state(base_path))")
        # catch e
        #     println(e.msg)
        #     @show path_spec
        #     @show schedule_node
        #     throw(e)
        # end
    end
    # make sure base_path hits t0 constraint
    if get_end_index(base_path) < env.cache.t0[v]
        log_info(1, solver, string("LOW LEVEL SEARCH: in schedule node ",v," -- ",
            string(schedule_node),": cache.t0[v] - get_end_index(base_path) = ",env.cache.t0[v] - get_end_index(base_path),". Extending path to ",env.cache.t0[v]," ..."))
        # base_path = extend_path(cbs_env,base_path,env.cache.t0[v])
        extend_path!(cbs_env,base_path,env.cache.t0[v])
    end
    # log_info(2,solver.l3_verbosity,
    # "LOW LEVEL SEARCH: v=$(v), node=$(string(schedule_node)), deadline=$(deadline), ",
    # "local_slack=$(env.cache.slack[v]), slack=$(env.cache.slack[v]), goal=$(string(cbs_env.goal)),
    # ")
    return cbs_env, base_path
end
CRCBS.build_env(solver, env::SearchEnv, node::ConstraintTreeNode, v::Int) = build_env(solver, env, node, get_node_from_vtx(env.schedule,v), v)
"""
For COLLABORATIVE transport problems
"""
function CRCBS.build_env(solver, env::E, node::N, schedule_node::TEAM_ACTION, v::Int;kwargs...) where {E<:SearchEnv,N<:ConstraintTreeNode}
    envs = []
    starts = Vector{PCCBS.State}()
    meta_cost = MetaCost(Vector{get_cost_type(env)}(),get_initial_cost(env.env))
    # path_specs = Vector{PathSpec}()
    for (i, sub_node) in enumerate(schedule_node.instructions)
        # if i == 1 # leader
        ph = PerfectHeuristic(env.dist_function.dist_mtxs[schedule_node.shape][i])
        # heuristic = construct_heuristic_model(solver,env_graph;ph=ph)
        heuristic = construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
        # else
        #     heuristic = get_heuristic_model(env.env)
        # end
        cbs_env, base_path = build_env(solver,env,node,sub_node,v,generate_path_spec(env.schedule,env.problem_spec,sub_node);
            heuristic=heuristic,
            kwargs...
        ) # TODO need problem_spec here
        push!(envs, cbs_env)
        push!(starts, get_final_state(base_path))
        push!(meta_cost.independent_costs, get_cost(base_path))
    end
    meta_env = MetaAgentCBS.construct_meta_env([envs...], get_cost_model(env))
    meta_path = Path{MetaAgentCBS.State{PCCBS.State},MetaAgentCBS.Action{PCCBS.Action},MetaCost{get_cost_type(env)}}(
        s0 = MetaAgentCBS.State(starts),
        cost = MetaCost(meta_cost.independent_costs, aggregate_costs(get_cost_model(meta_env), meta_cost.independent_costs))
    )
    return meta_env, meta_path
end

export
    reset_solution!

"""
    Helper to reset the solution in a constraint node between re-runs of ISPS
"""
function reset_solution!(node::N,base_route_plan) where {N<:ConstraintTreeNode}
    for (agent_id,path) in enumerate(get_paths(base_route_plan))
        set_solution_path!(node.solution, deepcopy(path), agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
        node.solution.cost = base_route_plan.cost
        node.cost = get_cost(node.solution)
    end
    node
end


################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

export
    PC_MAPF


CRCBS.get_cost_model(env::E) where {E<:SearchEnv}   = get_cost_model(env.env)
CRCBS.get_cost_type(env::E) where {E<:SearchEnv}    = get_cost_type(env.env)
function CRCBS.initialize_root_node(env::SearchEnv)
    N = env.num_agents
    # It is important to only have N starts! Does not matter if they are invalid states.
    dummy_mapf = MAPF(env.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N))
    initialize_root_node(dummy_mapf,deepcopy(env.route_plan))
end
function CRCBS.initialize_child_search_node(node::N,env::SearchEnv) where {N<:ConstraintTreeNode}
    initialize_child_search_node(node,deepcopy(env.route_plan))
end
function CRCBS.default_solution(env::SearchEnv)
    N = env.num_agents
    dummy_mapf = MAPF(env.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N))
    default_solution(dummy_mapf)
end

"""
    `PC_MAPF`

    A precedence-constrained multi-agent path-finding problem. All agents have
    assigned tasks, but there are precedence constraints between tasks.
"""
struct PC_MAPF{E<:SearchEnv} <: AbstractMAPF
    env::E
end
CRCBS.initialize_root_node(pc_mapf::P) where {P<:PC_MAPF} = initialize_root_node(pc_mapf.env)
CRCBS.default_solution(pc_mapf::M) where {M<:PC_MAPF} = default_solution(pc_mapf.env)
CRCBS.initialize_child_search_node(node::N,pc_mapf::PC_MAPF) where {N<:ConstraintTreeNode} = initialize_child_search_node(node,pc_mapf.env)

function CRCBS.check_termination_criteria(solver::S,env::E,cost_so_far,s) where {S<:PC_TAPF_Solver,E<:AbstractLowLevelEnv}
    # solver.num_A_star_iterations += 1
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
        return true
    end
    return false
end

end # PathPlanning module
