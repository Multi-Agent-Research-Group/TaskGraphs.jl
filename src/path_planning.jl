# module PathPlanning
#
# using Parameters
# using LightGraphs
# using MetaGraphs
# using DataStructures
# using MathOptInterface, JuMP
# using TOML
# using JLD2, FileIO
#
# using GraphUtils
# using CRCBS
#
# using ..TaskGraphs


export
    PlanningCache,
    isps_queue_cost,
    initialize_planning_cache,
    reset_cache!


@with_kw_noshow struct PlanningCache
    closed_set::Set{Int}                    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}                    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} = PriorityQueue{Int,Tuple{Int,Float64}}() # active nodes prioritized by slack
    t0::Vector{Int}                         = Vector{Int}()
    tF::Vector{Int}                         = Vector{Int}()
    slack::Vector{Vector{Float64}}          = Vector{Vector{Float64}}()
    local_slack::Vector{Vector{Float64}}    = Vector{Vector{Float64}}()
    # max_deadline::Vector{Int}               = Vector{Int}() # Marks the time at which this node will begin accumulating a delay cost
end
function sprint_cache(io::IO, cache::PlanningCache;label_pad=14,pad=5)
    lpad(str) = sprint_padded(str;pad=label_pad,leftaligned=true)
    rpad(str) = sprint_padded(str;pad=label_pad,leftaligned=false)
    spad(str;kwargs...) = sprint_padded_list(str;pad=pad,leftaligned=false,kwargs...)
    print(io,"PlanningCache:","\n")
    print(io,"\t",lpad("closed_set:  "),cache.closed_set,"\n")
    print(io,"\t",lpad("active_set:  "),cache.active_set,"\n")
    print(io,"\t",lpad("node_queue:  "),cache.node_queue,"\n")
    print(io,"\t",rpad("          v: "),spad(1:length(cache.t0);lchar=" ",rchar=" "),"\n")
    print(io,"\t",lpad("t0:          "),spad(cache.t0),"\n")
    print(io,"\t",lpad("tF:          "),spad(cache.tF),"\n")
    if !isempty(cache.slack)
        slack_dim = length(cache.slack[1])
        for i in 1:slack_dim
            pre = (i == 1) ? "slack:" : ""
            print(io,"\t",lpad(pre),spad(map(slc->slc[i],cache.slack)),"\n")
        end
        for i in 1:slack_dim
            pre = (i == 1) ? "local_slack:" : ""
            print(io,"\t",lpad(pre),spad(map(slc->slc[i],cache.local_slack)),"\n")
        end
    end
end

function Base.show(io::IO, cache::PlanningCache)
    sprint_cache(io, cache)
end

function isps_queue_cost(schedule::OperatingSchedule,cache::PlanningCache,v::Int)
    path_spec = get_path_spec(schedule,v)
    return (Int(path_spec.plan_path), minimum(cache.slack[v]))
end

function initialize_planning_cache(schedule::OperatingSchedule,t0_=zeros(nv(schedule)),tF_=zeros(nv(schedule)))
    t0,tF,slack,local_slack = process_schedule(schedule,t0_,tF_)
    # allowable_slack = map(i->minimum(i),slack) # soft deadline (can be tightened as necessary)
    # allowable_slack = map(i->i==Inf ? typemax(Int) : Int(i), allowable_slack) # initialize deadline at infinity
    cache = PlanningCache(t0=t0,tF=tF,slack=slack,local_slack=local_slack) #,max_deadline=allowable_slack)
    for v in get_all_root_nodes(schedule)
        push!(cache.active_set,v)
        enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
    end
    # TODO skip over all nodes for which path_spec.plan_path == false
    cache
end
function reinitialize_planning_cache!(sched,cache,
        t0_ = vcat(cache.t0,zeros(nv(sched)-length(cache.t0))),
        tF_ = vcat(cache.tF,zeros(nv(sched)-length(cache.tF)))
        )
    t0,tF,slack,local_slack = process_schedule(sched,t0_,tF_)
    empty!(cache.t0)
    empty!(cache.tF)
    empty!(cache.slack)
    empty!(cache.local_slack)
    append!(cache.t0,t0)
    append!(cache.tF,tF)
    append!(cache.slack,slack)
    append!(cache.local_slack,local_slack)
    cache
end

"""
    `reset_cache!(cache,schedule)`

    Resets the cache so that a solution can be repaired (otherwise calling
    low_level_search!() will return immediately because the cache says it's
    complete)
"""
function reset_cache!(cache::PlanningCache,schedule::OperatingSchedule,t0=cache.t0,tF=cache.tF)
    t0,tF,slack,local_slack = process_schedule(schedule,t0,tF)
    cache.t0            .= t0
    cache.tF            .= tF
    cache.slack         .= slack
    cache.local_slack   .= local_slack
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    # for v in get_all_terminal_nodes(schedule)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
        end
    end
    cache
end

export
    EnvGraphDict,
    construct_env_graph_dict

const EnvGraphDict = Dict{Symbol,GridFactoryEnvironment}
function construct_env_graph_dict(sched::OperatingSchedule,env::GridFactoryEnvironment)
    dict = EnvGraphDict()
    dict[:Default] = env
    for v in vertices(sched)
        node = get_node_from_vtx(sched,v)
        k = graph_key(node)
        if !haskey(dict,k)
            dict[k] = GridFactoryEnvironment(
                env,
                graph=deepcopy(env.graph),
                dist_function=deepcopy(get_dist_matrix(env))
            )
        end
    end
    return dict
end

function initialize_route_plan end

const State = CRCBS.GraphState
const Action = CRCBS.GraphAction

export
    construct_cost_model,
    construct_heuristic_model

function construct_cost_model end
function construct_heuristic_model end

export
    EnvironmentLayer,
    SearchEnv,
    construct_search_env,
    update_env!

@with_kw_noshow struct EnvironmentLayer
    graph::GridFactoryEnvironment   = GridFactoryEnvironment()
    problem_spec::ProblemSpec       = ProblemSpec()
end
CRCBS.get_graph(layer::EnvironmentLayer)            = layer.graph
get_problem_spec(layer::EnvironmentLayer)           = layer.problem_spec
CRCBS.get_cost_model(layer::EnvironmentLayer)       = layer.cost_model
CRCBS.get_heuristic_model(layer::EnvironmentLayer)  = layer.heuristic_model

function construct_environment_layer(env::GridFactoryEnvironment,problem_spec::ProblemSpec)
    env_graph = GridFactoryEnvironment(env, graph=deepcopy(env.graph),
        dist_function=deepcopy(get_dist_matrix(env))
        )
    prob_spec = ProblemSpec(problem_spec, D=get_dist_matrix(env_graph))
    EnvironmentLayer(env_graph,prob_spec)
end

function populate_environment_layer_dict!(layers,
        sched::OperatingSchedule,
        env::GridFactoryEnvironment,
        prob_spec::ProblemSpec,
    )
    gkeys = unique(map(graph_key, values(sched.planning_nodes)))
    for k in gkeys
        layers[k] = construct_environment_layer(env,prob_spec)
    end
    return layers
end
function construct_environment_layer_dict(args...)
    layers = Dict{Symbol,EnvironmentLayer}()
    populate_environment_layer_dict!(layers,args...)
end

@with_kw_noshow struct SearchEnv{C,H,S} <: GraphEnv{State,Action,C}
    schedule::OperatingSchedule     = OperatingSchedule()
    cache::PlanningCache            = PlanningCache()
    # Each agent type needs its own set of these #
    env_layers::Dict{Symbol,EnvironmentLayer} = Dict{Symbol,EnvironmentLayer}()
    # ###################################### #
    cost_model::C                   = C()
    heuristic_model::H              = H()
    num_agents::Int                 = length(get_robot_ICs(schedule))
    route_plan::S                   = initialize_route_plan(schedule,cost_model)
end

export
    get_schedule,
    get_cache,
    get_problem_spec,
    get_route_plan

CRCBS.get_graph(env::SearchEnv,k=graph_key())          = get_graph(env.env_layers[k])
get_problem_spec(env::SearchEnv,k=graph_key())         = get_problem_spec(env.env_layers[k])
CRCBS.get_cost_model(env::SearchEnv)        = env.cost_model
CRCBS.get_heuristic_model(env::SearchEnv)   = env.heuristic_model
get_schedule(env::SearchEnv) = env.schedule
get_schedule(sched::OperatingSchedule) = sched
get_cache(env::SearchEnv) = env.cache
get_route_plan(env::SearchEnv) = env.route_plan
function CRCBS.get_start(env::SearchEnv,v::Int)
    start_vtx   = get_path_spec(get_schedule(env),v).start_vtx
    start_time  = get_cache(env).t0[v]
    State(start_vtx,start_time)
end
CRCBS.num_agents(env::SearchEnv) = env.num_agents
for op in [
    :cost_type,:state_type,:action_type,:path_type,:get_cost,:get_paths,
    :get_path_costs,:set_cost!,:set_solution_path!,:set_path_cost!,
    :convert_to_vertex_lists,:detect_conflicts,:detect_conflicts!
    ]
    @eval CRCBS.$op(env::SearchEnv,args...) = $op(get_route_plan(env),args...)
end
function sprint_search_env(io::IO,env::SearchEnv)
    # print(io,"schedule: ",get_schedule(env),"\n")
    print(io,"SearchEnv: \n")
    print(io,"cache: ",sprint(sprint_cache,get_cache(env)))
    print(io,"active task nodes:","\n")
    for v in get_cache(env).active_set
        print(io,"\t","v = ",
            sprint_padded(v)," => ",
            string(get_node_from_vtx(get_schedule(env),v)),"\n")
    end
    print(io,"route_plan: ",get_route_plan(env),"\n")
end
function Base.show(io::IO,env::SearchEnv)
    sprint_search_env(io,env)
end

export
    get_t0,
    get_tF
    
get_t0(env::SearchEnv,v::Int) = get_cache(env).t0[v]
get_tF(env::SearchEnv,v::Int) = get_cache(env).tF[v]
for op in [:get_t0,:get_tF]
    @eval $op(env::SearchEnv,id::AbstractID) = $op(env,get_vtx(get_schedule(env),id))
end

export
    get_env_snapshot,
    trim_route_plan

"""
    get_env_snapshot(route_plan::S,t)
"""
# function get_env_snapshot(route_plan::S,t) where {S<:LowLevelSolution}
#     Dict(RobotID(i)=>ROBOT_AT(i, get_sp(get_path_node(path,t)).vtx) for (i,path) in enumerate(get_paths(route_plan)))
# end
function get_env_snapshot(env::SearchEnv,t)
    sched = get_schedule(env)
    route_plan = get_route_plan(env)
    Dict(id=>BOT_AT(id,
        LocationID(get_sp(get_path_node(get_paths(route_plan)[get_id(id)],t)).vtx)
        ) for (id,r) in get_robot_ICs(sched))
end

export
    EnvState,
    get_env_state

"""
    EnvState
Reflects the state of the SearchEnv environment at a given time step.
"""
@with_kw_noshow struct EnvState
    robot_positions::Dict{BotID,BOT_AT} = Dict{BotID,BOT_AT}()
    object_positions::Dict{ObjectID,OBJECT_AT} = Dict{ObjectID,OBJECT_AT}()
    objects_active::Dict{ObjectID,Bool} = Dict{ObjectID,Bool}()
end

export
    robot_positions,
    object_positions,
    objects_active,
    object_position,
    object_active,
    robot_position

robot_positions(s::EnvState) = s.robot_positions
object_positions(s::EnvState) = s.object_positions
objects_active(s::EnvState) = s.objects_active
object_position(s::EnvState,k) = object_positions(s)[k]
robot_position(s::EnvState,k) = robot_positions(s)[k]
object_active(s::EnvState,k) = objects_active(s)[k]

function Base.show(io::IO,s::EnvState)
    print(io,"EnvState")
    for (n,dict) in [
        ("robot_positions",robot_positions(s)),
        ("object_positions",object_positions(s)),
        ("objects_active",objects_active(s)),
        ]
        print(io,"\n\t",n," : ")
        for k in sort(collect(keys(dict)))
            val = dict[k]
            print(io,string(val),", ")
        end
    end
end
function get_env_state(env,t)
    sched = get_schedule(env)
    cache = get_cache(env)
    robot_positions = get_env_snapshot(env,t)
    object_positions = Dict{ObjectID,OBJECT_AT}()
    objects_active = Dict{ObjectID,Bool}()
    for (o,o_node) in get_object_ICs(sched)
        v = get_vtx(sched,o)
        if t <= cache.t0[v]
            object_positions[o] = o_node
            objects_active[o] = t >= cache.t0[v]
        else
            object_positions[o] = o_node
            vtxs = capture_connected_nodes(
                sched,v,v->check_object_id(get_node_from_vtx(sched,v),o)
                )
            node_ids = map(v->get_vtx_id(sched,v),collect(vtxs))
            # @show node_ids
            v_deposit = filter(v->isa(get_node_from_vtx(sched,v),
            Union{BOT_DEPOSIT,TEAM_DEPOSIT}),
                collect(vtxs))[1]
            v_collect = filter(v->isa(get_node_from_vtx(sched,v),
            Union{BOT_COLLECT,TEAM_COLLECT}),
                collect(vtxs))[1]

            node = get_node_from_vtx(sched,v_deposit)
            if t >= cache.t0[v_deposit]
                object_positions[o] = OBJECT_AT(o,
                    map(n->get_location_id(n),sub_nodes(node))
                    )
            elseif t >= cache.t0[v_collect]
                r_ids = get_robot_ids(node)
                object_positions[o] = OBJECT_AT(o,
                    map(r->get_location_id(robot_positions[r]),r_ids)
                    )
            # else
            #     r_ids = get_robot_ids(node)
            #     @show object_positions[o] = OBJECT_AT(o,
            #         map(r->get_location_id(robot_positions[r]),r_ids)
            #         )
            end
            objects_active[o] = (t <= cache.tF[v_deposit])
        end
    end
    EnvState(
        robot_positions = robot_positions,
        object_positions = object_positions,
        objects_active = objects_active
    )
end

export get_node_start_and_end_times
"""
    get_start_and_end_maps(sched,cache,default=0)

Return dictionaries mapping each node id in schedule to its start and end time
"""
function get_node_start_and_end_times(sched::OperatingSchedule,cache::PlanningCache,default=0)
    t0 = Dict{AbstractID,Int}(get_vtx_id(sched, v)=>get(cache.t0, v, default) for v in vertices(sched))
    tF = Dict{AbstractID,Int}(get_vtx_id(sched, v)=>get(cache.tF, v, default) for v in vertices(sched))
    return t0,tF
end
function get_node_start_and_end_times(env::SearchEnv,args...)
    get_node_start_and_end_times(get_schedule(env),get_cache(env),args...)
end

export
    AbstractPC_MAPF,
    PC_MAPF,
    C_PC_MAPF,
    AbstractPC_TAPF,
    PC_TAPF,
    PC_TA,
    C_PC_TAPF

export
    construct_routing_problem

"""
    AbstractPC_MAPF

An abstract type of which all Precedence-Constrained Multi-Agent Path-Finding
problems are concrete subtypes.
"""
abstract type AbstractPC_MAPF <: AbstractMAPF end

"""
    AbstractPC_TAPF <: AbstractPC_MAPF

An abstract type of which all Precedence-Constrained Multi-Agent Task Assignment
and Path-Finding problems are concrete subtypes.
"""
abstract type AbstractPC_TAPF <: AbstractPC_MAPF end

"""
    PC_TAPF{E<:SearchEnv}

Precedence-Constrained Multi-Agent Task Assignment and Path-Finding problem.
"""
struct PC_TAPF{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end

"""
    PC_TA{E<:SearchEnv}

Precedence-Constrained Multi-Agent Task Assignment problem (no route planning).
"""
struct PC_TA{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end

"""
    `PC_MAPF`

A precedence-constrained multi-agent path-finding problem (no task assignment).
"""
struct PC_MAPF{E<:SearchEnv} <: AbstractPC_MAPF
    env::E
end
construct_routing_problem(prob::PC_TAPF,env) = PC_MAPF(env)

"""
    `C_PC_MAPF`

A collaborative precedence-constrained multi-agent path-finding problem. All
agents have assigned tasks, there are precedence constraints between tasks, and
some tasks must be done in teams.
"""
struct C_PC_MAPF{E<:SearchEnv} <: AbstractPC_MAPF
    env::E
end

"""
    C_PC_TAPF{L<:LowLevelSolution}

Defines an instance of a Collaborative Precedence-Constrained Multi-Agent Task
    Assignment and Path-Finding problem, where agents must sometimes transport
    objects in teams.
"""
struct C_PC_TAPF{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end
C_PC_TAPF(p::PC_TAPF) = C_PC_TAPF(p.env)
construct_routing_problem(prob::C_PC_TAPF,env) = C_PC_MAPF(env)

for T in [:PC_TAPF,:PC_MAPF,:PC_TA]
    @eval $T(prob::$T,env::SearchEnv) = $T(env)
end


initialize_planning_cache(env::SearchEnv) = initialize_planning_cache(get_schedule(env),deepcopy(get_cache(env).t0),deepcopy(get_cache(env).tF))

"""
    get_next_vtx_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_vtx_matching_agent_id(env::SearchEnv,agent_id)
    @assert isa(agent_id,BotID)
    for v in get_cache(env).active_set
        if agent_id == get_path_spec(get_schedule(env), v).agent_id
        # if agent_id == get_robot_id(get_node_from_vtx(get_schedule(env), v))
            return v
        end
    end
    return -1
end

"""
    get_next_node_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_node_matching_agent_id(env::SearchEnv,agent_id)
    @assert isa(agent_id,BotID)
    v = get_next_vtx_matching_agent_id(env,agent_id)
    if has_vertex(get_schedule(env),v)
        return get_vtx_id(get_schedule(env),v)
    end
    # return RobotID(agent_id)
    return agent_id
end

export
    update_planning_cache!

"""
    update_planning_cache!(solver,env,v,path)
"""
update_planning_cache!(solver,env::SearchEnv,v::Int,path::Path) = update_planning_cache!(solver,env,v,get_t(get_final_state(path)))
function update_planning_cache!(solver,env::SearchEnv,v::Int,t::Int=-1)
    cache = get_cache(env)
    schedule = get_schedule(env)
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)
    # update t0, tF, slack, local_slack
    Δt = t - cache.tF[v]
    if Δt > 0
        # delay = Δt - minimum(cache.slack[v])
        # if delay > 0
        #     @log_info(-1,solver.l3_verbosity,"LOW LEVEL SEARCH: schedule delay of ",delay," time steps incurred by path for vertex v = ",v," - ",string(get_node_from_vtx(schedule,v)), " - tF = ",get_final_state(path).t)
        #     # Backtracking
        #     tightenable_set = backtrack_deadlines(solver,cache,schedule,v)
        #     delay_vec = get_delay_vec(solver,cache,schedule,v)
        #     # @show delay_vec
        #     if any(delay_vec .> 0)
        #         for v_ in topological_sort(graph)
        #             if delay_vec[v_] > 0
        #                 @log_info(-1,solver,"ISPS: tightening at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
        #                 tighten_deadline!(solver,env,solution,v_)
        #                 break
        #             end
        #         end
        #     # if length(tightenable_set) > 0
        #     #     for v_ in tightenable_set
        #     #         @log_info(-1,solver,"ISPS: backtracking at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
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
        cache.tF[v] = t
        t0,tF,slack,local_slack = process_schedule(schedule,cache.t0,cache.tF)
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
    @log_info(2,solver,"moved ",v," to closed set, moved ",activated_vtxs," to active set")
    @log_info(3,solver,string("cache.tF[v] = ",cache.tF))
    return cache
end
"""
    update_planning_cache!(solver,env)

Update cache continually. After a call to this function, the start and end times
of all schedule nodes will be updated to reflect the progress of active schedule
nodes (i.e., if a robot had not yet completed a GO task, the predicted final
time for that task will be updated based on the robot's current state and
distance to the goal).
All active nodes that don't require planning will be automatically marked as
complete.
"""
function update_planning_cache!(solver,env)
    cache = get_cache(env)
    schedule = get_schedule(env)
    node = initialize_root_node(env)
    # dummy_path = path_type(env)()
    # Skip over nodes that don't need planning (either they have already been
    # planned, or they don't need planning period.)
    while true
        done = true
        for v in collect(cache.active_set)
            if get_path_spec(schedule,v).plan_path == false
                update_planning_cache!(solver,env,v) # NOTE I think this is all we need, since there is no actual path to update
                done = false
            end
        end
        # for (i,path) in enumerate(get_paths(env))
        #     cbs_env = build_env(solver,env,node,AgentID(i))
        #     sp = get_final_state(path)
        #     if is_goal(cbs_env,sp) && CRCBS.is_valid(cbs_env,get_goal(cbs_env))
        #         v = get_vtx(get_schedule(env),env.node_id)
        #         update_env!(solver,env,v,path)
        #         update_planning_cache!(solver,solution)
        #     end
        # end
        if done
            break
        end
    end
    for v in collect(cache.active_set)
        path_spec = get_path_spec(schedule,v)
        agent_id = get_id(path_spec.agent_id)
        if 1 <= agent_id <= num_agents(env)
            # @show string(get_node_from_vtx(get_schedule(env),v))
            path = get_paths(env)[agent_id]
            s = get_final_state(path)
            t = get_t(s)
            d = get_distance(env,s,State(path_spec.final_vtx,-1))
            cache.tF[v] = max(cache.tF[v],t+d)
        end
    end
    t0,tF,slack,local_slack = process_schedule(schedule,cache.t0,cache.tF)
    cache.t0 .= t0
    cache.tF .= tF
    cache.slack .= slack
    cache.local_slack .= local_slack

    cache
end

function CRCBS.SumOfMakeSpans(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    SumOfMakeSpans(
        cache.tF,
        schedule.terminal_vtxs,
        map(k->schedule.weights[k], schedule.terminal_vtxs),
        cache.tF[schedule.terminal_vtxs])
end
function CRCBS.MakeSpan(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    MakeSpan(
        cache.tF,
        schedule.terminal_vtxs,
        map(k->schedule.weights[k], schedule.terminal_vtxs),
        cache.tF[schedule.terminal_vtxs])
end


function initialize_route_plan(schedule::OperatingSchedule,cost_model)
    starts = State[]
    robot_ics = get_robot_ICs(schedule)
    for k in sort(collect(keys(robot_ics)))
        push!(starts, State(vtx = get_id(get_location_id(robot_ics[k])), t = 0))
    end
    # cost_model = get_cost_model(env)
    paths = map(s->Path{State,Action,cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end
function initialize_route_plan(env::SearchEnv)
    cost_model=get_cost_model(env)
    paths=deepcopy(get_paths(get_route_plan(env)))
    costs = map(p->get_cost(p), paths)
    LowLevelSolution(
        paths=paths,
        cost_model=cost_model,
        costs=costs,
        cost=aggregate_costs(cost_model, costs)
        )
end
function initialize_route_plan(env::SearchEnv,cost_model)
    paths = [Path(
        path_nodes=deepcopy(p.path_nodes),
        s0=p.s0,
        cost = compute_path_cost(cost_model,get_graph(env),p,i)
    ) for (i,p) in enumerate(get_paths(get_route_plan(env)))]
    costs=map(p->get_cost(p),paths)
    LowLevelSolution(
        paths=paths,
        cost_model=cost_model,
        costs=costs,
        cost=aggregate_costs(cost_model, costs)
        )
end
function construct_search_env(
        solver,
        schedule::OperatingSchedule,
        problem_spec::ProblemSpec,
        env_graph,
        cache::PlanningCache=initialize_planning_cache(schedule),
        primary_objective=problem_spec.cost_function,
        ;
        extra_T=400,
        kwargs...
    )
    cost_model, heuristic_model = construct_cost_model(
        solver,
        schedule,
        cache,
        problem_spec,
        env_graph,
        primary_objective;
        extra_T=extra_T,
        )
    layers = construct_environment_layer_dict(schedule,env_graph,problem_spec)
    route_plan = initialize_route_plan(schedule,cost_model)
    N = length(get_paths(route_plan))
    search_env = SearchEnv(
        schedule=schedule,
        cache=cache,
        env_layers=layers,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
        num_agents=N,
        route_plan=route_plan)
    return search_env
end
"""
    construct_search_env(solver,schedule,env,...)

Constructs a new search env by combining the new `schedule` with the pre-
existing `get_route_plan(env)`. This involves constructing a new cost function that
reflects the new schedule structure.
TODO: Carry over information about `get_cache(search_env)`
"""
function construct_search_env(
        solver,
        schedule::OperatingSchedule,
        env::SearchEnv,
        cache::PlanningCache=initialize_planning_cache(schedule,
            deepcopy(get_cache(env).t0), deepcopy(get_cache(env).tF)),
        env_graph=get_graph(env),
        problem_spec = get_problem_spec(env),
        primary_objective = problem_spec.cost_function,
        args...
        ;
        extra_T=400,
        kwargs...
    )
    cost_model, heuristic_model = construct_cost_model(
        solver,
        schedule,
        cache,
        problem_spec,
        env_graph,
        primary_objective;
        extra_T=extra_T,
        )
    layers = construct_environment_layer_dict(schedule,env_graph,problem_spec)
    route_plan = initialize_route_plan(env,cost_model)
    N = length(get_paths(route_plan))
    search_env = SearchEnv(
        schedule=schedule,
        cache=cache,
        env_layers=layers,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
        num_agents=N,
        route_plan=route_plan)
end

update_cost_model!(model::C,env::S) where {C,S<:SearchEnv} = nothing
function update_cost_model!(model::C,env::S) where {C<:MultiDeadlineCost,S<:SearchEnv}
    model.tF .= get_cache(env).tF
end
function update_cost_model!(model::C,env::S) where {C<:CompositeCostModel,S<:SearchEnv}
    for m in model.cost_models
        update_cost_model!(m,env)
    end
end
update_cost_model!(env::S) where {S<:SearchEnv} = update_cost_model!(get_cost_model(env),env)

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver,env::SearchEnv,v::Int,path::P,
        agent_id::Int=get_id(get_path_spec(get_schedule(env),v).agent_id)
        ) where {P<:Path}
    route_plan = get_route_plan(env)
    cache = get_cache(env)
    schedule = get_schedule(env)
    # UPDATE CACHE
    update_planning_cache!(solver,env,v,get_t(get_final_state(path)))
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
        # @log_info(3,solver,
        #     "Adding vertex list path to conflict cost model for agent ",
        #     agent_id,": ",
        #     convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
        partially_set_path!(get_cost_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
    end

    env
end

################################################################################
############################### Low Level Search ###############################
################################################################################

include("pccbs.jl")

export evaluate_path_gap

"""
    evaluate_path_gap(search_env::SearchEnv,path,v)

Returns the gap between a path's length and it's expected length (based on times
stored in `get_cache(env).t0`)
"""
function evaluate_path_gap(search_env::SearchEnv,path,v)
    t0 = get_cache(search_env).t0[v]
    gap = t0 - get_end_index(path)
    return gap
end

function replan_path!(solver, pc_mapf::AbstractPC_MAPF, env::SearchEnv, node::ConstraintTreeNode, vtx::VtxID,t0)
    v = get_id(vtx)
    sched = get_schedule(env)
    node_id = get_vtx_id(sched,v)
    schedule_node = get_node_from_id(sched,node_id)
    ### trim path
    cbs_env = build_env(solver,pc_mapf,env,node,VtxID(v))
    path = get_base_path(solver,env,cbs_env)
    # @log_info(-1,solver,"old path cost: ",get_cost(path))
    # @log_info(3,low_level(solver),"old path: \n",convert_to_vertex_lists(path))
    trim_path!(cbs_env,path,get_cache(env).t0[v])
    # @log_info(3,low_level(solver),"trimmed path: \n",convert_to_vertex_lists(path))
    # @log_info(-1,solver,"trimmed path cost: ",get_cost(path))
    ### plan path with new goal time
    status = plan_path!(low_level(solver),pc_mapf,env,node,schedule_node,v)
    return status
end

export tighten_gaps!

"""
    tighten_gaps!(solver, pc_mapf, env::SearchEnv, node::ConstraintTreeNode)

If any path ends before it should (based on times stored in `get_cache(env)`),
recomputes the path segment for the final node in that line.
"""
function tighten_gaps!(solver, pc_mapf::AbstractPC_MAPF, env::SearchEnv, node::ConstraintTreeNode)
    solver.tighten_paths ? nothing : return env
    sched = get_schedule(env)
    active_nodes = robot_tip_map(sched,get_cache(env).active_set)
    for (robot_id, node_id) in active_nodes
        path = get_paths(env)[get_id(robot_id)]
        v = get_vtx(sched,node_id)
        gap = evaluate_path_gap(env,path,v)
        if gap > 0
            t0 = get_cache(env).t0[v]
            @log_info(2, solver, "tighten_gaps!: base path for ",
                string(get_node_from_vtx(sched,v)),
                ", v = ",v," ends at t=",get_end_index(path),
                " but should end at t=",t0," (gap = ", gap,").")
            vtxs = backtrack_node(sched,v)
            for vp in vtxs
                if get_cache(env).tF[vp] < t0
                    @log_info(2,solver," Re-launching planner on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0," ...")
                    if replan_path!(solver, pc_mapf, env, node, VtxID(vp),t0)
                        @log_info(2,solver,"tightening succeeded on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0)
                    else
                        @log_info(2,solver,"tightening failed on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0)
                        return false
                    end
                end
            end
        end
    end
    return true
end

function get_base_path(solver,search_env::SearchEnv,env::PCCBSEnv)
    base_path = get_paths(search_env)[get_agent_id(env)]
    return base_path
end
function CRCBS.build_env(
        solver,
        pc_mapf::AbstractPC_MAPF,
        env::SearchEnv,
        node::ConstraintTreeNode,
        schedule_node::T,
        v::Int,
        path_spec=get_path_spec(get_schedule(env), v),
        ;
        heuristic = get_heuristic_model(env),
        cost_model = get_cost_model(env)
    ) where {T}
    agent_id = get_id(path_spec.agent_id)
    goal_vtx = path_spec.final_vtx
    goal_time = get_cache(env).tF[v] # time after which goal can be satisfied
    # deadline = get_cache(env).tF[v] .+ min.(get_cache(env).max_deadline[v],get_cache(env).slack[v]) # NOTE iterative deadline tightening was causing problems with slack running out before the goal time, so this line has been replaced by the original
    # deadline = get_cache(env).tF[v] .+ min.(max.(get_cache(env).local_slack[v], get_cache(env).max_deadline[v]),get_cache(env).slack[v]) # This is a potential fix that would allow iterative tightening to keep working
    deadline = get_cache(env).tF[v] .+ get_cache(env).slack[v]         # deadline for DeadlineCost
    # Adjust deadlines if necessary:
    if path_spec.tight == true
        goal_time += minimum(get_cache(env).local_slack[v])
    end
    for v_next in outneighbors(get_graph(get_schedule(env)),v)
        if get_path_spec(get_schedule(env), v_next).static == true
            duration_next = get_path_spec(get_schedule(env),v_next).min_path_duration
            for c in sorted_state_constraints(env,get_constraints(node, agent_id)) #.sorted_state_constraints
                if get_sp(get_path_node(c)).vtx == goal_vtx
                    if 0 < get_time_of(c) - goal_time < duration_next
                        @log_info(1,solver,"extending goal_time for node ",v,
                            " from ",goal_time," to ",get_time_of(c),
                            " to avoid constraints")
                        goal_time = max(goal_time, get_time_of(c))
                    end
                end
            end
        end
    end
    if (path_spec.free == true) && is_terminal_node(get_graph(get_schedule(env)),v)
        goal_time = maximum(get_cache(env).tF)
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        @log_info(3,solver,string("BUILD ENV: ",string(schedule_node),
            " - setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    @assert goal_time != Inf "goal time set to $goal_time for node $(string(schedule_node))"
    cbs_env = PCCBSEnv(
        # graph       = get_graph(env),
        search_env = env,
        schedule_node = schedule_node,
        node_id     = get_vtx_id(get_schedule(env),v),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        constraints = get_constraints(node, agent_id), # agent_id represents the whole path
        goal        = State(goal_vtx,goal_time),
        # heuristic   = heuristic,
        # cost_model  = cost_model
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    @log_info(3,solver,"build_env: ",string(schedule_node),
        ", goal_time=",goal_time,", deadline=",deadline)
    return cbs_env
end
function CRCBS.build_env(
    solver,
    pc_mapf::AbstractPC_MAPF,
    env::SearchEnv,
    node::ConstraintTreeNode,
    v::VtxID,
    schedule_node=get_node_from_vtx(get_schedule(env),get_id(v)),
    args...
    ;
    kwargs...
    )
    id = get_id(v)
    env = build_env(solver,pc_mapf,env,node,schedule_node,id,args...;kwargs...)
end
function CRCBS.build_env(
        solver,
        pc_mapf::AbstractPC_MAPF,
        env::SearchEnv,
        node::ConstraintTreeNode,
        agent_id::AgentID
        )
    node_id = get_next_node_matching_agent_id(env,RobotID(get_id(agent_id)))
    build_env(solver,pc_mapf,env,node,VtxID(get_vtx(get_schedule(env),node_id)))
    # get_node_from_id(get_schedule(env),node_id),
end
CRCBS.build_env(env::SearchEnv) = PCCBSEnv(search_env=env)

function get_base_path(solver,search_env::SearchEnv,meta_env::MetaAgentCBS.AbstractMetaEnv)
    base_paths = map(env->get_base_path(solver,search_env,env),MetaAgentCBS.get_envs(meta_env))
    cost = aggregate_costs(get_cost_model(meta_env), map(get_cost,base_paths))
    MetaAgentCBS.construct_meta_path(base_paths,cost)
end

"""
    For COLLABORATIVE transport problems
"""
function CRCBS.build_env(
    solver,
    pc_mapf::C_PC_MAPF,
    env::E,
    node::N,
    schedule_node::T,
    v::Int,
    ;
    kwargs...) where {E<:SearchEnv,N<:ConstraintTreeNode,T}
    envs = []
    agent_idxs = Int[]
    for (i, sub_node) in enumerate(sub_nodes(schedule_node))
        ph = PerfectHeuristic(get_team_config_dist_function(get_graph(env),team_configuration(schedule_node),i))
        heuristic = construct_heuristic_model(solver,get_graph(env),ph)
        cbs_env = build_env(solver,PC_MAPF(get_env(pc_mapf)),env,node,VtxID(v),sub_node,generate_path_spec(get_schedule(env),get_problem_spec(env),sub_node);
            heuristic=heuristic,
            kwargs...
        )
        push!(envs, cbs_env)
        push!(agent_idxs, get_id(get_robot_id(sub_node)))
    end
    meta_env = MetaAgentCBS.construct_team_env(
        [envs...],
        agent_idxs,
        get_cost_model(env)
        )
    return meta_env
end

export
    reset_route_plan!

"""
    Helper to reset the solution in a constraint node between re-runs of ISPS
"""
function reset_route_plan!(node::N,base_route_plan) where {N<:ConstraintTreeNode}
    for (agent_id,path) in enumerate(get_paths(base_route_plan))
        set_solution_path!(node.solution, deepcopy(path), agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
    end
    set_cost!(node, get_cost(base_route_plan))
    node
end

################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

CRCBS.build_env(solver, pc_mapf::AbstractPC_MAPF, args...) = build_env(solver,pc_mapf,get_env(pc_mapf),args...)
CRCBS.build_env(prob::Union{PC_TAPF,PC_MAPF}) = build_env(get_env(prob))
CRCBS.get_initial_solution(pc_mapf::PC_MAPF) = get_env(pc_mapf)
function CRCBS.initialize_root_node(env::SearchEnv)
    solution = SearchEnv(
        env,
        cache=initialize_planning_cache(env),
        route_plan=initialize_route_plan(env)
        )
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            i=>discrete_constraint_table(env,i) for i in 1:num_agents(env)
            ),
        id = 1)
end
function CRCBS.discrete_constraint_table(env::SearchEnv,agent_id=-1,tf=2*maximum(get_cache(env).tF)+100*num_agents(env))
    discrete_constraint_table(num_states(env),num_actions(env),agent_id,tf)
end
CRCBS.initialize_root_node(solver,pc_mapf::AbstractPC_MAPF) = initialize_root_node(get_env(pc_mapf))
function Base.copy(env::SearchEnv)
    SearchEnv(
        env,
        cache=deepcopy(get_cache(env)),
        route_plan=deepcopy(get_route_plan(env))
        )
end
function CRCBS.default_solution(env::SearchEnv)
    solution = deepcopy(env)
    set_cost!(get_route_plan(solution),get_infeasible_cost(get_route_plan(solution)))
    solution, get_cost(solution)
end
CRCBS.default_solution(pc_mapf::M) where {M<:AbstractPC_MAPF} = default_solution(get_env(pc_mapf))
function CRCBS.cbs_update_conflict_table!(solver,mapf::AbstractPC_MAPF,node,constraint)
    search_env = node.solution
    idxs = collect(1:num_agents(search_env))
    t0 = max(minimum(get_cache(search_env).t0), 1) # This is particularly relevant for replanning, where we don't care to look for conflicts way back in the past.
    detect_conflicts!(node.conflict_table,get_route_plan(search_env),idxs,t0)
end
CRCBS.detect_conflicts!(table,env::SearchEnv,args...) = detect_conflicts!(table,get_route_plan(env),args...)

# CRCBS.serialize(pc_mapf::PC_MAPF,args...) = serialize(get_env(pc_mapf),args...)
# CRCBS.deserialize(pc_mapf::PC_MAPF,args...) = serialize(get_env(pc_mapf),args...)
CRCBS.get_env(pc_mapf::AbstractPC_MAPF)             = pc_mapf.env
for op in [
    :cost_type,:state_type,:action_type,:path_type,:num_states,:num_actions,
    :num_agents,:serialize,:deserialize,
    :discrete_constraint_table,
    ]
    @eval CRCBS.$op(prob::AbstractPC_MAPF,args...) = $op(get_env(prob),args...)
    # @eval CRCBS.$op(prob::AbstractPC_MAPF,args...) = $op(get_env(prob),args...)
end
CRCBS.base_env_type(pc_mapf::AbstractPC_MAPF)               = PCCBSEnv
CRCBS.base_env_type(pc_mapf::Union{C_PC_MAPF,C_PC_TAPF})    = MetaAgentCBS.TeamMetaEnv

include("legacy/pc_tapf_solver.jl")

# end # PathPlanning module
