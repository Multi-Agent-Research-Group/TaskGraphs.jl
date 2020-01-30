module TaskGraphsUtils

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP, MathOptInterface, Gurobi
using Random

using ..TaskGraphs
using CRCBS
# using ..FactoryWorlds
# using ..PlanningPredicates
# using ..TaskGraphsCore

export
    # validate,
    exclude_solutions!,
    cached_pickup_and_delivery_distances,
    construct_task_graphs_problem

# function get_station_precedence_dict(model::M) where {M<:JuMP.Model} end
# function validate(path::Path, msg::String)
#     @assert( !any(convert_to_vertex_lists(path) .== -1), msg )
#     return true
# end
# function validate(path::Path, v::Int)
#     validate(path, string("invalid path with -1 vtxs: v = ",v,", path = ",convert_to_vertex_lists(path)))
# end
# function validate(path::Path, v::Int, cbs_env)
#     validate(path, string("v = ",v,", path = ",convert_to_vertex_lists(path),", goal: ",cbs_env.goal))
# end
# function validate(project_schedule::ProjectSchedule)
#     G = get_graph(project_schedule)
#     try
#         @assert !is_cyclic(G) "is_cyclic(G)"
#         for (id,node) in project_schedule.planning_nodes
#             if typeof(node) <: COLLECT
#                 @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
#             end
#         end
#         for v in vertices(G)
#             node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#             @assert( outdegree(G,v) >= sum([0, values(required_successors(node))...]) )
#             @assert( indegree(G,v) >= sum([0, values(required_predecessors(node))...]) )
#         end
#     catch e
#         # if typeof(e) <: AssertionError
#         #     print(e.msg)
#         # else
#             throw(e)
#         # end
#         return false
#     end
#     return true
# end
# function validate(project_schedule::ProjectSchedule,paths::Vector{Vector{Int}},t0::Vector{Int},tF::Vector{Int})
#     G = get_graph(project_schedule)
#     for v in vertices(G)
#         node = get_node_from_vtx(project_schedule, v)
#         path_spec = get_path_spec(project_schedule, v)
#         agent_id = path_spec.agent_id
#         if agent_id != -1
#             path = paths[agent_id]
#             start_vtx = path_spec.start_vtx
#             final_vtx = path_spec.final_vtx
#             try
#                 if start_vtx != -1
#                     @assert(path[t0[v] + 1] == start_vtx, string("node: ",typeof(node), ", vtx: ",start_vtx, ", t+1: ",t0[v]+1,", path: ", path))
#                 end
#                 if final_vtx != -1
#                     @assert(path[tF[v] + 1] == final_vtx, string("node: ",typeof(node), ", vtx: ",start_vtx, ", t+1: ",t0[v]+1,", path: ", path))
#                 end
#             catch e
#                 # if typeof(e) <: AssertionError
#                 #     print(e.msg)
#                 # else
#                     throw(e)
#                 # end
#                 return false
#             end
#         end
#     end
#     return true
# end

export
    remap_object_id,
    remap_object_ids!

# utilities for remappingg object ids
remap_object_id(x,args...) = x
remap_object_id(id::ObjectID,max_obj_id) = ObjectID(get_id(id) + max_obj_id)
remap_object_id(node::OBJECT_AT,args...) = OBJECT_AT(remap_object_id(get_object_id(node),args...), get_initial_location_id(node))
function remap_object_ids!(node::Operation,args...)
    new_pre = Set([remap_object_id(o,args...) for o in node.pre])
    empty!(node.pre)
    union!(node.pre, new_pre)
    new_post = Set([remap_object_id(o,args...) for o in node.post])
    empty!(node.post)
    union!(node.post, new_post)
    node
end
function remap_object_ids!(project_schedule::ProjectSchedule,args...)
    for i in 1:length(project_schedule.vtx_ids)
        project_schedule.vtx_ids[i] = remap_object_id(project_schedule.vtx_ids[i],args...)
    end
    for dict in (project_schedule.vtx_map, project_schedule.planning_nodes)
        for k in collect(keys(dict))
            if typeof(k) <: ObjectID
                dict[remap_object_id(k,args...)] = remap_object_id(dict[k],args...)
                delete!(dict, k)
            end
        end
    end
    project_schedule
end

export
    exclude_solutions!,
    exclude_current_solution!

"""
    `exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})`

    This is the key utility for finding the next best solution to the MILP
    problem. It simply excludes every specific solution passed to it. It requires
    that X be a binary matrix.
"""
function exclude_solutions!(model::JuMP.Model,X::Matrix{Int})
    @assert !any((X .< 0) .| (X .> 1))
    @constraint(model, sum(model[:X] .* X) <= sum(model[:X])-1)
end
exclude_solutions!(model::TaskGraphsMILP,args...) = exclude_solutions!(model.model, args...)
function exclude_solutions!(model::JuMP.Model,M::Int,forbidden_solutions::Vector{Matrix{Int}})
    for X in forbidden_solutions
        exclude_solutions!(model,X)
    end
end
function exclude_solutions!(model::JuMP.Model)
    if termination_status(model) != MOI.OPTIMIZE_NOT_CALLED
        X = get_assignment_matrix(model)
        exclude_solutions!(model,X)
    end
end
exclude_current_solution!(args...) = exclude_solutions!(args...)

"""
    `cached_pickup_and_delivery_distances(r₀,oₒ,sₒ,dist=(x1,x2)->norm(x2-x1,1))`

    Inputs:
        `r₀` - vector of initial robot locations.
        `sₒ` - vector of initial object locations.
        `sₜ` - vector of station locations (object i must be brough to station i
            from its initial location)

    Outputs:
        `Drs` - distance from initial robot locations (including dummies) to
            object pickup locations
        `Dss` - distance from pickup stations to delivery stations (only the
            diagonal) is relevant for our problem
"""
function cached_pickup_and_delivery_distances(r₀,s₀,sₜ,dist=(x1,x2)->norm(x2-x1,1))
    N = size(r₀,1)
    M = size(s₀,1)
    # augment r₀ to include "dummy" robots that appear after dropoff
    r₀ = [r₀;sₜ]
    # Construct distance matrix
    Drs = zeros(N+M,M) # distance robot to pickup station
    for i in 1:N+M
        for j in 1:M
            Drs[i,j] = dist(r₀[i],s₀[j])
        end
    end
    Dss = zeros(M,M) # distance robot to delivery station
    for i in 1:M
        for j in 1:M
            # distance from dummy robot to object + object to station
            Dss[i,j] = dist(s₀[i],sₜ[j])
        end
    end
    return Drs, Dss
end

"""
    `construct_task_graphs_problem`
"""
function construct_task_graphs_problem(
        project_spec::P,
        r0::Vector{Int},
        s0::Vector{Int},
        sF::Vector{Int},
        dist_matrix,
        Δt_collect=zeros(length(s0)),
        Δt_deliver=zeros(length(sF));
        cost_function=SumOfMakeSpans,
        task_shapes=map(o->(1,1),s0),
        shape_dict=Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}()
        ) where {P<:ProjectSpec}
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    # r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    # project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    N = length(r0)
    M = length(s0)
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o, get(shape_dict[s0[o]], task_shapes[o], s0[o])) for o in 1:M]) # initial object conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,get(shape_dict[sF[o]], task_shapes[o], sF[o])) for o in 1:M]) # final object conditions
    new_project_spec = ProjectSpec(
        M=M,
        initial_conditions=object_ICs,
        final_conditions=object_FCs
    )
    for op in project_spec.operations
        add_operation!(
            new_project_spec,
            construct_operation(
                new_project_spec,
                op.station_id,
                map(o->get_id(get_object_id(o)), collect(op.pre)),
                map(o->get_id(get_object_id(o)), collect(op.post)),
                op.Δt
            )
        )
    end
    # M = length(object_ICs)
    # object_ICs = project_spec.initial_conditions
    # object_FCs = project_spec.final_conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N) # initial robot conditions
    for r in 1:M # dummy robots
        robot_ICs[r+N] = ROBOT_AT(r+N,sF[r])
    end

    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])

    delivery_graph = construct_delivery_graph(new_project_spec,M)
    # display(delivery_graph.tasks)
    G = delivery_graph.graph
    Δt = get_duration_vector(project_spec) # initialize vector of operation times
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_root_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    root_node_groups = map(v->get_input_ids(new_project_spec.operations[v]),collect(new_project_spec.root_nodes))
    problem_spec = ProblemSpec(N=N,M=M,graph=G,D=dist_matrix,Drs=Drs,
        Dss=Dss,Δt=Δt,tr0_=tr0_,to0_=to0_,root_nodes=root_node_groups,
        cost_function=cost_function,
        Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,r0=r0,s0=s0,sF=sF)
    # @show problem_spec.root_nodes
    return new_project_spec, problem_spec, object_ICs, object_FCs, robot_ICs
end
function construct_task_graphs_problem(
        operations::Vector{Operation},
        robot_ICs::Vector{ROBOT_AT},
        object_ICs::Vector{OBJECT_AT},
        object_FCs::Vector{OBJECT_AT},
        dist_function,
        Δt_collect=zeros(length(object_ICs)),
        Δt_deliver=zeros(length(object_ICs)),
        Δt_process=zeros(length(operations));
        cost_function=SumOfMakeSpans
        ) where {P<:ProjectSpec}
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    # r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    # project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    N = length(robot_ICs)
    M = length(object_ICs)
    r0 = map(pred->get_id(get_location_id(pred)),robot_ICs)
    s0 = map(pred->get_id(get_location_id(pred)),object_ICs)
    sF = map(pred->get_id(get_location_id(pred)),object_FCs)
    # object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial object conditions
    # object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final object conditions
    # M = length(object_ICs)
    # object_ICs = project_spec.initial_conditions
    # object_FCs = project_spec.final_conditions
    # robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N) # initial robot conditions
    for r in 1:M # dummy robots
        robot_ICs[r+N] = ROBOT_AT(r+N,sF[r])
    end

    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,dist_function)

    delivery_graph = construct_delivery_graph(project_spec,M)
    # display(delivery_graph.tasks)
    G = delivery_graph.graph
    # Δt = get_duration_vector(project_spec) # initialize vector of operation times
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_root_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    root_node_groups = map(v->get_input_ids(project_spec.operations[v]),collect(project_spec.root_nodes))
    problem_spec = ProblemSpec(N=N,M=M,graph=G,D=dist_matrix,Drs=Drs,
        Dss=Dss,Δt=Δt_process,tr0_=tr0_,to0_=to0_,root_nodes=root_node_groups,
        cost_function=cost_function,
        Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,r0=r0,s0=s0,sF=sF)
    # @show problem_spec.root_nodes
    return project_spec, problem_spec, object_ICs, object_FCs, robot_ICs
end

export
    construct_random_project_spec,
    get_random_problem_instantiation,
    construct_random_task_graphs_problem

"""
    construct_random_project_spec(M::Int;max_children=1)

    Inputs:
        `M` - number of objects involved in the operation
        `max_parents` - determines the max number of inputs to any operation
        `depth_bias` ∈ [0,1] - hyperparameter for tuning depth.
            If `depth_bias` == 1.0, the project_spec graph will always be depth
            balanced (all paths through the tree will be of the same length).
            For `depth_bias` == 0.0, the graph will be as "strung out" as
            possible.
"""
function construct_random_project_spec(M::Int,object_ICs::Vector{OBJECT_AT},object_FCs::Vector{OBJECT_AT};
        max_parents=1,
        depth_bias=1.0,
        Δt_min=0,
        Δt_max=0,
        zone_map=Dict{Int,Vector{Int}}(), # maps vtx to vector of vtxs for collaborative tasks
    )
    project_spec = ProjectSpec(
        M=M,
        initial_conditions=object_ICs,
        final_conditions=object_FCs
        )
    # fill with random operations going backwards
    i = M-1
    frontier = PriorityQueue{Int,Int}([M=>1])
    while i > 0
        depth = 1
        while true
            if (rand() > depth_bias) && (depth < length(frontier))
                depth += 1
            else
                break
            end
        end
        pairs = Vector{Pair{Int,Int}}()
        # pair = Pair{Int,Int}(0,0)
        for d in 1:depth
            push!(pairs, dequeue_pair!(frontier))
        end
        for p in pairs[1:end-1]
            enqueue!(frontier,p)
        end
        output_idx = pairs[end].first
        station_id = output_idx
        input_idxs = collect(max(1,1+i-rand(1:max_parents)):i)
        i = i - length(input_idxs)
        # Δt = Δt_min + (Δt_max-Δt_min)*rand()
        Δt=rand(Δt_min:Δt_max)
        input_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), input_idxs)
        output_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), [output_idx])
        add_operation!(project_spec,construct_operation(project_spec, station_id, input_ids, output_ids, Δt))
        for idx in input_idxs
            enqueue!(frontier, idx=>M-i)
        end
    end
    Δt=0
    final_idx = get_id(get_object_id(project_spec.initial_conditions[M]))
    add_operation!(project_spec,construct_operation(project_spec, -1, [final_idx], [], Δt))
    project_spec
end
# function construct_random_project_spec(M::Int,s0::Vector{Int},sF::Vector{Int};
function construct_random_project_spec(M::Int,s0::Vector{V},sF::Vector{V};kwargs...) where {V<:Union{Int,Vector{Int}}}
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(s0)])
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(sF)])
    construct_random_project_spec(M,object_ICs,object_FCs;
        kwargs...)
end

export
    choose_random_object_sizes,
    convert_to_collaborative

"""
    Tool for randomly selecting how many robots (and in what configuration)
        should deliver each task.
"""
function choose_random_object_sizes(M,probs::Dict{Tuple{Int,Int},Float64})
    k = sort(collect(keys(probs)))
    v = cumsum(map(key->probs[key], k))
    v = v / sum(collect(values(probs)))
    sizes = Vector{Tuple{Int,Int}}()
    for j in 1:M
        n = rand()
        i = 1
        while n > v[i]
            i += 1
        end
        push!(sizes,k[i])
    end
    sizes
end
function choose_random_object_sizes(M,probs::Dict{Int,Float64}=Dict(1=>1.0,2=>0.0,4=>0.0),choices=[(1,1),(2,1),(1,2),(2,2)])
    k = sort(collect(keys(probs)))
    size_probs = Dict(s=>probs[s[1]*s[2]] for s in choices)
    for ksize in k
        counts = sum(map(s->s[1]*s[2]==ksize, choices))
        for s in choices
            size_probs[s] = size_probs[s] / counts
        end
    end
    choose_random_object_sizes(M,size_probs)
end

"""
    convert_to_collaborative(project_spec)
"""
function convert_to_collaborative(project_spec::ProjectSpec,new_starts::Dict{ObjectID,Vector{Int}},new_goals::Dict{ObjectID,Vector{Int}})
    new_spec = ProjectSpec(
        M=project_spec.M,
        initial_conditions  = map(o->OBJECT_AT(get_object_id(o), new_starts[get_object_id(o)]), project_spec.initial_conditions),
        final_conditions    = map(o->OBJECT_AT(get_object_id(o), new_goals[get_object_id(o)]), project_spec.final_conditions),
    )
    for op in project_spec.operations
        add_operation!(
            new_spec,
            construct_operation(
                new_spec,
                op.station_id,
                map(o->get_id(get_object_id(o)), collect(op.pre)),
                map(o->get_id(get_object_id(o)), collect(op.post)),
                op.Δt
            )
        )
    end
    new_spec
end

"""
    `get_random_problem_instantiation`

    Args:
    - `N`: number of robots
    - `M`: number of delivery tasks
    - `robot_zones`: list of possible start locations for robots
    - `pickup_zones`: list of possible start locations for objects
    - `dropoff_zones`: list of possible destinations for objects
"""
function get_random_problem_instantiation(N::Int,M::Int,pickup_zones,dropoff_zones,robot_zones)
    ##### Random Problem Initialization #####
    r0 = robot_zones[sortperm(rand(length(robot_zones)))][1:N]
    s0 = pickup_zones[sortperm(rand(length(pickup_zones)))][1:M]
    sF = dropoff_zones[sortperm(rand(length(dropoff_zones)))][1:M]
    return r0,s0,sF
end

"""
    `construct_randomd_task_graphs_problem`
"""
function construct_random_task_graphs_problem(N::Int,M::Int,
    pickup_vtxs::Vector{Int},dropoff_vtxs::Vector{Int},free_vtxs::Vector{Int},dist_matrix,
    Δt_collect::Vector{Float64}=zeros(M),
    Δt_deliver::Vector{Float64}=zeros(M)
    )
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)

    construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
end

export
    initialize_test_problem

function initialize_test_problem(N=8,M=12;env_id=2)
    # experiment_dir = joinpath(dirname(pathof(WebotsSim)),"..","experiments")
    filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml");
    factory_env = read_env(filename);

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_random_task_graphs_problem(
        N,M,
        get_pickup_zones(factory_env),
        get_dropoff_zones(factory_env),
        get_free_zones(factory_env),
        get_dist_matrix(factory_env.graph));

    project_spec, problem_spec, robot_ICs, factory_env.graph
end


export
    combine_project_specs

"""
    `combine_project_specs(specs::Vector{ProjectSpec})`

    A helper for combining multiple `ProjectSpec`s into a single
    ProjectSpec.
"""
function combine_project_specs(specs::Vector{P}) where {P<:ProjectSpec}
    M = 0
    new_spec = ProjectSpec(
        initial_conditions=vcat(map(spec->spec.initial_conditions, specs)...),
        final_conditions=vcat(map(spec->spec.final_conditions, specs)...)
        )
    for spec in specs
        # spec_M = length(spec.initial_conditions)
        for op in spec.operations
            new_op = Operation(station_id=op.station_id, Δt = op.Δt)
            for pred in preconditions(op)
                push!(new_op.pre, OBJECT_AT(get_object_id(pred), get_location_id(pred)))
            end
            for pred in postconditions(op)
                push!(new_op.post, OBJECT_AT(get_object_id(pred), get_location_id(pred)))
            end
            add_operation!(new_spec, new_op)
        end
        # M = M + spec_M
    end
    new_spec
end

################################################################################
################################### Rendering ##################################
################################################################################

export
    title_string,
    get_display_metagraph

title_string(pred::OBJECT_AT,verbose=true) = verbose ? string("O",get_id(get_object_id(pred)),"-",get_id(get_location_id(pred))) : string("O",get_id(get_object_id(pred)));
title_string(pred::ROBOT_AT,verbose=true)  = verbose ? string("R",get_id(get_robot_id(pred)),"-",get_id(get_location_id(pred))) : string("R",get_id(get_robot_id(pred)));
title_string(a::GO,verbose=true)        = verbose ? string("go\n",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),",",get_id(get_destination_location_id(a))) : "go";
title_string(a::COLLECT,verbose=true)   = verbose ? string("collect\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "collect";
title_string(a::CARRY,verbose=true)     = verbose ? string("carry\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_destination_location_id(a))) : "carry";
title_string(a::DEPOSIT,verbose=true)   = verbose ? string("deposit\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "deposit";
title_string(op::Operation,verbose=true)= verbose ? string("op",get_id(get_operation_id(op))) : "op";
title_string(a::TEAM_ACTION{A},verbose=true) where {A} = verbose ? string("team", A, "\n","r: (",map(i->string(get_id(get_robot_id(i)), ","), a.instructions)...,")") : string("team", A)

Base.string(pred::OBJECT_AT) =  string("O(",get_id(get_object_id(pred)),",",get_id(get_location_id(pred)),")")
Base.string(pred::ROBOT_AT)  =  string("R(",get_id(get_robot_id(pred)),",",get_id(get_location_id(pred)),")")
Base.string(a::GO)        =  string("GO(",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),",",get_id(get_destination_location_id(a)),")")
Base.string(a::COLLECT)   =  string("COLLECT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(a::CARRY)     =  string("CARRY(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_destination_location_id(a)),")")
Base.string(a::DEPOSIT)   =  string("DEPOSIT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(op::Operation)=  string("OP(",get_id(get_operation_id(op)),")")
Base.string(a::TEAM_ACTION{A}) where {A} =  string("TEAM_ACTION( ",map(i->string(string(i), ","), a.instructions)...," )")

function get_display_metagraph(project_schedule::ProjectSchedule;
    verbose=true,
    f=(v,p)->title_string(p,verbose),
    object_color="orange",
    robot_color="lime",
    action_color="cyan",
    operation_color="red",
    remove_leaf_robots=false
    )
    graph = MetaDiGraph(deepcopy(project_schedule.graph))
    for (id,pred) in get_object_ICs(project_schedule)
        v = get_vtx(project_schedule, get_object_id(pred))
        set_prop!(graph, v, :vtype, :object_ic)
        set_prop!(graph, v, :text, f(v,pred))
        set_prop!(graph, v, :color, object_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,op) in get_operations(project_schedule)
        v = get_vtx(project_schedule, OperationID(id))
        set_prop!(graph, v, :vtype, :operation)
        set_prop!(graph, v, :text, f(v,op))
        set_prop!(graph, v, :color, operation_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,a) in get_actions(project_schedule)
        v = get_vtx(project_schedule, ActionID(id))
        set_prop!(graph, v, :vtype, :action)
        set_prop!(graph, v, :text, f(v,a))
        set_prop!(graph, v, :color, action_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,pred) in get_robot_ICs(project_schedule)
        v = get_vtx(project_schedule, get_robot_id(pred))
        set_prop!(graph, v, :vtype, :robot_ic)
        set_prop!(graph, v, :text, f(v,pred))
        set_prop!(graph, v, :color, robot_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    if remove_leaf_robots == true
        for v in reverse(vertices(graph))
            # if get_prop(graph,v,:vtype) ==:robot_ic
            if typeof(get_vtx_id(project_schedule,v)) <: RobotID
                if length(outneighbors(graph,v)) == 0
                    rem_vertex!(graph,v)
                end
            end
        end
    end
    graph
end

################################################################################
######################### COMPARISONS AND VERIFICATION #########################
################################################################################
function Base.:(==)(o1::OBJECT_AT,o2::OBJECT_AT)
    try
        @assert(o1.o == o2.o)
        @assert(o1.x == o2.x)
    catch e
        println(e.msg)
        # throw(e)
        return false
    end
    return true
end
function Base.:(==)(op1::Operation,op2::Operation)
    try
        @assert(op1.pre == op2.pre)
        @assert(op1.post == op2.post)
        @assert(op1.Δt == op2.Δt)
        @assert(op1.station_id == op2.station_id)
    catch e
        println(e.msg)
        # throw(e)
        return false
    end
    return true
end
function Base.:(==)(spec1::ProjectSpec,spec2::ProjectSpec)
    try
        @assert(spec1.initial_conditions == spec2.initial_conditions)
        @assert(spec1.final_conditions == spec2.final_conditions)
        @assert(spec1.operations == spec2.operations)
        @assert(spec1.pre_deps == spec2.pre_deps)
        @assert(spec1.graph == spec2.graph)
        @assert(spec1.root_nodes == spec2.root_nodes)
        @assert(spec1.weights == spec2.weights)
        @assert(spec1.weight == spec2.weight)
        @assert(spec1.object_id_to_idx == spec2.object_id_to_idx)
    catch e
        println(e.msg)
        # throw(e)
        return false
    end
    return true
end

end # module TaskGraphsUtils
