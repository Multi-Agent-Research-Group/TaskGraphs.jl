module TaskGraphsCore

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using DataStructures
using JuMP
using TOML

using ..PlanningPredicates

export
    TaskGraphProblemSpec

"""
    `TaskGraphProblemSpec{G}`

    Elements:
    - N::Int - num robots
    - M::Int - num tasks
    - graph::G - delivery graph
    - Drs::Matrix{Float64} - distance matrix robot initial locations -> pickup stations
    - Dss::Matrix{Float64} - distance matrix piickup stations -> dropoff stations
    - Δt::Vector{Float64}  - durations of operations
    - tr0_::Dict{Int,Float64} - robot start times
    - to0_::Dict{Int,Float64} - object start times
"""
@with_kw struct TaskGraphProblemSpec{G}
    N::Int                  = 0 # num robots
    M::Int                  = 0 # num tasks
    graph::G                = DiGraph() # delivery graph
    D::Matrix{Float64}      = zeros(0,0) # full distance matrix
    Drs::Matrix{Float64}    = zeros(N+M,M) # distance matrix robot initial locations -> pickup stations
    Dss::Matrix{Float64}    = zeros(M,M) # distance matrix piickup stations -> dropoff stations
    Δt::Vector{Float64}     = Vector{Float64}() # durations of operations
    Δt_collect::Vector{Float64} = zeros(M) # duration of COLLECT operations
    Δt_deliver::Vector{Float64} = zeros(M) # duration of DELIVER operations
    tr0_::Dict{Int,Float64} = Dict{Int,Float64}() # robot start times
    to0_::Dict{Int,Float64} = Dict{Int,Float64}() # object start times
    root_nodes::Vector{Int} = sort(collect(get_all_root_nodes(graph)))
    weights::Dict{Int,Float64} = Dict{Int,Float64}(v=>1.0 for v in root_nodes)
    s0::Vector{Int} = zeros(M) # pickup stations for each task
    sF::Vector{Int} = zeros(M) # delivery station for each task
end

export
    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    get_duration_vector,
    read_project_spec,
    construct_operation

"""
    `ProjectSpec{G}`

    Defines a list of operations that must be performed in order to complete a
    specific project, in addition to the dependencies between those operations
"""
@with_kw struct ProjectSpec{G}
    initial_conditions::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    final_conditions::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = MetaDiGraph()
    M::Int                        = length(initial_conditions)
    weight::Float64               = 1.0
end
get_initial_nodes(tg::ProjectSpec) = setdiff(
    Set(collect(vertices(tg.graph))),collect(keys(tg.pre_deps)))
get_input_ids(op::Operation) = Set{Int}([get_id(get_object_id(p)) for p in op.pre])
get_output_ids(op::Operation) = Set{Int}([get_id(get_object_id(p)) for p in op.post])
get_pre_deps(tg::ProjectSpec, i::Int) = get(tg.pre_deps, i, Set{Int}())
get_post_deps(tg::ProjectSpec, i::Int) = get(tg.post_deps, i, Set{Int}())
get_num_delivery_tasks(spec::ProjectSpec) = length(collect(union(map(op->union(get_input_ids(op),get_output_ids(op)),spec.operations)...)))
function get_duration_vector(spec::P) where {P<:ProjectSpec}
    Δt = zeros(get_num_delivery_tasks(spec))
    for op in spec.operations
        for id in get_output_ids(op)
            Δt[id] = duration(op)
        end
    end
    Δt
end
function add_pre_dep!(tg::ProjectSpec, i::Int, op_idx::Int)
    push!(get!(tg.pre_deps, i, Set{Int}()),op_idx)
end
function add_post_dep!(tg::ProjectSpec, i::Int, op_idx::Int)
    push!(get!(tg.post_deps, i, Set{Int}()),op_idx)
end
function add_operation!(task_graph::ProjectSpec, op::Operation)
    G = task_graph.graph
    ops = task_graph.operations
    add_vertex!(G)
    push!(ops, op)
    op_id = length(ops)
    for id in get_output_ids(op)
        # object[id] is a product of operation[op_id]
        add_pre_dep!(task_graph, id, op_id)
        for op0_id in get_post_deps(task_graph, id)
            # for each operation that requires object[id]
            add_edge!(G, op_id, op0_id)
        end
    end
    for id in get_input_ids(op)
        # object[id] is a prereq for operation[op_id]
        add_post_dep!(task_graph, id, op_id)
        for op0_id in get_pre_deps(task_graph, id)
            # for each (1) operation that generates object[id]
            add_edge!(G, op0_id, op_id)
        end
    end
    task_graph
end
function construct_operation(spec::ProjectSpec, station_id, input_ids, output_ids, Δt)
    Operation(
        Set{OBJECT_AT}(map(o->get(spec.final_conditions, o, OBJECT_AT(o,station_id)), input_ids)),
        Set{OBJECT_AT}(map(o->get(spec.initial_conditions, o, OBJECT_AT(o,station_id)), output_ids)),
        Δt,
        StationID(station_id)
    )
end

# Some tools for writing and reading project specs
function TOML.parse(pred::OBJECT_AT)
    [get_id(get_object_id(pred)),get_id(get_location_id(pred))]
end
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->[get_id(get_object_id(pred)),get_id(get_location_id(pred))], collect(op.pre))
    toml_dict["post"] = map(pred->[get_id(get_object_id(pred)),get_id(get_location_id(pred))], collect(op.post))
    toml_dict["Δt"] = duration(op)
    return toml_dict
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    toml_dict["initial_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.initial_conditions)
    toml_dict["final_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.final_conditions)
    toml_dict
end
function read_project_spec(toml_dict::Dict)
    project_spec = ProjectSpec()
    for (k,arr) in toml_dict["initial_conditions"]
        object_id = arr[1]
        station_id = arr[2]
        project_spec.initial_conditions[object_id] = OBJECT_AT(object_id, station_id)
    end
    for (k,arr) in toml_dict["final_conditions"]
        object_id = arr[1]
        station_id = arr[2]
        project_spec.final_conditions[object_id] = OBJECT_AT(object_id, station_id)
    end
    for op_dict in toml_dict["operations"]
        op = Operation(
            pre     = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),op_dict["pre"])),
            post    = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),op_dict["post"])),
            Δt      = op_dict["Δt"]
            )
        add_operation!(project_spec, op)
    end
    return project_spec
end
function read_project_spec(io)
    read_project_spec(TOML.parsefile(io))
end

export
    SimpleProblemDef,
    read_problem_def

struct SimpleProblemDef
    project_spec::ProjectSpec
    r0::Vector{Int}
    s0::Vector{Int}
    sF::Vector{Int}
end

function TOML.parse(def::SimpleProblemDef)
    toml_dict = TOML.parse(def.project_spec)
    toml_dict["r0"] = def.r0
    toml_dict["s0"] = def.s0
    toml_dict["sF"] = def.sF
    toml_dict
end
function read_problem_def(toml_dict::Dict)
    SimpleProblemDef(
        read_project_spec(toml_dict),
        toml_dict["r0"],
        toml_dict["s0"],
        toml_dict["sF"]
    )
end
function read_problem_def(io)
    read_problem_def(TOML.parsefile(io))
end

export
    DeliveryTask,
    DeliveryGraph,
    construct_delivery_graph

"""
    `DeliveryTask`
"""
struct DeliveryTask
    o::Int
    s1::Int
    s2::Int
end

"""
    `DeliveryGraph{G}`
"""
struct DeliveryGraph{G}
    tasks::Vector{DeliveryTask}
    graph::G
end

"""
    `construct_delivery_graph(project_spec::ProjectSpec,M::Int)`

    Assumes that initial station ids correspond to object ids.
"""
function construct_delivery_graph(project_spec::ProjectSpec,M::Int)
    delivery_graph = DeliveryGraph(Vector{DeliveryTask}(),MetaDiGraph(M))
    for op in project_spec.operations
        # for i in get_input_ids(op)
        for pred in preconditions(op)
            i = get_id(get_object_id(pred))
            pre = project_spec.initial_conditions[i]
            post = project_spec.final_conditions[i]
            push!(delivery_graph.tasks,DeliveryTask(i,get_id(get_location_id(pre)),get_id(get_location_id(post))))
            for j in get_output_ids(op)
                add_edge!(delivery_graph.graph, i, j)
                # push!(delivery_graph.tasks,DeliveryTask(i,i,j))
            end
        end
    end
    delivery_graph
end

export
    PathSpec,
    generate_path_spec

@with_kw struct PathSpec
    # element             ::T             = nothing
    # op_duration         ::Int           = 0 # time between end of predecessor and start of current
    start_vtx           ::Int           = -1
    final_vtx           ::Int           = -1
    min_path_duration   ::Int           =  0 # duration

    agent_id            ::Int           = -1
    dummy_id            ::Int           = -1
    path_id             ::Int           = -1
    # object_id           ::Int           = -1
    # slack               ::Int           = 0.0
    # deadline            ::Int           = 0.0
end


export
    ProjectSchedule,
    get_graph,
    get_object_ICs,
    get_object_FCs,
    get_robot_ICs,
    get_actions,
    get_operations,
    get_vtx_ids,
    get_node_from_id,
    get_num_actions,
    get_num_operations,
    get_num_object_ICs,
    get_num_robot_ICs,
    get_num_vtxs,
    get_num_paths,
    get_next_path_id,
    get_completion_time,
    get_duration,
    get_vtx,
    get_vtx_id,
    get_path_spec,
    set_path_spec!,
    add_to_schedule!,
    construct_project_schedule,
    process_schedule

# abstract type AbstractProjectSchedule end
# struct CompositeProjectSchedule <: AbstractProjectSchedule
#     schedules::Vector{ProjectSchedule}
#     weights::Vector{Float64}
# end
"""
    `ProjectSchedule`
"""
@with_kw struct ProjectSchedule{G<:AbstractGraph}# <: AbstractProjectSchedule
    graph               ::G                 = MetaDiGraph()
    object_ICs          ::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    robot_ICs           ::Dict{Int,ROBOT_AT}  = Dict{Int,ROBOT_AT}()
    actions             ::Dict{Int,AbstractRobotAction} = Dict{Int,AbstractRobotAction}()
    operations          ::Dict{Int,Operation} = Dict{Int,Operation}()
    #
    completion_times    ::Vector{Float64} = Vector{Float64}()
    durations           ::Vector{Float64} = Vector{Float64}()
    completion_status   ::Vector{Bool}    = Vector{Bool}() # INACTIVE, ACTIVE, COMPLETE
    path_specs          ::Vector{PathSpec}= Vector{PathSpec}()
    #
    robot_id_map        ::Dict{Int,Int}   = Dict{Int,Int}() # maps dummy id to true id
    root_nodes          ::Vector{Int}     = Vector{Int}() # list of "project heads"
    weights             ::Dict{Int,Float64} = Dict{Int,Float64}() # weights corresponding to project heads
    path_id_to_vtx_map  ::Dict{Int,Int}   = Dict{Int,Int}() # maps path_id to vertex
    vtx_ids             ::Vector{AbstractID} = Vector{AbstractID}() # maps vertex to actual graph node
    object_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
    robot_vtx_map       ::Dict{Int,Int}   = Dict{Int,Int}()
    operation_vtx_map   ::Dict{Int,Int}   = Dict{Int,Int}()
    action_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
end
get_graph(schedule::P) where {P<:ProjectSchedule}       = schedule.graph
get_object_ICs(schedule::P) where {P<:ProjectSchedule}  = schedule.object_ICs
get_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = schedule.robot_ICs
get_actions(schedule::P) where {P<:ProjectSchedule}     = schedule.actions
get_operations(schedule::P) where {P<:ProjectSchedule}  = schedule.operations
get_vtx_ids(schedule::P) where {P<:ProjectSchedule}     = schedule.vtx_ids
get_root_nodes(schedule::P) where {P<:ProjectSchedule}  = schedule.root_nodes
get_root_node_weights(schedule::P) where {P<:ProjectSchedule}  = schedule.weights

get_node_from_id(schedule::P,id::ActionID) where {P<:ProjectSchedule}       = get_actions(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::OperationID) where {P<:ProjectSchedule}    = get_operations(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::ObjectID) where {P<:ProjectSchedule}       = get_object_ICs(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::RobotID) where {P<:ProjectSchedule}        = get_robot_ICs(schedule)[get_id(id)]

get_vtx(schedule::ProjectSchedule,i::ObjectID)      = get(schedule.object_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::RobotID)       = get(schedule.robot_vtx_map,       get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::ActionID)      = get(schedule.action_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::OperationID)   = get(schedule.operation_vtx_map,   get_id(i), -1)
get_vtx_id(schedule::P,v::Int) where {P<:ProjectSchedule} = schedule.vtx_ids[v]

get_num_actions(schedule::P) where {P<:ProjectSchedule}     = length(get_actions(schedule))
get_num_operations(schedule::P) where {P<:ProjectSchedule}  = length(get_operations(schedule))
get_num_object_ICs(schedule::P) where {P<:ProjectSchedule}  = length(get_object_ICs(schedule))
get_num_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = length(get_robot_ICs(schedule))
get_num_vtxs(schedule::P) where {P<:ProjectSchedule}        = nv(get_graph(schedule))
get_num_paths(schedule::P) where {P<:ProjectSchedule}       = get_num_actions(schedule) + get_num_robot_ICs(schedule)

get_next_path_id(schedule::P) where {P<:ProjectSchedule}    = length(schedule.path_id_to_vtx_map) + 1

get_completion_time(schedule::ProjectSchedule,i::Int) = schedule.completion_times[i]
function set_completion_time!(schedule::ProjectSchedule,i::Int,t)
    schedule.completion_times[i] = t
end
get_duration(schedule::ProjectSchedule,i::Int) = schedule.durations[i]
function set_duration!(schedule::ProjectSchedule,i::Int,t)
    schedule.duration[i] = t
end
get_completion_status(schedule::ProjectSchedule,i::Int) = schedule.completion_status[i]
function set_completion_status!(schedule::ProjectSchedule,i::Int,t)
    schedule.completion_status[i] = t
end
get_path_spec(schedule::P,i::Int) where {P<:ProjectSchedule} = schedule.path_specs[i]
function set_path_spec!(schedule::P,i::Int,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    schedule.path_specs[i] = spec
    schedule.path_id_to_vtx_map[spec.path_id] = i
end
function add_path_spec!(schedule::P,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    push!(schedule.path_specs, spec)
    if spec.path_id != -1
        schedule.path_id_to_vtx_map[spec.path_id] = nv(get_graph(schedule))
    end
    # push!(schedule.completion_times, 0.0)
    # push!(schedule.durations, 0.0)
end

function insert_to_vtx_map!(schedule::P,pred::OBJECT_AT,id::ObjectID,idx::Int) where {P<:ProjectSchedule}
    push!(schedule.vtx_ids, id)
    get_object_ICs(schedule)[get_id(id)] = pred
    schedule.object_vtx_map[get_id(id)] = idx
end
function insert_to_vtx_map!(schedule::P,pred::ROBOT_AT,id::RobotID,idx::Int) where {P<:ProjectSchedule}
    push!(schedule.vtx_ids, id)
    get_robot_ICs(schedule)[get_id(id)] = pred
    schedule.robot_vtx_map[get_id(id)] = idx
end
function insert_to_vtx_map!(schedule::P,op::Operation,id::OperationID,idx::Int) where {P<:ProjectSchedule}
    push!(schedule.vtx_ids, id)
    get_operations(schedule)[get_id(id)] = op
    schedule.operation_vtx_map[get_id(id)] = idx
end
function insert_to_vtx_map!(schedule::P,a::A,id::ActionID,idx::Int) where {P<:ProjectSchedule,A<:AbstractRobotAction}
    push!(schedule.vtx_ids, id)
    get_actions(schedule)[get_id(id)] = a
    schedule.action_vtx_map[get_id(id)] = idx
end

"""
    PathSpec provides details about the path that corresponds to this node in
    the schedule
"""
function generate_path_spec(schedule::P,spec::T,a::A) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    path_spec = PathSpec(
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=spec.D[s0,s], # TaskGraphProblemSpec distance matrix
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=schedule.robot_id_map[r]
        )
end
function generate_path_spec(schedule::P,spec::T,a::COLLECT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=spec.Δt_collect[o],
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=schedule.robot_id_map[r]
        )
end
function generate_path_spec(schedule::P,spec::T,a::DEPOSIT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=spec.Δt_deliver[o],
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=schedule.robot_id_map[r]
        )
end
function generate_path_spec(schedule::P,spec::T,pred::OBJECT_AT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    path_spec = PathSpec(
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0
        )
end
function generate_path_spec(schedule::P,spec::T,pred::ROBOT_AT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    r = get_id(get_robot_id(pred))
    if !(r in collect(keys(schedule.robot_id_map)))
        @show r
        @show schedule.robot_id_map
    end
    path_spec = PathSpec(
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,r)
        )
end
function generate_path_spec(schedule::P,spec::T,op::Operation) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    path_spec = PathSpec(
        start_vtx = -1,
        final_vtx = -1,
        min_path_duration=duration(op)
        )
end

function add_to_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,ID<:AbstractID}
    @assert get_vtx(schedule, id) == -1
    add_vertex!(get_graph(schedule))
    insert_to_vtx_map!(schedule,pred,id,nv(get_graph(schedule)))
    path_spec = generate_path_spec(schedule,spec,pred)
    add_path_spec!(schedule,path_spec)
    schedule
end

function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    success = add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    # @show success, get_vtx(schedule,id1), get_vtx(schedule,id2)
    schedule
end

"""
    `get_task_sequences`

    Returns a dict mapping robot i => task sequence. The Vector `assignments`
    must map task j -> robot i (including dummy robots).
"""
function get_task_sequences(N::Int,M::Int,assignments::Vector{Int})
    task_sequences = Dict{Int,Vector{Int}}()
    for i in 1:N
        robot_id = i
        seq = Vector{Int}()
        for j in 1:M
            if assignments[j] == robot_id
                push!(seq, j)
                robot_id = j + N
            end
        end
        task_sequences[i] = seq
    end
    task_sequences
end
"""
    `populate_agent_ids!`

    Fills the schedule's dictionary mapping path_id (including dummy robots) to
    real agent id.
"""
function populate_agent_ids!(robot_id_map::Dict{Int,Int},spec::T,assignments::Vector{Int}) where {T<:TaskGraphProblemSpec}
    N, M = spec.N, spec.M
    for agent_id in 1:N
        path_id = agent_id
        robot_id_map[path_id] = agent_id
        j = 1
        while j <= M
            if assignments[j] == path_id
                path_id = j + N
                robot_id_map[path_id] = agent_id
                j = 0
            end
            j += 1
        end
    end
    schedule
end
function populate_agent_ids!(schedule::P,spec::T,assignments::Vector{Int}) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    populate_agent_ids!(schedule.robot_id_map,spec,assignments)
end

"""
    `construct_project_schedule`

    Args:
    - `spec` - a ProjectSpec
    - `object_ICs` - a list of initial object locations
    - `robot_ICs` - a list of initial robot locations
    - `assignments` - a list of robot assignments. `assignments[i] == j` means
    that robot `i` is assigned to transport object `j`
"""
function construct_project_schedule(
    project_spec::P,
    problem_spec::T,
    object_ICs::Dict{Int,OBJECT_AT},
    object_FCs::Dict{Int,OBJECT_AT},
    robot_ICs::Dict{Int,ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,T<:TaskGraphProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    schedule = ProjectSchedule()
    populate_agent_ids!(schedule,problem_spec,assignments)
    graph = get_graph(schedule)
    # add object ICs to graph
    for (id, pred) in object_ICs
        add_to_schedule!(schedule, problem_spec, pred, get_object_id(pred))
    end
    # add robot ICs to graph
    for (id, pred) in robot_ICs
        # if (id in assignments)
        add_to_schedule!(schedule, problem_spec, pred, get_robot_id(pred))
        # end
    end
    M = length(object_ICs) # number of objects
    N = length(robot_ICs) - M # number of robots
    # add operations to graph
    # root_nodes = Set{Int}()
    for op_vtx in topological_sort(project_spec.graph)
        op = project_spec.operations[op_vtx]
        operation_id = OperationID(get_num_operations(schedule) + 1)
        add_to_schedule!(schedule, problem_spec, op, operation_id)
        v = nv(get_graph(schedule))
        for object_id in get_input_ids(op)
            if object_id in problem_spec.root_nodes
                push!(schedule.root_nodes, v)
                schedule.weights[v] = problem_spec.weights[object_id]
            end
        end
        for object_id in get_input_ids(op)
            # add action sequence
            robot_id = assignments[object_id]
            robot_pred = get_robot_ICs(schedule)[robot_id]
            robot_start_station_id = get_id(get_location_id(robot_pred))

            object_ic = get_object_ICs(schedule)[object_id]
            pickup_station_id = get_id(get_location_id(object_ic))

            object_fc = object_FCs[object_id]
            dropoff_station_id = get_id(get_location_id(object_fc))

            action_id = ActionID(get_num_actions(schedule) + 1)
            add_to_schedule!(schedule, problem_spec, GO(robot_id, robot_start_station_id, pickup_station_id), action_id)
            add_edge!(schedule, RobotID(robot_id), action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)
            add_edge!(schedule, ObjectID(object_id), action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)

            add_edge!(schedule, action_id, operation_id)
            new_robot_id = object_id + N
            # if (new_robot_id in assignments)
            #     @show new_robot_id
            add_edge!(schedule, action_id, RobotID(new_robot_id))
            # end
        end
        for object_id in get_output_ids(op)
            # add_to_schedule!(schedule, object_pred, object_id)
            add_edge!(schedule, operation_id, ObjectID(object_id))
        end
    end
    # # process schedule
    # t0 = zeros(Int,get_num_vtxs(schedule))
    # for v in vertices(get_graph(schedule))
    #     node = get_node_from_id(schedule, get_vtx_id(schedule, v))
    #     if typeof(node) <: ROBOT_AT
    #         r = get_id(robot_id(node))
    #         t0[v] = get(project_spec.tr0_, r, 0)
    #     end
    #     if typeof(node) <: OBJECT_AT
    #         r = get_id(object_id(node))
    #         t0[v] = get(project_spec.to0_, r, 0)
    #     end
    # end
    # t0,tF,slack,local_slack = process_schedule(schedule)
    # # Add dependencies between nodes that use the same station
    # for v in vertices(get_graph(schedule))
    #     node = get_node_from_id(schedule, get_vtx_id(schedule, v))
    #     if typeof(node) <: Union{COLLECT,DEPOSIT}
    #         for v2 in v:get_num_vtxs(schedule)
    #             node2 = get_node_from_id(schedule, get_vtx_id(schedule, v2))
    #             if typeof(node2) <: Union{COLLECT,DEPOSIT}
    #                 if station_id(node) == station_id(node2)
    #                     if t0[v] < t0[v2]
    #                         add_edge!(get_graph(schedule),v,v2)
    #                     else
    #                         add_edge!(get_graph(schedule),v2,v)
    #                     end
    #                 end
    #             end
    #         end
    #     end
    # end
    sort!(schedule.root_nodes)
    # identify root nodes and store their indices in the schedule (important for multi-headed projects)
    # root_nodes = get_all_root_nodes(get_graph(schedule))
    # for (idx,pred) in get_robot_ICs(schedule)
    #     setdiff!(root_nodes,get_vtx(schedule, RobotID(idx)))
    # end
    # for v in sort(collect(root_nodes))
    #     push!(schedule.root_nodes, v)
    # end
    return schedule
end
function construct_project_schedule(
    project_spec::P,
    problem_spec::T,
    robot_ICs::Dict{Int,ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,T<:TaskGraphProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    construct_project_schedule(
        project_spec,
        problem_spec,
        project_spec.initial_conditions,
        project_spec.final_conditions,
        robot_ICs,
        assignments
    )
end

"""
    `process_schedule(schedule::P) where {P<:ProjectSchedule}`

    Compute the optimistic start and end times, along with the slack associated
    with each vertex in the `schedule`. Slack for each vertex is represented as
    a vector in order to handle multi-headed projects.
"""
function process_schedule(schedule::P; t0=zeros(Int,get_num_vtxs(schedule)),
        tF=zeros(Int,get_num_vtxs(schedule))
    ) where {P<:ProjectSchedule}

    solution_graph = schedule.graph
    traversal = topological_sort_by_dfs(solution_graph)
    n_roots = length(schedule.root_nodes)
    slack = map(i->Inf*ones(n_roots), vertices(solution_graph))
    local_slack = map(i->Inf*ones(n_roots), vertices(solution_graph))
    # True terminal nodes
    for (i,v) in enumerate(schedule.root_nodes)
        slack[v] = Inf*ones(n_roots)
        slack[v][i] = 0 # only slack for corresponing head is set to 0
    end
    ########## Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = schedule.path_specs[v]
        for v2 in inneighbors(solution_graph,v)
            t0[v] = max(t0[v], tF[v2])
        end
        tF[v] = max(tF[v], t0[v] + path_spec.min_path_duration)
    end
    ########### Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in outneighbors(solution_graph,v)
            local_slack[v] = min.(local_slack[v], (t0[v2] - tF[v]) )
            slack[v] = min.(slack[v], slack[v2] .+ (t0[v2] - tF[v]) )
        end
    end
    t0,tF,slack,local_slack
end

export
    get_collect_node,
    get_deposit_node,
    add_job_shop_constraints!

function get_collect_node(schedule::P,id::ObjectID) where {P<:ProjectSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != COLLECT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end
function get_deposit_node(schedule::P,id::ObjectID) where {P<:ProjectSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != DEPOSIT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end

function add_job_shop_constraints!(schedule::P,spec::T,model::JuMP.Model) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    M = spec.M
    s0 = spec.s0
    sF = spec.sF
    tor = Int.(round.(value.(model[:tor]))) # collect begin time
    toc = Int.(round.(value.(model[:toc]))) # collect end time
    tod = Int.(round.(value.(model[:tod]))) # deposit begin time
    tof = Int.(round.(value.(model[:tof]))) # deposit end time
    for j in 1:M
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                if t1[2] < t2[1]
                    add_edge!(schedule, id1, id2)
                elseif t2[2] < t1[1]
                    add_edge!(schedule, id2, id1)
                else
                    throw(ErrorException("JOB SHOP CONSTRAINT VIOLATED"))
                end
            end
        end
    end
end


end # module TaskGraphCore
