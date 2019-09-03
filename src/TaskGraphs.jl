module TaskGraphs

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP
using TOML

include("planning_predicates.jl")

export
    Operation,
    construct_operation,
    get_o,
    get_s,
    preconditions,
    postconditions,
    duration,

    DeliveryTask,
    DeliveryGraph,
    construct_delivery_graph,

    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    read_project_spec


@with_kw struct Operation
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Float64             = 0
    station_id::StationID   = StationID(-1)
end
function construct_operation(station_id, input_ids, output_ids, Δt)
    Operation(
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), input_ids)),
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), output_ids)),
        Δt,
        StationID(station_id)
    )
end
get_s(op::Operation) = op.station_id
duration(op::Operation) = op.Δt
preconditions(op::Operation) = op.pre
postconditions(op::Operation) = op.post

"""
    `ProjectSpec{G}`

    Defines a list of operations that must be performed in order to complete a
    specific project, in addition to the dependencies between those operations
"""
@with_kw struct ProjectSpec{G}
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = DiGraph()
    M::Int                        = -1
end
get_initial_nodes(tg::ProjectSpec) = setdiff(
    Set(collect(vertices(tg.graph))),collect(keys(tg.pre_deps)))
get_input_ids(op::Operation) = Set{Int}([get_id(get_o(p)) for p in op.pre])
get_output_ids(op::Operation) = Set{Int}([get_id(get_o(p)) for p in op.post])
get_pre_deps(tg::ProjectSpec, i::Int) = get(tg.pre_deps, i, Set{Int}())
get_post_deps(tg::ProjectSpec, i::Int) = get(tg.post_deps, i, Set{Int}())
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

# Some tools for writing and reading project specs
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->[get_id(get_o(pred)),get_id(get_s(pred))], collect(op.pre))
    toml_dict["post"] = map(pred->[get_id(get_o(pred)),get_id(get_s(pred))], collect(op.post))
    toml_dict["Δt"] = duration(op)
    return toml_dict
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["title"]      = "ProjectSpec"
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    toml_dict
end
function read_project_spec(io)
    toml_dict = TOML.parsefile(io)
    project_spec = ProjectSpec()
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
        for i in get_input_ids(op)
            for j in get_output_ids(op)
                add_edge!(delivery_graph.graph, i, j)
                push!(delivery_graph.tasks,DeliveryTask(i,i,j))
            end
        end
    end
    delivery_graph
end

export
    ProjectSchedule,
    construct_project_schedule

"""
    `ProjectSchedule`
"""
@with_kw struct ProjectSchedule{G<:AbstractGraph}
    graph               ::G                 = MetaDiGraph()
    object_ICs          ::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    robot_ICs           ::Dict{Int,ROBOT_AT}  = Dict{Int,ROBOT_AT}()
    actions             ::Dict{Int,AbstractRobotAction} = Dict{Int,AbstractRobotAction}()
    operations          ::Dict{Int,Operation} = Dict{Int,Operation}() # first num_operations vtxs all belong to operations
    #
    completion_times    ::Vector{Float64} = Vector{Float64}()
    durations           ::Vector{Float64} = Vector{Float64}()
    #
    object_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
    robot_vtx_map       ::Dict{Int,Int}   = Dict{Int,Int}()
    operation_vtx_map   ::Dict{Int,Int}   = Dict{Int,Int}()
    action_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
end
get_graph(schedule::P) where {P<:ProjectSchedule} = schedule.graph
get_object_ICs(schedule::P) where {P<:ProjectSchedule} = schedule.object_ICs
get_robot_ICs(schedule::P) where {P<:ProjectSchedule} = schedule.robot_ICs
get_actions(schedule::P) where {P<:ProjectSchedule} = schedule.actions
get_operations(schedule::P) where {P<:ProjectSchedule} = schedule.operations
get_num_actions(schedule::P) where {P<:ProjectSchedule} = length(get_actions(schedule))
get_num_operations(schedule::P) where {P<:ProjectSchedule} = length(get_operations(schedule))
get_num_object_ICs(schedule::P) where {P<:ProjectSchedule} = length(get_object_ICs(schedule))
get_num_robot_ICs(schedule::P) where {P<:ProjectSchedule} = length(get_robot_ICs(schedule))
get_completion_time(schedule::ProjectSchedule,i::Int) = schedule.completion_times[i]
function set_completion_time!(schedule::ProjectSchedule,i::Int,t::Int)
    schedule.completion_times[i] = t
end
get_duration(schedule::ProjectSchedule,i::Int) = schedule.durations[i]
const DEFAULT_ORDER = (:objects,:robots,:operations,:actions)
get_vtx(schedule::ProjectSchedule,i::ObjectID)      = get(schedule.object_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::RobotID)       = get(schedule.robot_vtx_map,       get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::ActionID)      = get(schedule.action_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::OperationID)   = get(schedule.operation_vtx_map,   get_id(i), -1)
function add_to_schedule!(schedule::P,pred::OBJECT_AT,id::ObjectID) where {P<:ProjectSchedule}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    set_prop!(graph,nv(graph),:vtype,:object_ic)
    get_object_ICs(schedule)[get_id(id)] = pred
    schedule.object_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,pred::ROBOT_AT,id::RobotID) where {P<:ProjectSchedule}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    set_prop!(graph,nv(graph),:vtype,:robot_ic)
    get_robot_ICs(schedule)[get_id(id)] = pred
    schedule.robot_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,op::Operation,id::OperationID) where {P<:ProjectSchedule}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    set_prop!(graph,nv(graph),:vtype,:operation)
    schedule.operations[get_id(id)] = op
    schedule.operation_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,a::A,id::ActionID) where {P<:ProjectSchedule,A<:AbstractRobotAction}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    set_prop!(graph,nv(graph),:vtype,:action)
    schedule.actions[get_id(id)] = a
    schedule.action_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    schedule
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
function construct_project_schedule(spec::P,
    object_ICs::Vector{OBJECT_AT},
    robot_ICs::Vector{ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    schedule = ProjectSchedule()
    graph = get_graph(schedule)
    # add object ICs to graph
    for pred in object_ICs
        add_to_schedule!(schedule, pred, get_o(pred))
    end
    # add robot ICs to graph
    for pred in robot_ICs
        add_to_schedule!(schedule, pred, get_r(pred))
    end
    # add operations to graph
    for op_vtx in topological_sort(spec.graph)
        op = spec.operations[op_vtx]
        operation_id = OperationID(get_num_operations(schedule) + 1)
        add_to_schedule!(schedule, op, operation_id)
        for object_id in get_input_ids(op)
            # add action sequence
            robot_id = assignments[object_id]
            robot_pred = get_robot_ICs(schedule)[object_id]
            object_pred = get_object_ICs(schedule)[object_id]

            pickup_station_id = get_id(get_s(object_pred))
            dropoff_station_id = get_id(get_s(op))
            action_id = ActionID(get_num_actions(schedule))
            add_to_schedule!(schedule, GO(robot_id, pickup_station_id), action_id)
            action_id += 1
            add_edge!(schedule, ObjectID(object_id), action_id)
            # add_edge!(schedule, RobotID(object_id), ActionID(get_num_actions(schedule)))

            add_to_schedule!(schedule, COLLECT(robot_id, object_id, pickup_station_id), action_id)
            add_edge!(schedule, action_id, action_id+1)
            action_id += 1

            add_to_schedule!(schedule, CARRY(robot_id, object_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id, action_id+1)
            action_id += 1

            add_to_schedule!(schedule, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id, action_id + 1)
            action_id += 1
            add_edge!(schedule, action_id, operation_id)
        end
        for j in get_output_ids(op)
            robot_pred = ROBOT_AT(-1,-1)
            robot_id = get_id(get_r(robot_pred))
            object_pred = OBJECT_AT(j, get_id(get_s(op)))
            object_id = ObjectID(j)

            add_to_schedule!(schedule, object_pred, object_id)
            add_edge!(schedule, operation_id, object_id)
        end
    end
    return schedule
end


include("utils.jl")

end
