# module TaskGraphsCore

# using Parameters
# using LightGraphs, MetaGraphs
# using GraphUtils
# using DataStructures
# using JuMP
# using Gurobi
# using TOML
# using CRCBS
# using SparseArrays
#
# using ..TaskGraphs

function TOML.print(io,dict::Dict{Symbol,A}) where {A}
    TOML.print(io,Dict(string(k)=>v for (k,v) in dict))
end

export
    get_debug_file_id,
    reset_debug_file_id!

DEBUG_ID_COUNTER = 0
get_debug_file_id() = Int(global DEBUG_ID_COUNTER += 1)
function reset_debug_file_id!()
    global DEBUG_ID_COUNTER = 0
end

export validate
function validate end

export
    ProblemSpec

"""
    ProblemSpec{G}

Encodes the the relaxed PC-TAPF problem that ignores collision-avoidance
constraints.

Elements:
- N::Int - num robots
- M::Int - num tasks
- graph::G - task graph
- Δt::Vector{Float64}  - durations of operations
- tr0_::Dict{Int,Float64} - robot start times
- to0_::Dict{Int,Float64} - object start times
- terminal_vtxs::Vector{Set{Int}} - identifies "project heads"
- weights::Dict{Int,Float64} - stores weights associated with each project head
- cost_function::F - the optimization objective (default is SumOfMakeSpans)
- r0::Vector{Int} - initial locations for all robots
- s0::Vector{Int} - pickup stations for each task
- sF::Vector{Int} - delivery station for each task
- nR::Vector{Int} - num robots required for each task (>1 => collaborative task)
"""
@with_kw struct ProblemSpec{G,F,T}
    graph::G                = DiGraph() # delivery graph
    D::T                    = zeros(0,0) # full distance matrix
    Δt::Vector{Float64}     = Vector{Float64}() # durations of operations
    tr0_::Dict{Int,Float64} = Dict{Int,Float64}() # robot start times
    to0_::Dict{Int,Float64} = Dict{Int,Float64}() # object start times
    terminal_vtxs::Vector{Set{Int}} = [get_all_terminal_nodes(graph)]
    weights::Dict{Int,Float64} = Dict{Int,Float64}(v=>1.0 for v in 1:length(terminal_vtxs))
    cost_function::F        = SumOfMakeSpans()
    r0::Vector{Int}         = Int[]
    s0::Vector{Int}         = Int[] # pickup stations for each task
    sF::Vector{Int}         = Int[] # delivery station for each task
    # nR::Vector{Int}         = map(o->1,s0) # num robots required for each task (>1 => collaborative task)
    # N::Int                  = length(r0) # num robots
    # M::Int                  = length(s0) # num tasks
    Δt_collect::Vector{Float64} = zeros(length(s0)) # duration of COLLECT operations
    Δt_deliver::Vector{Float64} = zeros(length(s0)) # duration of DELIVER operations
    # @assert length(r0) == N
    @assert length(s0) == length(sF) == length(Δt_collect) == length(Δt_deliver) # == M
end
TaskGraphs.get_distance(spec::ProblemSpec,args...) = get_distance(spec.D,args...)

export
    ProjectSpec,
    get_initial_nodes,
    add_operation!,
    set_initial_condition!,
    set_final_condition!,
    get_initial_condition,
    get_final_condition,
    get_duration_vector,
    construct_operation

"""
    ProjectSpec{G}

Encodes a list of operations that must be performed in order to complete a
specific project, in addition to the dependencies between those operations.

Elements:
- initial_conditions::Vector{OBJECT_AT} - maps object id to initial condition predicate
- final_conditions::Vector{OBJECT_AT} - maps object id to final condition predicate
- operations::Vector{Operation} - list of manufacturing operations
- pre_deps::Dict{Int,Set{Int}} - maps object id to ids of operations that are required to produce that object
- post_deps::Dict{Int,Set{Int}} - maps object id to ids of operations that depend on that object
- graph::G
- terminal_vtxs::Set{Int}
- weights::Dict{Int,Float64}
- M::Int
- weight::Float64
- object_id_to_idx::Dict{Int,Int}
"""
@with_kw struct ProjectSpec{G}
    initial_conditions::Vector{OBJECT_AT} = Vector{OBJECT_AT}()
    final_conditions::Vector{OBJECT_AT} = Vector{OBJECT_AT}()
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = DiGraph()
    terminal_vtxs::Set{Int}          = Set{Int}()
    weights::Dict{Int,Float64}    = Dict{Int,Float64}(v=>1.0 for v in terminal_vtxs)
    weight::Float64               = 1.0
    M::Int                        = length(initial_conditions)
    object_id_to_idx::Dict{Int,Int} = Dict{Int,Int}(get_id(get_object_id(id))=>k for (k,id) in enumerate(initial_conditions))
    op_id_to_vtx::Dict{Int,Int}    = Dict{Int,Int}()
end
function ProjectSpec(ics::V,fcs::V) where {V<:Vector{OBJECT_AT}}
    ProjectSpec(initial_conditions=ics,final_conditions=fcs)
end
get_initial_nodes(spec::ProjectSpec) = setdiff(
    Set(collect(vertices(spec.graph))),collect(keys(spec.pre_deps)))
get_pre_deps(spec::ProjectSpec, op_id::Int) = get(spec.pre_deps, op_id, Set{Int}())
get_post_deps(spec::ProjectSpec, op_id::Int) = get(spec.post_deps, op_id, Set{Int}())
get_num_delivery_tasks(spec::ProjectSpec) = length(collect(union(map(op->union(get_input_ids(op),get_output_ids(op)),spec.operations)...)))
function get_duration_vector(spec::P) where {P<:ProjectSpec}
    Δt = zeros(get_num_delivery_tasks(spec))
    for op in spec.operations
        for id in get_output_ids(op)
            Δt[get_id(id)] = duration(op)
        end
    end
    Δt
end
function add_pre_dep!(spec::ProjectSpec, object_id::Int, op_id::Int)
    # operation[op_id] is a prereq of object[object_id]
    push!(get!(spec.pre_deps, object_id, Set{Int}()),op_id)
end
function add_post_dep!(spec::ProjectSpec, object_id::Int, op_id::Int)
    # operation[op_id] is a postreq of object[object_id]
    push!(get!(spec.post_deps, object_id, Set{Int}()),op_id)
end
function set_condition!(spec,object_id,pred,array)
    idx = get!(spec.object_id_to_idx,get_id(object_id),length(spec.object_id_to_idx)+1)
    @assert haskey(spec.object_id_to_idx, object_id)
    while idx > length(array)
        push!(array,OBJECT_AT(-1,-1))
    end
    @assert idx <= length(array)
    array[idx] = pred
    array
end
set_initial_condition!(spec,object_id,pred) = set_condition!(spec,object_id,pred,spec.initial_conditions)
set_final_condition!(spec,object_id,pred) = set_condition!(spec,object_id,pred,spec.final_conditions)
get_condition(spec,object_id,array) = get(array,get(spec.object_id_to_idx,get_id(object_id),-1),OBJECT_AT(object_id,-1))
get_initial_condition(spec,object_id) = get_condition(spec,object_id,spec.initial_conditions)
get_final_condition(spec,object_id) = get_condition(spec,object_id,spec.final_conditions)
function construct_operation(spec::ProjectSpec, station_id, input_ids, output_ids, Δt, id=get_unique_operation_id())
    op = Operation(
        # pre = Set{OBJECT_AT}(map(id->get(spec.final_conditions, spec.object_id_to_idx[id], OBJECT_AT(id,station_id)), input_ids)),
        # post = Set{OBJECT_AT}(map(id->get(spec.initial_conditions, spec.object_id_to_idx[id], OBJECT_AT(id,station_id)), output_ids)),
        pre = Set{OBJECT_AT}(map(id->spec.final_conditions[spec.object_id_to_idx[id]], input_ids)),
        post = Set{OBJECT_AT}(map(id->spec.initial_conditions[spec.object_id_to_idx[id]], output_ids)),
        Δt = Δt,
        station_id = LocationID(station_id),
        id = id
    )
end
function add_operation!(spec::ProjectSpec, op::Operation)
    G = spec.graph
    ops = spec.operations
    add_vertex!(G)
    push!(ops, op)
    spec.op_id_to_vtx[get_id(op)] = nv(G)
    op_id = length(ops)
    @assert op_id == nv(G)
    for pred in postconditions(op)
        object_id = get_id(get_object_id(pred))
        set_initial_condition!(spec,object_id,pred)
        set_final_condition!(spec,object_id,get_final_condition(spec,object_id))
        # object[id] is a product of operation[op_id]
        add_pre_dep!(spec, object_id, op_id)
        for op0_id in get_post_deps(spec, object_id)
            add_edge!(G, op_id, op0_id) # for each operation that requires object[id]
        end
    end
    for pred in preconditions(op)
        object_id = get_id(get_object_id(pred))
        set_initial_condition!(spec,object_id,get_initial_condition(spec,object_id))
        set_final_condition!(spec,object_id,pred)
        # object[id] is a prereq for operation[op_id]
        spec.final_conditions[spec.object_id_to_idx[object_id]]
        add_post_dep!(spec, object_id, op_id)
        for op0_id in get_pre_deps(spec, object_id)
            add_edge!(G, op0_id, op_id) # for each (1) operation that generates object[id]
        end
    end
    # set spec.terminal_vtxs = get_all_terminal_nodes(spec.graph)
    union!(spec.terminal_vtxs, get_all_terminal_nodes(spec.graph))
    intersect!(spec.terminal_vtxs, get_all_terminal_nodes(spec.graph))
    validate(spec,op)
    spec
end

export
    read_operation,
    read_project_spec

# Some tools for writing and reading project specs
# function TOML.parse(pred::OBJECT_AT)
#     [get_id(get_object_id(pred)),get_id(get_location_id(pred))]
# end
# parse_object(arr,dict=Dict{String,Any}()) = merge!(dict,Dict("type"=>"OBJECT_AT","data"=>[arr[1],arr[2]]))
# read_object(arr) = OBJECT_AT(arr[1],arr[2])

parse_object(pred::OBJECT_AT,dict=Dict{String,Any}()) = merge!(dict,Dict("type"=>"OBJECT_AT","data"=>[get_id(get_object_id(pred)),get_id(get_location_id(pred))]))
TOML.parse(pred::OBJECT_AT) = parse_object(pred)
read_object(dict) = OBJECT_AT(dict["data"]...)
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->TOML.parse(pred), collect(op.pre))
    toml_dict["post"] = map(pred->TOML.parse(pred), collect(op.post))
    toml_dict["dt"] = duration(op)
    toml_dict["station_id"] = get_id(op.station_id)
    toml_dict["id"] = get_id(op.id)
    return toml_dict
end
function read_operation(toml_dict::Dict,keep_id=false)
    op_id = OperationID(get(toml_dict,"id",-1))
    if get_id(op_id) == -1 || keep_id == false
        op_id = get_unique_operation_id()
    end
    op = Operation(
        pre     = Set{OBJECT_AT}(map(arr->read_object(arr),toml_dict["pre"])),
        post    = Set{OBJECT_AT}(map(arr->read_object(arr),toml_dict["post"])),
        Δt      = get(toml_dict,"dt",get(toml_dict,"Δt",0.0)),
        station_id = LocationID(get(toml_dict,"station_id",-1)),
        id = op_id
        )
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    # toml_dict["initial_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.initial_conditions)
    # toml_dict["final_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.final_conditions)
    toml_dict["initial_conditions"] = map(pred->TOML.parse(pred), project_spec.initial_conditions)
    toml_dict["final_conditions"] = map(pred->TOML.parse(pred), project_spec.final_conditions)
    toml_dict
end
function read_project_spec(toml_dict::Dict)
    project_spec = ProjectSpec()
    ics = toml_dict["initial_conditions"]
    fcs = toml_dict["final_conditions"]
    ics = isa(ics,Dict) ? collect(values(ics)) : ics
    fcs = isa(fcs,Dict) ? collect(values(fcs)) : fcs
    for arr in ics
        push!(project_spec.initial_conditions, read_object(arr))
    end
    object_ids = map(get_object_id, project_spec.initial_conditions)
    for (i, object_id) in enumerate(sort(object_ids))
        project_spec.object_id_to_idx[get_id(object_id)] = i
    end
    for arr in fcs
        push!(project_spec.final_conditions, read_object(arr))
    end
    for op_dict in toml_dict["operations"]
        op = read_operation(op_dict)
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

@with_kw struct SimpleProblemDef
    project_spec::ProjectSpec       = ProjectSpec()
    r0::Vector{Int}                 = Int[]
    s0::Vector{Int}                 = Int[]
    sF::Vector{Int}                 = Int[]
    shapes::Vector{Tuple{Int,Int}}  = Vector{Tuple{Int,Int}}([(1,1) for o in s0])
end
SimpleProblemDef(project_spec,r0,s0,sF) = SimpleProblemDef(project_spec=project_spec,r0=r0,s0=s0,sF=sF)

function TOML.parse(def::SimpleProblemDef)
    toml_dict = TOML.parse(def.project_spec)
    toml_dict["r0"] = def.r0
    toml_dict["s0"] = def.s0
    toml_dict["sF"] = def.sF
    toml_dict["shapes"] = map(s->[s...], def.shapes)
    toml_dict
end
function read_problem_def(toml_dict::Dict)
    SimpleProblemDef(
        read_project_spec(toml_dict),
        toml_dict["r0"],
        toml_dict["s0"],
        toml_dict["sF"],
        # map(s->tuple(s...), toml_dict["shapes"])
        map(s->tuple(s...), get(toml_dict,"shapes",[[1,1] for o in toml_dict["s0"]]) )
    )
end
function read_problem_def(io)
    read_problem_def(TOML.parsefile(io))
end

export
    DeliveryTask,
    DeliveryGraph


"""
    DeliveryTask
"""
struct DeliveryTask
    o::Int
    s1::Int
    s2::Int
end

"""
    DeliveryGraph{G}
"""
struct DeliveryGraph{G}
    tasks::Vector{DeliveryTask}
    graph::G
end

export
    construct_delivery_graph

"""
    construct_delivery_graph(project_spec::ProjectSpec,M::Int)

Assumes that initial station ids correspond to object ids.
"""
function construct_delivery_graph(project_spec::ProjectSpec,M::Int)
    delivery_graph = DeliveryGraph(Vector{DeliveryTask}(),MetaDiGraph(M))
    for op in project_spec.operations
        for pred in preconditions(op)
            id = get_id(get_object_id(pred))
            idx = project_spec.object_id_to_idx[id]
            pre = get_initial_condition(project_spec,id)
            post = get_final_condition(project_spec,id)
            push!(delivery_graph.tasks,DeliveryTask(id,get_id(get_location_id(pre)),get_id(get_location_id(post))))
            for j in get_output_ids(op)
                idx2 = project_spec.object_id_to_idx[get_id(j)]
                add_edge!(delivery_graph.graph, idx, idx2)
            end
        end
    end
    delivery_graph
end

export
    PathSpec,
    generate_path_spec

"""
    PathSpec

Encodes information about the path that must be planned for a particular
schedule node.

Fields:
* `node_type::Symbol = :EMPTY`
* `start_vtx::Int = -1`
* `final_vtx::Int = -1`
* `min_path_duration::Int = 0`
* `agent_id::Int = -1`
* `object_id::Int = -1`
* `plan_path::Bool = true` - flag indicating whether a path must be planned.
    For example, `Operation` nodes do not require any path planning.
* `tight::Bool = false` - if true, the path may not terminate prior to the
    beginning of successors. If `tight == true`, local slack == 0. For example,
    `GO` must not end before `COLLECT` can begin, because this would produce
    empty time between planning phases.
* `static::Bool = false` - if true, the robot must remain in place for this
    planning phase (e.g., COLLECT, DEPOSIT).
* `free::Bool = false` - if true, and if the node is a terminal node, the
    planning must go on until all non-free nodes are completed.
* `fixed::Bool = false` - if true, do not plan path because it is already fixed.
    Instead, retrieve the portion of the path directly from the pre-existing
    solution.
"""
@with_kw struct PathSpec
    node_type           ::Symbol        = :EMPTY
    start_vtx           ::Int           = -1
    final_vtx           ::Int           = -1
    min_path_duration   ::Int           =  0
    agent_id            ::BotID         = RobotID()
    object_id           ::ObjectID      = ObjectID()
    plan_path           ::Bool          = true
    tight               ::Bool          = false
    static              ::Bool          = false
    free                ::Bool          = false
    fixed               ::Bool          = false
end

# remap_object_id(spec::PathSpec,max_obj_id)  = PathSpec(spec,object_id=spec.object_id + max_obj_id)

export
    OperatingSchedule,
    get_graph,
    get_object_ICs,
    get_object_FCs,
    get_robot_ICs,
    get_actions,
    get_operations,
    get_vtx_ids,
    get_node_from_id,
    get_node_from_vtx,
    get_completion_time,
    get_duration,
    get_vtx,
    get_vtx_id,
    replace_in_schedule!,
    add_to_schedule!,
    # construct_project_schedule,
    process_schedule

"""
    OperatingSchedule

Encodes discrete events/activities that need to take place, and the precedence
constraints between them. Each `ScheduleNode` has a corresponding vertex index
and an `AbstractID`. An edge from node1 to node2 indicates a precedence
constraint between them.
"""
@with_kw struct OperatingSchedule{G<:AbstractGraph} <: AbstractGraph{Int}
    graph               ::G                     = DiGraph()
    planning_nodes      ::Dict{AbstractID,AbstractPlanningPredicate}    = Dict{AbstractID,AbstractPlanningPredicate}()
    vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
    # TODO add UID vector so that vertex deletion can be constant time
    vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}() # maps vertex uid to actual graph node
    path_specs          ::Vector{PathSpec}      = Vector{PathSpec}()
    terminal_vtxs       ::Vector{Int}           = Vector{Int}() # list of "project heads"
    weights             ::Dict{Int,Float64}     = Dict{Int,Float64}() # weights corresponding to project heads
end
Base.zero(schedule::OperatingSchedule{G}) where {G} = OperatingSchedule(graph=G())
LightGraphs.edges(schedule::OperatingSchedule) = edges(schedule.graph)
LightGraphs.is_directed(schedule::OperatingSchedule) = true
for op in [
    :edgetype,:has_edge,:has_vertex,:inneighbors,:ne,:nv,:outneighbors,
    :vertices,#:indegree,:outdegree
    ]
    @eval LightGraphs.$op(sched::OperatingSchedule,args...) = $op(sched.graph,args...)
end

CRCBS.get_graph(schedule::P) where {P<:OperatingSchedule}       = schedule.graph
get_vtx_ids(schedule::P) where {P<:OperatingSchedule}     = schedule.vtx_ids
get_terminal_vtxs(schedule::P) where {P<:OperatingSchedule}  = schedule.terminal_vtxs
get_root_node_weights(schedule::P) where {P<:OperatingSchedule}  = schedule.weights

get_node_from_id(schedule::P,id::A) where {P<:OperatingSchedule,A<:AbstractID}= schedule.planning_nodes[id]
CRCBS.get_vtx(schedule::P,id::A) where {P<:OperatingSchedule,A<:AbstractID} = get(schedule.vtx_map, id, -1)
get_vtx_id(schedule::P,v::Int) where {P<:OperatingSchedule}                   = schedule.vtx_ids[v]
get_node_from_vtx(schedule::P,v::Int) where {P<:OperatingSchedule}= schedule.planning_nodes[schedule.vtx_ids[v]]

get_nodes_of_type(schedule::P,T) where {P<:OperatingSchedule} = Dict(id=>get_node_from_id(schedule, id) for id in schedule.vtx_ids if typeof(id)<:T)
get_object_ICs(schedule::P) where {P<:OperatingSchedule}  = get_nodes_of_type(schedule,ObjectID)
get_robot_ICs(schedule::P) where {P<:OperatingSchedule}   = get_nodes_of_type(schedule,BotID)
get_actions(schedule::P) where {P<:OperatingSchedule}     = get_nodes_of_type(schedule,ActionID)
get_operations(schedule::P) where {P<:OperatingSchedule}  = get_nodes_of_type(schedule,OperationID)

export
    set_vtx_map!,
    insert_to_vtx_map!

function set_vtx_map!(schedule::S,pred::P,id::A,v::Int) where {S<:OperatingSchedule,P<:AbstractPlanningPredicate,A<:AbstractID}
    schedule.planning_nodes[id] = pred
    schedule.vtx_map[id] = v
end
function insert_to_vtx_map!(schedule::P,pred,id::ID,idx::Int) where {P<:OperatingSchedule,ID<:AbstractID}
    push!(schedule.vtx_ids, id)
    set_vtx_map!(schedule,pred,id,idx)
end

export
    get_path_spec,
    set_path_spec!,
    add_path_spec!

get_path_spec(schedule::P,v::Int) where {P<:OperatingSchedule} = schedule.path_specs[v]
function set_path_spec!(schedule::P,v::Int,spec::S) where {P<:OperatingSchedule,S<:PathSpec}
    schedule.path_specs[v] = spec
end
function add_path_spec!(schedule::P,spec::S) where {P<:OperatingSchedule,S<:PathSpec}
    push!(schedule.path_specs, spec)
    set_path_spec!(schedule, nv(schedule.graph), spec)
end

export
    ScheduleNode,
    get_schedule_node

struct ScheduleNode{I,V}
    v::Int
    node_id::I
    node::V
    path_spec::PathSpec
end
function get_schedule_node(schedule,node_id::AbstractID)
    v = get_vtx(schedule,node_id)
    node = get_node_from_id(schedule,node_id)
    path_spec = get_path_spec(schedule,v)
    return ScheduleNode(v,node_id,node,path_spec)
end
get_schedule_node(schedule,v::Int) = get_schedule_node(schedule,get_vtx_id(schedule,v))

"""
    generate_path_spec(schedule,spec,node)

Generates a `PathSpec` struct that encodes information about the path to be
planned for `node`.

Arguments:
* schedule::P OperatingSchedule
* spec::ProblemSpec
* node::T <: AbstractPlanningPredicate
"""
function generate_path_spec(schedule::P,spec::T,a::BOT_GO) where {P<:OperatingSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_robot_id(a)
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get_distance(spec.D,s0,s), # ProblemSpec distance matrix
        agent_id=r,
        tight=true,
        free = (s==-1) # if destination is -1, there is no goal location
        )
end
function generate_path_spec(schedule::P,spec::T,a::BOT_CARRY) where {P<:OperatingSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_robot_id(a)
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get_distance(spec.D,s0,s), # ProblemSpec distance matrix
        agent_id=r,
        object_id = o
        )
end
function generate_path_spec(schedule::P,spec::T,a::BOT_COLLECT) where {P<:OperatingSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_robot_id(a)
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get(spec.Δt_collect,o,0),
        agent_id=r,
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,a::BOT_DEPOSIT) where {P<:OperatingSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_robot_id(a)
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get(spec.Δt_deliver,o,0),
        agent_id=r,
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,pred::OBJECT_AT) where {P<:OperatingSchedule,T<:ProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        plan_path = false,
        object_id = get_id(get_object_id(pred))
        )
end
function generate_path_spec(schedule::P,spec::T,pred::BOT_AT) where {P<:OperatingSchedule,T<:ProblemSpec}
    r = get_robot_id(pred)
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        agent_id=r,
        free=true
        )
end
function generate_path_spec(schedule::P,spec::T,op::Operation) where {P<:OperatingSchedule,T<:ProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(op)),
        start_vtx = -1,
        final_vtx = -1,
        plan_path = false,
        min_path_duration=duration(op)
        )
end
function generate_path_spec(schedule::P,spec::T,pred::TEAM_ACTION) where {P<:OperatingSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(pred.instructions[1]))
    s = get_id(get_destination_location_id(pred.instructions[1]))
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        # min_path_duration = maximum(map(a->generate_path_spec(schedule,spec,a).min_path_duration, pred.instructions)),
        min_path_duration = get_distance(spec.D,s0,s,team_configuration(pred)),
        plan_path = true,
        static = (team_action_type(pred) <: Union{COLLECT,DEPOSIT})
        )
end
function generate_path_spec(schedule::P,pred) where {P<:OperatingSchedule}
    generate_path_spec(schedule,ProblemSpec(),pred)
end

"""
    replace_in_schedule!(schedule::OperatingSchedule,path_spec::T,pred,id::ID) where {T<:PathSpec,ID<:AbstractID}

Replace the `ScheduleNode` associated with `id` with the new node `pred`, and
the accompanying `PathSpec` `path_spec`.
"""
function replace_in_schedule!(schedule::P,path_spec::T,pred,id::ID) where {P<:OperatingSchedule,T<:PathSpec,ID<:AbstractID}
    v = get_vtx(schedule, id)
    @assert v != -1 "node id $(string(id)) is not in schedule and therefore cannot be replaced"
    set_vtx_map!(schedule,pred,id,v)
    set_path_spec!(schedule,v,path_spec)
    schedule
end
function replace_in_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:OperatingSchedule,T<:ProblemSpec,ID<:AbstractID}
    replace_in_schedule!(schedule,generate_path_spec(schedule,spec,pred),pred,id)
end
function replace_in_schedule!(schedule::P,pred,id::ID) where {P<:OperatingSchedule,ID<:AbstractID}
    replace_in_schedule!(schedule,ProblemSpec(),pred,id)
end

"""
    add_to_schedule!(schedule::OperatingSchedule,path_spec::T,pred,id::ID) where {T<:PathSpec,ID<:AbstractID}

Add `ScheduleNode` pred to `sched` with associated `AbstractID` `id` and
`PathSpec` `path_spec`.
"""
function add_to_schedule!(sched::P,path_spec::T,pred,id::ID) where {P<:OperatingSchedule,T<:PathSpec,ID<:AbstractID}
    @assert get_vtx(sched, id) == -1 "Trying to add $(string(id)) => $(string(pred)) to schedule, but $(string(id)) => $(string(get_node_from_id(sched,id))) already exists"
    add_vertex!(get_graph(sched))
    insert_to_vtx_map!(sched,pred,id,nv(get_graph(sched)))
    add_path_spec!(sched,path_spec)
    sched
end
function add_to_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:OperatingSchedule,T<:ProblemSpec,ID<:AbstractID}
    add_to_schedule!(schedule,generate_path_spec(schedule,spec,pred),pred,id)
end
function add_to_schedule!(schedule::P,pred,id::ID) where {P<:OperatingSchedule,ID<:AbstractID}
    add_to_schedule!(schedule,ProblemSpec(),pred,id)
end

function LightGraphs.has_edge(s::OperatingSchedule,i::AbstractID,j::AbstractID)
    has_edge(get_graph(s), get_vtx(s,i), get_vtx(s,j))
end
function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:OperatingSchedule,A<:AbstractID,B<:AbstractID}
    success = add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    schedule
end
function LightGraphs.rem_edge!(schedule::P,id1::A,id2::B) where {P<:OperatingSchedule,A<:AbstractID,B<:AbstractID}
    success = rem_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    schedule
end

export
    get_leaf_operation_vtxs,
    set_leaf_operation_vtxs!,
    delete_node!,
    delete_nodes!

function get_leaf_operation_vtxs(schedule::OperatingSchedule)
    terminal_vtxs = Int[]
    for v in get_all_terminal_nodes(schedule)
        if typeof(get_node_from_id(schedule, get_vtx_id(schedule,v))) == Operation
            push!(terminal_vtxs,v)
        end
    end
    return terminal_vtxs
end
function set_leaf_operation_vtxs!(schedule::OperatingSchedule)
    empty!(get_terminal_vtxs(schedule))
    empty!(get_root_node_weights(schedule))
    for vtx in get_leaf_operation_vtxs(schedule)
        push!(get_terminal_vtxs(schedule),vtx)
        get_root_node_weights(schedule)[vtx] = 1.0
    end
    schedule
end

"""
    delete_node!

removes a node (by id) from schedule.
"""
function delete_node!(schedule::OperatingSchedule, id::AbstractID)
    v = get_vtx(schedule, id)
    delete!(schedule.vtx_map, id)
    delete!(schedule.planning_nodes, id)
    rem_vertex!(get_graph(schedule), v)
    deleteat!(schedule.vtx_ids, v)
    deleteat!(schedule.path_specs, v)
    for vtx in v:nv(get_graph(schedule))
        node_id = schedule.vtx_ids[vtx]
        schedule.vtx_map[node_id] = vtx
    end
    schedule
end
delete_node!(schedule::OperatingSchedule, v::Int) = delete_node!(schedule,get_vtx_id(schedule,v))
function delete_nodes!(schedule::OperatingSchedule, vtxs::Vector{Int})
    node_ids = map(v->get_vtx_id(schedule,v), vtxs)
    for id in node_ids
        delete_node!(schedule,id)
    end
    schedule
end

export backtrack_node

"""
    `backtrack_node(sched::OperatingSchedule,v::Int)`

Find the closest ancestor of `v` with overlapping `RobotID`s.
"""
function backtrack_node(sched::OperatingSchedule,v::Int)
    robot_ids = get_robot_ids(sched,get_vtx_id(sched,v),v)
    vtxs = Int[]
    if isempty(robot_ids)
        return vtxs
    end
    for vp in inneighbors(sched,v)
        if !isempty(intersect(get_robot_ids(sched,vp),robot_ids))
            push!(vtxs,vp)
        end
    end
    return vtxs
end

export
    validate,
    sanity_check

# function dump_schedule(sched,path="tmp.schedule")
#
# end

function validate(path::Path, msg::String)
    @assert( !any(convert_to_vertex_lists(path) .== -1), msg )
    return true
end
function validate(path::Path, v::Int)
    validate(path, string("invalid path with -1 vtxs: v = ",v,", path = ",convert_to_vertex_lists(path)))
end
function validate(path::Path, v::Int, cbs_env)
    validate(path, string("v = ",v,", path = ",convert_to_vertex_lists(path),", goal: ",cbs_env.goal))
end
function sanity_check(project_schedule::OperatingSchedule,append_string="")
    G = get_graph(project_schedule)
    try
        @assert !is_cyclic(G) "is_cyclic(G)"
        for (id,node) in project_schedule.planning_nodes
            if typeof(node) <: COLLECT
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
        end
        for v in vertices(G)
            node = get_node_from_vtx(project_schedule, v)
            if isa(node,Operation)
                input_ids = Set(map(o->get_id(get_object_id(o)),collect(node.pre)))
                for v2 in inneighbors(G,v)
                    node2 = get_node_from_vtx(project_schedule, v2)
                    @assert(get_id(get_object_id(node2)) in input_ids, string(string(node2), " should not be an inneighbor of ",string(node), " whose inputs should be ",input_ids))
                end
                @assert(
                    indegree(G,v) == length(node.pre),
                    string("Operation ",string(node),
                        " needs edges from objects ",collect(input_ids),
                        " but only has edges from objects ",map(v2->get_id(get_object_id(get_node_from_vtx(project_schedule, v2))),inneighbors(G,v))
                    )
                    )
            end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            showerror(stdout,e,bt)
            print(string(e.msg, append_string))
        else
            rethrow(e)
        end
        return false
    end
    return true
end
function validate(spec::ProjectSpec,op)
    # for op in spec.operations
    for pred in preconditions(op)
        @assert get_final_condition(spec,get_object_id(pred)) == pred
    end
    for pred in postconditions(op)
        @assert get_initial_condition(spec,get_object_id(pred)) == pred
    end
    # end
end
function validate(project_schedule::OperatingSchedule)
    G = get_graph(project_schedule)
    try
        @assert !is_cyclic(G) "is_cyclic(G)"
        for (id,node) in project_schedule.planning_nodes
            if typeof(node) <: COLLECT
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
        end
        for e in edges(G)
            node1 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.src))
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.dst))
            @assert(validate_edge(node1,node2), string(" INVALID EDGE: ", string(node1), " --> ",string(node2)))
        end
        for v in vertices(G)
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            @assert( outdegree(G,v) >= sum([0, values(required_successors(node))...]) , string("node = ", string(node), " outdegree = ",outdegree(G,v), " "))
            @assert( indegree(G,v) >= sum([0, values(required_predecessors(node))...]), string("node = ", string(node), " indegree = ",indegree(G,v), " ") )
            if isa(node, Union{BOT_GO,BOT_COLLECT,BOT_CARRY,BOT_DEPOSIT})
                for v2 in outneighbors(G,v)
                    node2 = get_node_from_vtx(project_schedule, v2)
                    if isa(node2, Union{BOT_GO,BOT_COLLECT,BOT_CARRY,BOT_DEPOSIT})
                        if length(intersect(resources_reserved(node),resources_reserved(node2))) == 0 # job shop constraint
                            @assert( get_robot_id(node) == get_robot_id(node2), string("robot IDs do not match: ",string(node), " --> ", string(node2)))
                        end
                    end
                end
            end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            showerror(stdout,e,bt)
            print(e.msg)
        else
            rethrow(e)
        end
        return false
    end
    return true
end
function validate(project_schedule::OperatingSchedule,paths::Vector{Vector{Int}},t0::Vector{Int},tF::Vector{Int})
    # G = get_graph(project_schedule)
    for v in vertices(project_schedule)
        node = get_node_from_vtx(project_schedule, v)
        path_spec = get_path_spec(project_schedule, v)
        agent_id = get_id(path_spec.agent_id)
        if agent_id != -1
            path = paths[agent_id]
            start_vtx = path_spec.start_vtx
            final_vtx = path_spec.final_vtx
            try
                @assert(length(path) > t0[v], string("length(path) == $(length(path)), should be greater than t0[v] == $(t0[v]) in node ",string(node)))
                @assert(length(path) > t0[v], string("length(path) == $(length(path)), should be greater than tF[v] == $(tF[v]) in node ",string(node)))
                if start_vtx != -1
                    if length(path) > t0[v]
                        @assert(path[t0[v] + 1] == start_vtx, string("node: ",string(node), ", start_vtx: ",start_vtx, ", t0+1: ",t0[v]+1,", path[t0[v] + 1] = ",path[t0[v] + 1],", path: ", path))
                    end
                end
                if final_vtx != -1
                    if length(path) > tF[v]
                        @assert(path[tF[v] + 1] == final_vtx, string("node: ",string(node), ", final vtx: ",final_vtx, ", tF+1: ",tF[v]+1,", path[tF[v] + 1] = ",path[tF[v] + 1],", path: ", path))
                    end
                end
            catch e
                if typeof(e) <: AssertionError
                    print(e.msg)
                else
                    throw(e)
                end
                return false
            end
        end
    end
    return true
end

export
    add_single_robot_delivery_task!

# function add_single_robot_delivery_task!(sched::OperatingSchedule,spec::ProblemSpec,id::AbstractID,r::Int,o::Int,x1::Int,x2::Int)
#     add_single_robot_delivery_task!(sched,spec,id,RobotID(r),ObjectID(o),LocationID(x1),LocationID(x2))
# end
# function add_single_robot_delivery_task!(
#         schedule::S,
#         problem_spec::T,
#         pred_id::AbstractID,
#         robot_id::RobotID,
#         object_id::ObjectID,
#         pickup_station_id::LocationID,
#         dropoff_station_id::LocationID
#         ) where {S<:OperatingSchedule,T<:ProblemSpec}
#
#     if robot_id != -1
#         robot_pred = get_node_from_id(schedule,robot_id)
#         robot_start_station_id = get_initial_location_id(get_node_from_id(schedule, pred_id))
#     else
#         robot_start_station_id = LocationID(-1)
#     end
#
#     # THIS NODE IS DETERMINED BY THE TASK ASSIGNMENT.
#     action_id = get_unique_action_id()
#     add_to_schedule!(schedule, problem_spec, GO(robot_id, robot_start_station_id, pickup_station_id), action_id)
#     add_edge!(schedule, pred_id, action_id)
#
#     prev_action_id = action_id
#     action_id = get_unique_action_id()
#     add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id)
#     add_edge!(schedule, prev_action_id, action_id)
#     add_edge!(schedule, ObjectID(object_id), action_id)
#
#     prev_action_id = action_id
#     action_id = get_unique_action_id()
#     add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id)
#     add_edge!(schedule, prev_action_id, action_id)
#
#     prev_action_id = action_id
#     action_id = get_unique_action_id()
#     add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
#     add_edge!(schedule, prev_action_id, action_id)
#
#     action_id
# end
function add_headless_delivery_task!(
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        object_id::ObjectID,
        op_id::OperationID,
        robot_type::Type{R}=DeliveryBot,
        pickup_station_id::LocationID=get_location_id(get_node_from_id(sched,object_id)),
        dropoff_station_id::LocationID=get_dropoff(get_node_from_id(sched,op_id),object_id),
        # robot_id::BotID{R}=RobotID(-1)
    ) where {R<:AbstractRobotType}

    robot_id = BotID{R}(-1)
    action_id = get_unique_action_id()
    add_to_schedule!(sched, problem_spec, BOT_COLLECT(robot_id, object_id, pickup_station_id), action_id)
    add_edge!(sched, object_id, action_id)

    prev_action_id = action_id
    action_id = get_unique_action_id()
    add_to_schedule!(sched, problem_spec, BOT_CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id)
    add_edge!(sched, prev_action_id, action_id)

    prev_action_id = action_id
    action_id = get_unique_action_id()
    add_to_schedule!(sched, problem_spec, BOT_DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
    add_edge!(sched, action_id, op_id)
    add_edge!(sched, prev_action_id, action_id)

    prev_action_id = action_id
    action_id = get_unique_action_id()
    add_to_schedule!(sched, problem_spec, BOT_GO(robot_id, dropoff_station_id,LocationID(-1)), action_id)
    add_edge!(sched, prev_action_id, action_id)

    return
end
function add_headless_team_delivery_task!(
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        object_id::ObjectID,
        op_id::OperationID,
        robot_type::Type{R}=DeliveryBot,
        pickup_station_ids::Vector{LocationID}=get_location_ids(get_node_from_id(sched,object_id)),
        dropoff_station_ids::Vector{LocationID}=get_dropoffs(get_node_from_id(sched,op_id),object_id)
        ) where {R<:AbstractRobotType}

    robot_id = BotID{R}(-1)
    @assert length(pickup_station_ids) == length(dropoff_station_ids)
    n = length(pickup_station_ids)

    object_node = get_node_from_id(sched,object_id)
    shape = object_node.shape
    # COLLABORATIVE COLLECT
    team_action_id = get_unique_action_id()
    team_action = TEAM_COLLECT(
        instructions = map(x->BOT_COLLECT(robot_id, object_id, x), pickup_station_ids),
        shape=shape
        )
    add_to_schedule!(sched, problem_spec, team_action, team_action_id)
    # Add Edge from object
    add_edge!(sched,object_id,team_action_id)
    # Add GO inputs to TEAM_COLLECT
    for x in pickup_station_ids
        action_id = get_unique_action_id()
        add_to_schedule!(sched,problem_spec,BOT_GO(robot_id,x,x),action_id)
        add_edge!(sched,action_id,team_action_id)
    end
    # Add TEAM_CARRY
    prev_team_action_id = team_action_id
    team_action_id = get_unique_action_id()
    team_action = TEAM_CARRY(
        instructions = [BOT_CARRY(robot_id, object_id, x1, x2) for (x1,x2) in zip(pickup_station_ids,dropoff_station_ids)],
        shape = shape
        )
    add_to_schedule!(sched, problem_spec, team_action, team_action_id)
    add_edge!(sched,prev_team_action_id,team_action_id)
    # TEAM_DEPOSIT
    prev_team_action_id = team_action_id
    team_action_id = get_unique_action_id()
    team_action = TEAM_DEPOSIT(
        instructions = map(x->BOT_DEPOSIT(robot_id, object_id, x), dropoff_station_ids),
        shape=shape
        )
    add_to_schedule!(sched, problem_spec, team_action, team_action_id)
    add_edge!(sched,prev_team_action_id,team_action_id)
    # Edge to Operation
    add_edge!(sched,team_action_id,op_id)
    # Individual GO nodes
    for x in dropoff_station_ids
        action_id = get_unique_action_id()
        add_to_schedule!(sched,problem_spec,BOT_GO(robot_id,x,-1),action_id)
        add_edge!(sched, team_action_id, action_id)
    end
    return
end
function add_single_robot_delivery_task!(
        sched::OperatingSchedule,
        spec::ProblemSpec,
        robot_id::BotID{R},
        object_id::ObjectID,
        op_id::OperationID,
        go_id::ActionID,
        x1::LocationID=get_location_id(get_node_from_id(sched,object_id)),
        x2::LocationID=get_dropoff(get_node_from_id(sched,op_id),object_id),
        args...
        # robot_id::BotID{R}=RobotID(-1)
    ) where {R<:AbstractRobotType}
    add_headless_delivery_task!(sched,spec,object_id,op_id,R,x1,x2)

    # if !is_valid(pred_id)
    #     pred = get_node_from_id(sched,pred_id)
    #     action_id = get_unique_action_id()
    #     add_to_schedule!(sched,BOT_GO(robot_id,get_destination_location_id(pred),x1),action_id)
    #     add_edge!(sched,pred_id,action_id)
    #     pred_id = action_id
    # end
    collect_id = get_vtx_id(sched,outneighbors(sched,get_vtx(sched,object_id))[1])
    add_edge!(sched,go_id,collect_id)
    propagate_valid_ids!(sched,spec)
end

export
    construct_partial_project_schedule

function add_new_robot_to_schedule!(sched,pred::BOT_AT,problem_spec=ProblemSpec())
    robot_id = get_robot_id(pred)
    add_to_schedule!(sched, problem_spec, pred, robot_id)
    action_id = get_unique_action_id()
    action = BOT_GO(r=robot_id,x1=get_location_id(pred))
    add_to_schedule!(sched, problem_spec, action, action_id)
    add_edge!(sched, robot_id, action_id)
    return sched
end

"""
    construct_partial_project_schedule

Constructs a partial project graph
"""
function construct_partial_project_schedule(
    object_ICs::Vector{OBJECT_AT},
    object_FCs::Vector{OBJECT_AT},
    robot_ICs::Vector{R},
    operations::Vector{Operation},
    root_ops::Vector{OperationID},
    problem_spec::ProblemSpec,
    ) where {R<:BOT_AT}

    # Construct Partial Project Schedule
    sched = OperatingSchedule()
    for op in operations
        add_to_schedule!(sched, problem_spec, op, get_operation_id(op))
    end
    for pred in object_ICs
        add_to_schedule!(sched, problem_spec, pred, get_object_id(pred))
    end
    for pred in robot_ICs
        add_new_robot_to_schedule!(sched,pred,problem_spec)
    end
    # add root nodes
    for op_id in root_ops
        v = get_vtx(sched, op_id)
        push!(sched.terminal_vtxs, v)
        sched.weights[v] = 1.0
    end
    # Fill in gaps in project schedule (except for GO assignments)
    for op in operations
        op_id = get_operation_id(op)
        for object_id in get_input_ids(op)
            # add action sequence
            object_ic           = get_node_from_id(sched, object_id)
            # pickup_station_ids  = get_location_ids(object_ic)
            # object_fc           = object_FCs[get_id(object_id)]
            # dropoff_station_ids = get_location_ids(object_fc)
            # @assert dropoff_station_ids == get_dropoffs(op,object_id)
            if length(get_location_ids(object_ic)) > 1 # COLLABORATIVE TRANSPORT
                add_headless_team_delivery_task!(sched,problem_spec,
                    ObjectID(object_id),op_id) #,pickup_station_ids,dropoff_station_ids)
            else # SINGLE AGENT TRANSPORT
                add_headless_delivery_task!(sched,problem_spec,
                    ObjectID(object_id),op_id)#,pickup_station_ids[1],dropoff_station_ids[1])
            end
        end
        for object_id in get_output_ids(op)
            add_edge!(sched, op_id, object_id)
        end
    end
    # NOTE: A hack to get speed up on SparseAdjacencyMILP. Not sure if it will work
    for (id, pred) in get_object_ICs(sched)
        v = get_vtx(sched, ObjectID(id))
        if indegree(get_graph(sched),v) == 0
            op_id = get_unique_operation_id()
            op = Operation(post=Set{OBJECT_AT}([pred]),id=op_id)
            add_to_schedule!(sched,op,op_id)
            add_edge!(sched,op_id,ObjectID(id))
        end
    end
    set_leaf_operation_vtxs!(sched)
    sched
end
function construct_partial_project_schedule(spec::ProjectSpec,problem_spec::ProblemSpec,robot_ICs=Vector{BOT_AT}())
    # @assert length(robot_ICs) == problem_spec.N "length(robot_ICs) == $(length(robot_ICs)), should be $(problem_spec.N)"
    construct_partial_project_schedule(
        spec.initial_conditions,
        spec.final_conditions,
        robot_ICs,
        spec.operations,
        map(op->op.id, spec.operations[collect(spec.terminal_vtxs)]),
        problem_spec,
    )
end

function construct_partial_project_schedule(robot_ICs::Vector{R},prob_spec=
        ProblemSpec(N=length(robot_ICs))
        ) where {R<:BOT_AT}
    construct_partial_project_schedule(
        Vector{OBJECT_AT}(),
        Vector{OBJECT_AT}(),
        robot_ICs,
        Vector{Operation}(),
        Vector{OperationID}(),
        prob_spec
    )
end

"""
    process_schedule(schedule::P) where {P<:OperatingSchedule}

Compute the optimistic start and end times, along with the slack associated
with each vertex in the `schedule`. Slack for each vertex is represented as
a vector in order to handle multi-headed projects.

Args:
* schedule::OperatingSchedule
* [OPTIONAL] t0::Vector{Int}: default = zeros(Int,nv(schedule))
* [OPTIONAL] tF::Vector{Int}: default = zeros(Int,nv(schedule))
"""
function process_schedule(schedule::P,t0=zeros(Int,nv(schedule)),
        tF=zeros(Int,nv(schedule))
    ) where {P<:OperatingSchedule}

    G = get_graph(schedule)
    traversal = topological_sort_by_dfs(G)
    n_roots = max(length(schedule.terminal_vtxs),1)
    slack = map(i->Inf*ones(n_roots), vertices(G))
    local_slack = map(i->Inf*ones(n_roots), vertices(G))
    # max_deadlines = map(i->typemax(Int), vertices(G))
    # True terminal nodes
    for (i,v) in enumerate(schedule.terminal_vtxs)
        slack[v] = Inf*ones(n_roots)
        slack[v][i] = 0 # only slack for corresponding head is set to 0
    end
    ########## Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = schedule.path_specs[v]
        Δt_min = path_spec.min_path_duration
        for v2 in inneighbors(G,v)
            t0[v] = max(t0[v], tF[v2])
        end
        tF[v] = max(tF[v], t0[v] + Δt_min)
    end
    ########### Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in outneighbors(G,v)
            local_slack[v] = min.(local_slack[v], (t0[v2] - tF[v]) )
            slack[v] = min.(slack[v], slack[v2] .+ (t0[v2] - tF[v]) )
        end
    end
    t0,tF,slack,local_slack
end

export
    get_collect_node,
    get_deposit_node

function get_collect_node(schedule::P,id::ObjectID) where {P<:OperatingSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != COLLECT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end
function get_deposit_node(schedule::P,id::ObjectID) where {P<:OperatingSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != DEPOSIT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end

# ################################################################################
# ############################## New Functionality ###############################
# ################################################################################
#
# export
#     TaskGraphsMILP,
#     AssignmentMILP,
#     AdjacencyMILP,
#     SparseAdjacencyMILP
#
# """
#     TaskGraphsMILP
#
# Concrete subtypes of `TaskGraphsMILP` define different ways to formulat the
# sequential assignment portion of a PC-TAPF problem.
# """
# abstract type TaskGraphsMILP end
# """
#     AssignmentMILP <: TaskGraphsMILP
#
# Used to formulate a MILP where the decision variable is a matrix `X`, where
# `X[i,j] = 1` means that robot `i` is assigned to delivery task `j`. The
# dimensionality of `X` is (N+M) × M, where N is the number of robots and M is the
# number of delivery tasks. the last M rows of `X` correspond to "dummy robots",
# i.e. the N+jth row corresponds to "the robot that already completed task j". The
# use of these dummy robot variables allows the sequential assignment problem to
# be posed as a one-off assignment problem with inter-task constraints.
# """
# @with_kw struct AssignmentMILP <: TaskGraphsMILP
#     model::JuMP.Model = Model()
# end
# """
#     TeamAssignmentMILP
#
# ***Not yet implemented.***
#
# Eextend the assignment matrix
# formulation of `AssignmentMILP` to the "team-forming" case where robots must
# collaboratively transport some objects.
# """
# @with_kw struct TeamAssignmentMILP <: TaskGraphsMILP
#     model::JuMP.Model = Model()
#     task_group::Vector{Vector{Int}}
# end
# @with_kw struct AdjacencyMILP <: TaskGraphsMILP
#     model::JuMP.Model = Model()
#     job_shop::Bool=false
# end
# """
#     SparseAdjacencyMILP
#
# Formulates a MILP where the decision variable is a sparse adjacency matrix `X`
#     for the operating schedule graph. If `X[i,j] = 1`, there is an edge from
#     node `i` to node `j`.
# Experiments have shown that the sparse matrix approach leads to much faster
# solve times than the dense matrix approach.
# """
# @with_kw struct SparseAdjacencyMILP <: TaskGraphsMILP
#     model::JuMP.Model = Model()
#     Xa::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # assignment adjacency matrix
#     Xj::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # job shop adjacency matrix
#     job_shop::Bool=false
# end
# JuMP.optimize!(model::M) where {M<:TaskGraphsMILP}          = optimize!(model.model)
# JuMP.termination_status(model::M) where {M<:TaskGraphsMILP} = termination_status(model.model)
# JuMP.objective_function(model::M) where {M<:TaskGraphsMILP} = objective_function(model.model)
# JuMP.objective_bound(model::M) where {M<:TaskGraphsMILP}    = objective_bound(model.model)
# JuMP.primal_status(model::M) where {M<:TaskGraphsMILP}      = primal_status(model.model)
# JuMP.dual_status(model::M) where {M<:TaskGraphsMILP}        = dual_status(model.model)
# # for op = (:optimize!, :termination_status, :objective_function)
# #     eval(quote
# #         JuMP.$op(model::M,args...) where {M<:TaskGraphsMILP} = $op(model.model,args...)
# #     end)
# # end
#
# export
#     exclude_solutions!,
#     exclude_current_solution!
#
# """
#     exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})
#
# Adds constraints to model such that the solution may not match any solution
# contained in forbidden_solutions. Assumes that the model contains a variable
# container called X whose entries are binary and whose dimensions are identical
# to the dimensions of each solution in forbidden_solutions.
# """
# function exclude_solutions!(model::JuMP.Model,X::Matrix{Int})
#     @assert !any((X .< 0) .| (X .> 1))
#     @constraint(model, sum(model[:X] .* X) <= sum(model[:X])-1)
# end
# exclude_solutions!(model::TaskGraphsMILP,args...) = exclude_solutions!(model.model, args...)
# function exclude_solutions!(model::JuMP.Model,M::Int,forbidden_solutions::Vector{Matrix{Int}})
#     for X in forbidden_solutions
#         exclude_solutions!(model,X)
#     end
# end
# function exclude_solutions!(model::JuMP.Model)
#     if termination_status(model) != MOI.OPTIMIZE_NOT_CALLED
#         X = get_assignment_matrix(model)
#         exclude_solutions!(model,X)
#     end
# end
# exclude_current_solution!(args...) = exclude_solutions!(args...)
#
#
# export
#     get_assignment_matrix,
#     get_assignment_dict
#
# function get_assignment_matrix(model::M) where {M<:JuMP.Model}
#     Matrix{Int}(min.(1, round.(value.(model[:X])))) # guarantees binary matrix
# end
# get_assignment_matrix(model::TaskGraphsMILP) = get_assignment_matrix(model.model)
#

export get_assignment_dict

"""
    get_assignment_dict(assignment_matrix,N,M)

Returns dictionary that maps each robot id to a sequence of tasks.
"""
function get_assignment_dict(assignment_matrix,N,M)
    assignment_dict = Dict{Int,Vector{Int}}()
    for i in 1:N
        vec = get!(assignment_dict,i,Int[])
        k = i
        j = 1
        while j <= M
            if assignment_matrix[k,j] == 1
                push!(vec,j)
                k = j + N
                j = 0
            end
            j += 1
        end
    end
    assignment_dict
end

# end # module TaskGraphCore
