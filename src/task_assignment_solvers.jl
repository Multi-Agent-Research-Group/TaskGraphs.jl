export
    TaskGraphsMILP,
    AssignmentMILP,
    AdjacencyMILP,
    SparseAdjacencyMILP,
    FastSparseAdjacencyMILP

"""
    TaskGraphsMILP

Concrete subtypes of `TaskGraphsMILP` define different ways to formulate the
sequential assignment portion of a PC-TAPF problem.
"""
abstract type TaskGraphsMILP end

"""
    AssignmentMILP <: TaskGraphsMILP

Used to formulate a MILP where the decision variable is a matrix `X`, where
`X[i,j] = 1` means that robot `i` is assigned to delivery task `j`. The
dimensionality of `X` is (N+M) × M, where N is the number of robots and M is the
number of delivery tasks. the last M rows of `X` correspond to "dummy robots",
i.e. the N+jth row corresponds to "the robot that already completed task j". The
use of these dummy robot variables allows the sequential assignment problem to
be posed as a one-off assignment problem with inter-task constraints.
"""
@with_kw struct AssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model                   = Model()
    sched::OperatingSchedule            = OperatingSchedule()
    robot_ics  ::Vector{Pair{RobotID,ScheduleNode}} = sort(collect(get_nodes_of_type(sched,BotID));by=p->p.first)
    object_ics ::Vector{Pair{ObjectID,ScheduleNode}} = sort(collect(get_nodes_of_type(sched,ObjectID));by=p->p.first)
    operations ::Vector{Pair{OperationID,ScheduleNode}} = sort(collect(get_nodes_of_type(sched,OperationID));by=p->p.first)
    robot_map    ::Dict{BotID,Int}      = Dict{BotID,Int}(p.first=>k for (k,p) in enumerate(robot_ics))
    object_map   ::Dict{ObjectID,Int}   = Dict{ObjectID,Int}(p.first=>k for (k,p) in enumerate(object_ics))
    operation_map::Dict{OperationID,Int}= Dict{OperationID,Int}(p.first=>k for (k,p) in enumerate(operations))
    # robot_ids::Vector{BotID}            = Vector{BotID}() # ids of all robots
    # robot_map::Dict{BotID,Int}          = Dict{BotID,Int}(id=>k for (k,id) in enumerate(robot_ids)) # robot id to idx
    # object_ids::Vector{ObjectID}        = Vector{ObjectID}()
    # object_map::Dict{ObjectID,Int}      = Dict{ObjectID,Int}(id=>k for (k,id) in enumerate(object_ids))
    # r0::Vector{Int}                     = Vector{Int}()
    # s0::Vector{Int}                     = Vector{Int}()
    # tr0_::Dict{Int,Int}                 = Dict{Int,Int}()
    # to0_::Dict{Int,Int}                 = Dict{Int,Int}()
    # Δt_collect::Vector{Int}             = zeros(Int,length(object_ids))
    # Δt_deposit::Vector{Int}             = zeros(Int,length(object_ids))
    # sF::Vector{Int}                     = -1.0*zeros(Int,length(s0))
    # Δt::Vector{Int}                     = zeros(Int,length(s0)) # the (parent operation duration) for each object
    # terminal_vtxs::Vector{Set{Int}}     = Vector{Set{Int}}()
    # weights::Vector{Int}                = map(s->1.0,terminal_vtxs) 
end
AssignmentMILP(model::JuMP.Model) = AssignmentMILP(model=model)

"""
    adj_mat_from_assignment_mat(sched,assignment_matrix)

Compute an adjacency matrix from an assignment matrix
"""
function adj_mat_from_assignment_mat(sched::OperatingSchedule,assignment_matrix)
    N = length(get_robot_ICs(sched))
    M = length(get_object_ICs(sched))
    assignment_dict = get_assignment_dict(assignment_matrix,N,M)
    G = get_graph(sched)
    adj_matrix = adjacency_matrix(G)
    for (robot_id, task_list) in assignment_dict
        robot_node = get_node_from_id(sched, RobotID(robot_id))
        v_go = outneighbors(G, get_vtx(sched, RobotID(robot_id)))[1] # GO_NODE
        for object_id in task_list
            v_collect = outneighbors(G,get_vtx(sched, ObjectID(object_id)))[1]
            adj_matrix[v_go,v_collect] = 1
            v_carry = outneighbors(G,v_collect)[1]
            v_deposit = outneighbors(G,v_carry)[1]
            for v in outneighbors(G,v_deposit)
                if isa(get_vtx_id(sched, v), ActionID)
                    v_go = v
                    break
                end
            end
        end
    end
    adj_matrix
end

"""
    TeamAssignmentMILP

***Not yet implemented.***

Eextend the assignment matrix
formulation of `AssignmentMILP` to the "team-forming" case where robots must
collaboratively transport some objects.
"""
@with_kw struct TeamAssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    task_group::Vector{Vector{Int}}
end
@with_kw struct AdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    job_shop::Bool=false
end
"""
    SparseAdjacencyMILP <: TaskGraphsMILP

Formulates a MILP where the decision variable is a sparse adjacency matrix `X`
    for the operating schedule graph. If `X[i,j] = 1`, there is an edge from
    node `i` to node `j`.
Experiments have shown that the sparse matrix approach leads to much faster
solve times than the dense matrix approach.
"""
@with_kw struct SparseAdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    Xa::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # assignment adjacency matrix
    Xj::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # job shop adjacency matrix
    job_shop::Bool=false
end
@with_kw struct FastSparseAdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    Xa::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # assignment adjacency matrix
    Xj::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # job shop adjacency matrix
    job_shop::Bool=false
end
# JuMP.optimize!(model::M) where {M<:TaskGraphsMILP}          = optimize!(model.model)
# JuMP.termination_status(model::M) where {M<:TaskGraphsMILP} = termination_status(model.model)
# JuMP.objective_function(model::M) where {M<:TaskGraphsMILP} = objective_function(model.model)
# JuMP.objective_bound(model::M) where {M<:TaskGraphsMILP}    = objective_bound(model.model)
# JuMP.primal_status(model::M) where {M<:TaskGraphsMILP}      = primal_status(model.model)
# JuMP.dual_status(model::M) where {M<:TaskGraphsMILP}        = dual_status(model.model)
for op in [
    :(JuMP.optimize!),
    :(JuMP.termination_status),
    :(JuMP.objective_function),
    :(JuMP.objective_bound),
    :(JuMP.primal_status),
    :(JuMP.dual_status),
    :(JuMP.set_silent),
    ]
    @eval $op(milp::TaskGraphsMILP) = $op(milp.model)
end
for op in [
    :(JuMP.set_optimizer_attribute),
    :(JuMP.set_optimizer_attributes),
    :(JuMP.set_time_limit_sec),
    ]
    @eval $op(milp::TaskGraphsMILP,args...) = $op(milp.model,args...)
end
# function JuMP.set_optimizer_attribute(milp::TaskGraphsMILP,k,v)
#     set_optimizer_attribute(milp.model,k,v)
# end
# JuMP.set_optimizer_attributes(milp::TaskGraphsMILP, pairs...) = set_optimizer_attributes(milp.model,pairs...) 
# JuMP.set_silent(model::TaskGraphsMILP) = set_optimizer_attribute(model,MOI.Silent(),true)
# JuMP.set_time_limit_sec(model::TaskGraphsMILP,val) = set_optimizer_attribute(model,MOI.TimeLimitSec(),val)
# for op in (:optimize!, :termination_status, :objective_function)
#     @eval JuMP.$op(model::M,args...) where {M<:TaskGraphsMILP} = $op(model.model,args...)
# end

export
    exclude_solutions!,
    exclude_current_solution!

"""
    exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})

Adds constraints to model such that the solution may not match any solution
contained in forbidden_solutions. Assumes that the model contains a variable
container called X whose entries are binary and whose dimensions are identical
to the dimensions of each solution in forbidden_solutions.
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


export
    get_assignment_matrix

function get_assignment_matrix(model::M) where {M<:JuMP.Model}
    Matrix{Int}(min.(1, round.(value.(model[:X])))) # guarantees binary matrix
end
get_assignment_matrix(model::TaskGraphsMILP) = get_assignment_matrix(model.model)

export
    add_job_shop_constraints!,
    formulate_optimization_problem,
    formulate_schedule_milp,
    formulate_milp,
    update_project_schedule!

"""
    get_sF(milp_model::AssignmentMILP)

Return an a vector of final object locations.
"""
function get_sF(milp_model::AssignmentMILP)
    @unpack sched, object_ics, object_map = milp_model
    M = length(object_ics)
    sF = zeros(Int,M)
    for (o_id,node) in object_ics
        _, deposit_node = get_deposit_node(sched,o_id)
        sF[object_map[o_id]] = get_id(get_destination_location_id(deposit_node))
    end
    return sF
end

"""
    add_job_shop_constraints!(milp_model::AssignmentMILP,sched::OperatingSchedule,spec::ProblemSpec) #,model::JuMP.Model)

After an `AssignmentMILP` has been optimized, add in any edges that result from
an active ``job shop'' constraint (where two robots require the same resource).
"""
function add_job_shop_constraints!(milp_model::AssignmentMILP,sched::OperatingSchedule,spec::ProblemSpec) #,model::JuMP.Model)
    @unpack model,robot_ics,object_ics = milp_model

    N = length(robot_ics) # number of robots
    M = length(object_ics) # number of delivery tasks
    r0 = map(p->get_id(get_initial_location_id(p.second.node)),robot_ics) # initial robot locations
    s0 = map(p->get_id(get_initial_location_id(p.second.node)),object_ics) # initial object locations
    sF = get_sF(milp_model)
    tor = Int.(round.(value.(model[:tor]))) # collect begin time
    toc = Int.(round.(value.(model[:toc]))) # collect end time
    tod = Int.(round.(value.(model[:tod]))) # deposit begin time
    tof = Int.(round.(value.(model[:tof]))) # deposit end time
    for (j,(o_id1,_)) in enumerate(object_ics)
        for (j2,(o_id2,_)) in Base.Iterators.drop(enumerate(object_ics),j)
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                if s0[j] == s0[j2]
                    id1, n1 = get_collect_node(sched, ObjectID(o_id1))
                    id2, n2 = get_collect_node(sched, ObjectID(o_id2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    id1, n1 = get_collect_node(sched, ObjectID(o_id1))
                    id2, n2 = get_deposit_node(sched, ObjectID(o_id2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    id1, n1 = get_deposit_node(sched, ObjectID(o_id1))
                    id2, n2 = get_collect_node(sched, ObjectID(o_id2))
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    id1, n1 = get_deposit_node(sched, ObjectID(o_id1))
                    id2, n2 = get_deposit_node(sched, ObjectID(o_id2))
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                if t1[2] < t2[1]
                    add_edge!(sched, id1, id2)
                elseif t2[2] < t1[1]
                    add_edge!(sched, id2, id1)
                else
                    throw(ErrorException("JOB SHOP CONSTRAINT VIOLATED"))
                end
            end
        end
    end
end


"""
    get_objective_expr

Helper for setting the objective function for a milp model
"""
function get_objective_expr(milp::AssignmentMILP, f::SumOfMakeSpans)
    @unpack model, sched, operations, object_map = milp
    terminal_ops = map(p->p.second.node, filter(p->is_terminal_node(sched,p.first),operations))
    tof = model[:tof]
    @variable(model, T[1:length(terminal_ops)])
    for (i,op) in enumerate(terminal_ops)
        for (o_id,o) in preconditions(op)
            @constraint(model, T[i] >= tof[object_map[o_id]] + duration(op))
        end
    end
    weights = map(i->1.0,1:length(terminal_ops))
    cost = @expression(model, sum(map(i->T[i]*weights[i],1:length(terminal_ops))))
    return cost
end
function get_objective_expr(milp::AssignmentMILP, f::MakeSpan)
    @unpack model, sched, operations, object_map = milp
    tof = model[:tof]
    terminal_ops = map(p->p.second.node, filter(p->is_terminal_node(sched,p.first),operations))
    @variable(model, cost)
    for (i,op) in enumerate(terminal_ops)
        for (o_id,o) in preconditions(op)
            @constraint(model, cost >= tof[object_map[o_id]] + duration(op))
        end
    end
    return cost
end

"""
    formulate_milp(milp_model::AssignmentMILP,sched,problem_spec;kwargs...)

Express the TaskGraphs assignment problem as an `AssignmentMILP` using the JuMP
optimization framework.

Inputs:
    milp_model::T <: TaskGraphsMILP : a milp model that determines how the
        sequential task assignment problem is modeled. Current options are
        `AssignmentMILP`, `SparseAdjacencyMILP` and `GreedyAssignment`.
    sched::OperatingSchedule : a partial operating schedule, where
        some or all assignment edges may be missing.
    problem_spec::ProblemSpec : encodes the distance matrix and other
        information about the problem.

Keyword Args:
    `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)
    `cost_model=MakeSpan` - optimization objective, currently either `MakeSpan`
        or `SumOfMakeSpans`. Defaults to the cost_model associated with
        `problem_spec`
Outputs:
    `model::AssignmentMILP` - an instantiated optimization problem
"""
function formulate_milp(milp_model::AssignmentMILP,
    sched::OperatingSchedule,
    problem_spec::ProblemSpec;
    optimizer=default_milp_optimizer(),
    cost_model=problem_spec.cost_function,
    Mm = 10000,
    kwargs...)

    # SETUP
    # Define optimization model
    model=Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model,default_optimizer_attributes()...)
    milp = AssignmentMILP(model=model,sched=sched)

    @unpack robot_ics,object_ics,operations,robot_map,object_map,operation_map = milp

    N = length(robot_ics) # number of robots
    M = length(object_ics) # number of delivery tasks
    Δt_collect = zeros(Int,M)
    Δt_deposit = zeros(Int,M)
    r0 = map(p->get_id(get_initial_location_id(p.second.node)),robot_ics) # initial robot locations
    s0 = map(p->get_id(get_initial_location_id(p.second.node)),object_ics) # initial object locations
    sF = zeros(Int,M)
    for (v,n) in enumerate(get_nodes(sched))
        if matches_node_type(n,BOT_COLLECT)
            Δt_collect[object_map[get_object_id(n)]] = get_min_duration(n)
        elseif matches_node_type(n,BOT_DEPOSIT)
            Δt_deposit[object_map[get_object_id(n)]] = get_min_duration(n)
            sF[object_map[get_object_id(n)]] = get_id(get_destination_location_id(n))
        end
    end
    # from ProblemSpec
    D = (x,y) -> get_distance(problem_spec,x,y)
    
    @variable(model, to0[1:M] >= 0.0) # object availability time
    @variable(model, tor[1:M] >= 0.0) # object robot arrival time
    @variable(model, toc[1:M] >= 0.0) # object collection complete time
    @variable(model, tod[1:M] >= 0.0) # object deliver begin time
    @variable(model, tof[1:M] >= 0.0) # object termination time
    @variable(model, tr0[1:N+M] >= 0.0) # robot availability time

    # Assignment matrix x
    @variable(model, X[1:N+M,1:M], binary = true) # X[i,j] ∈ {0,1}
    @constraint(model, X * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, X' * ones(N+M) .== 1)     # each task must have exactly 1 assignment
    for (id,node) in robot_ics # robot start times
        @constraint(model, tr0[robot_map[id]] == get_t0(node))
    end
    for (id,node) in object_ics # task start times
        if is_root_node(sched,id) # only applies to root tasks (with no prereqs)
            @constraint(model, to0[object_map[id]] == get_t0(node))
        end
    end
    precedence_graph = CustomNDiGraph{Nothing,ObjectID}()
    for (id,_) in object_ics
        add_node!(precedence_graph,nothing,id)
    end
    for (op_id,node) in operations #get_operations(sched) # precedence constraints on task start time
        op = node.node
        for (_,input) in preconditions(op)
            i = object_map[get_object_id(input)]
            for (_,output) in postconditions(op)
                j = object_map[get_object_id(output)]
                @constraint(model, to0[j] >= tof[i] + duration(op))
                add_edge!(precedence_graph,get_object_id(input),get_object_id(output))
            end
        end
    end
    # propagate upstream edges through precedence graph
    for v in topological_sort_by_dfs(precedence_graph)
        for v1 in inneighbors(precedence_graph,v)
            for v2 in outneighbors(precedence_graph,v)
                add_edge!(precedence_graph,v1,v2)
            end
        end
        add_edge!(precedence_graph,v,v)
    end
    # constraints
    r0 = vcat(r0,sF) # combine to get dummy robot ``spawn'' locations too
    for j in 1:M
        # constraint on dummy robot start time (corresponds to moment of object delivery)
        @constraint(model, tr0[j+N] == tof[j])
        # dummy robots can't do upstream jobs
        for v in inneighbors(precedence_graph,j)
            @constraint(model, X[j+N,v] == 0)
        end
        # lower bound on task completion time (task can't start until it's available).
        @constraint(model, tor[j] >= to0[j])
        for i in 1:N+M
            @constraint(model, tor[j] - (tr0[i] + D(r0[i],s0[j])) >= -Mm*(1 - X[i,j]))
        end
        @constraint(model, toc[j] == tor[j] + Δt_collect[j])
        @constraint(model, tod[j] == toc[j] + D(s0[j],sF[j]))
        @constraint(model, tof[j] == tod[j] + Δt_deposit[j])
        # "Job-shop" constraints specifying that no station may be double-booked. A station
        # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
        # the windows for these operations cannot overlap. In the constraints below, t1 and t2
        # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
        # respectively. If eny of the operations for these two tasks require use of the same
        # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
        # j must occur before the operation for task j2. The opposite is true for y == [0,1].
        # We use the big M method here as well to tightly enforce the binary constraints.
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary=true)
                @constraint(model, tmax >= t1[1])
                @constraint(model, tmax >= t2[1])
                @constraint(model, tmin <= t1[2])
                @constraint(model, tmin <= t2[2])

                @constraint(model, tmax - t2[1] <= (1 - y)*Mm)
                @constraint(model, tmax - t1[1] <= y*Mm)
                @constraint(model, tmin - t1[2] >= (1 - y)*-Mm)
                @constraint(model, tmin - t2[2] >= y*-Mm)
                # @constraint(model, tmin + 1 <= tmax)
                @constraint(model, tmin + 1 - X[j+N,j2] - X[j2+N,j] <= tmax) # NOTE +1 not necessary if the same robot is doing both
            end
        end
    end
    cost = get_objective_expr(milp, cost_model)
    @objective(model, Min, cost)
    milp
end

export
    preprocess_project_schedule

"""
    preprocess_project_schedule(sched)

Returns information about the eligible and required successors and predecessors
of nodes in `sched`

Arguments:
- `sched::OperatingSchedule`

Outputs:
- missing_successors
- missing_predecessors
- n_eligible_successors
- n_eligible_predecessors
- n_required_successors
- n_required_predecessors
- upstream_vertices
- non_upstream_vertices

TODO: OBJECT_AT nodes should always have the properties that
`indegree(G,v) == n_required_predecessors(v) == n_eligible_predecessors(v)`
`outdegree(G,v) == n_required_successors(v) == n_eligible_successors(v)`
Not sure if this is currently the case. UPDATE: I believe this has already been
    addressed by making each object come from an initial operation.
"""
function preprocess_project_schedule(sched)
    G = get_graph(sched);
    # Identify required and eligible edges
    missing_successors      = Dict{Int,Dict}()
    missing_predecessors    = Dict{Int,Dict}()
    n_eligible_successors   = zeros(Int,nv(G))
    n_eligible_predecessors = zeros(Int,nv(G))
    n_required_successors   = zeros(Int,nv(G))
    n_required_predecessors = zeros(Int,nv(G))
    for v in vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        for (key,val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key,val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key,val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key,val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key,typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G,v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key,typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))

    return missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors, n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices
end
function preprocess_project_schedule(sched,flag)
    missing_successors,
    missing_predecessors,
    n_eligible_successors,
    n_eligible_predecessors,
    n_required_successors,
    n_required_predecessors,
    upstream_vertices,
    non_upstream_vertices = preprocess_project_schedule(sched)
    cache = (
        missing_successors=missing_successors,
        missing_predecessors=missing_predecessors,
        n_eligible_successors=n_eligible_successors,
        n_eligible_predecessors=n_eligible_predecessors,
        n_required_successors=n_required_successors,
        n_required_predecessors=n_required_predecessors,
        upstream_vertices=upstream_vertices,
        non_upstream_vertices=non_upstream_vertices,
    )
    return cache
end


function formulate_schedule_milp(sched::OperatingSchedule,problem_spec::ProblemSpec;
        optimizer=default_milp_optimizer(),
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans(),
    )
    G = get_graph(sched);
    assignments = [];
    Δt = get_min_duration(sched)
    
    model=Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model,default_optimizer_attributes()...)
    
    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    @variable(model, Xa[1:nv(G),1:nv(G)], binary = true); # Precedence Adjacency Matrix TODO make sparse
    @constraint(model, Xa .+ Xa' .<= 1) # no bidirectional or self edges
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(sched, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(sched, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v])
        for v2 in outneighbors(G,v)
            @constraint(model, Xa[v,v2] == 1)
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT
        end
    end
    # Identify required and eligible edges
    missing_successors      = Dict{Int,Dict}()
    missing_predecessors    = Dict{Int,Dict}()
    n_eligible_successors   = zeros(Int,nv(G))
    n_eligible_predecessors = zeros(Int,nv(G))
    n_required_successors   = zeros(Int,nv(G))
    n_required_predecessors = zeros(Int,nv(G))
    for v in vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        for (key,val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key,val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key,val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key,val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key,typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G,v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key,typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    for v in vertices(G)
        for v2 in upstream_vertices[v]
            @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
        end
    end
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))
    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
            node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
            potential_match = false
            for (template, val) in missing_successors[v]
                if !matches_template(template, typeof(node2)) # possible to add an edge
                    continue
                end
                for (template2, val2) in missing_predecessors[v2]
                    if !matches_template(template2, typeof(node)) # possible to add an edge
                        continue
                    end
                    potential_match = true # TODO: investigate why the solver fails when this is moved inside the following if statement
                    if (val > 0 && val2 > 0)
                        new_node = align_with_successor(node,node2)
                        dt_min = generate_path_spec(sched,problem_spec,new_node).min_duration
                        @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                        break
                    end
                end
            end
            if potential_match == false
                @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
            end
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    @variable(model, Xj[1:nv(G),1:nv(G)], binary = true); # job shop adjacency matrix
    @constraint(model, Xj .+ Xj' .<= 1) # no bidirectional or self edges
    for v in 1:nv(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        for v2 in v+1:nv(G)
            node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
            common_resources = intersect(resources_reserved(node),resources_reserved(node2))
            if length(common_resources) > 0
                # @show common_resources
                # tmax = @variable(model)
                # tmin = @variable(model)
                # y = @variable(model, binary=true)
                # job_shop_variables[(v,v2)] = y
                # @constraint(model, tmax >= t0[v])
                # @constraint(model, tmax >= t0[v2])
                # @constraint(model, tmin <= tF[v])
                # @constraint(model, tmin <= tF[v2])
                #
                # @constraint(model, tmax - t0[v2] <= (1 - y)*Mm)
                # @constraint(model, tmax - t0[v] <= y*Mm)
                # @constraint(model, tmin - tF[v] >= (1 - y)*-Mm)
                # @constraint(model, tmin - tF[v2] >= y*-Mm)
                # @constraint(model, tmin + 1 <= tmax)

                # Big M constraints
                @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                if !(v2 in upstream_vertices[v])
                    @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                end
                if !(v in upstream_vertices[v2])
                    @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                end
            else
                @constraint(model, Xj[v,v2] == 0)
                @constraint(model, Xj[v2,v] == 0)
            end
        end
    end

    # Full adjacency matrix
    @variable(model, X[1:nv(G),1:nv(G)]); # Adjacency Matrix
    @constraint(model, X .== Xa .+ Xj)

    sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    milp = AdjacencyMILP(model=model) #, job_shop_variables
    cost1 = get_objective_expr(milp,cost_model,milp.model,sched,tF)
    @objective(milp.model, Min, cost1 + sparsity_cost)
    milp
end
function formulate_milp(milp_model::AdjacencyMILP,sched::OperatingSchedule,problem_spec::ProblemSpec;
    optimizer=default_milp_optimizer(),
    kwargs...)
    formulate_schedule_milp(sched,problem_spec;optimizer=optimizer,kwargs...)
end
function formulate_milp(milp_model::SparseAdjacencyMILP,sched::OperatingSchedule,problem_spec::ProblemSpec;
        optimizer=default_milp_optimizer(),
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans(),
        job_shop=milp_model.job_shop,
        kwargs...
    )

    model=Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model,default_optimizer_attributes()...)

    G = get_graph(sched);
    (missing_successors, missing_predecessors, n_eligible_successors,
        n_eligible_predecessors, n_required_successors, n_required_predecessors,
        upstream_vertices, non_upstream_vertices
        ) = preprocess_project_schedule(sched)

    Δt = get_min_duration(sched)

    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    Xa = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(sched, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(sched, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v]) # NOTE Δt may change for some nodes
        for v2 in outneighbors(G,v)
            Xa[v,v2] = @variable(model, binary=true) # TODO remove this (MUST UPDATE n_eligible_successors, etc. accordingly)
            @constraint(model, Xa[v,v2] == 1) #TODO this edge already exists--no reason to encode it as a decision variable
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT. Making this an equality constraint causes the solver to return a higher final value in some cases (e.g., toy problems 2,3,7). Why? Maybe the Big-M constraint forces it to bump up. I though the equality constraint might speed up the solver.
        end
    end

    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        potential_match = false
        if outdegree(G,v) < n_eligible_successors[v] # NOTE: Trying this out to save time on formulation
            for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
                if indegree(G,v2) < n_eligible_predecessors[v2]
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    for (template, val) in missing_successors[v]
                        if !matches_template(template, typeof(node2)) # possible to add an edge
                            continue
                        end
                        for (template2, val2) in missing_predecessors[v2]
                            if !matches_template(template2, typeof(node)) # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                potential_match = true
                                new_node = align_with_successor(node,node2)
                                if !robot_ids_match(node,node2)
                                    @log_info(-1,0,"Edge $(string(node)) --> $(string(node2)) should be illegal, but is being allowed by SparseAdjacencyMILP")
                                    @log_info(-1,0,"inneighbors(sched,$(string(node))):   ",  map(vp->string(string(get_node_from_vtx(sched,vp)),", "), inneighbors(sched,v))...)
                                    @log_info(-1,0,"outneighbors(sched,$(string(node))):  ", map(vp->string(string(get_node_from_vtx(sched,vp)),", "), outneighbors(sched,v))...)
                                    @log_info(-1,0,"inneighbors(sched,$(string(node2))):  ", map(vp->string(string(get_node_from_vtx(sched,vp)),", "), inneighbors(sched,v2))...)
                                    @log_info(-1,0,"outneighbors(sched,$(string(node2))): ",map(vp->string(string(get_node_from_vtx(sched,vp)),", "), outneighbors(sched,v2))...)
                                end

                                dt_min = generate_path_spec(sched,problem_spec,new_node).min_duration
                                Xa[v,v2] = @variable(model, binary=true) # initialize a new binary variable in the sparse adjacency matrix
                                @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                                @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                                break
                            end
                        end
                    end
                end
            end
        end
        if potential_match == false && job_shop == false
            @constraint(model, tF[v] == t0[v] + Δt[v]) # adding this constraint may provide some speedup
        end
    end

    # In the sparse implementation, these constraints must come after all possible edges are defined by a VariableRef
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);
    for i in 1:nv(G)
        for j in i:nv(G)
            # prevent self-edges and cycles
            @constraint(model, Xa[i,j] + Xa[j,i] <= 1)
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    Xj = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    if job_shop
        for v in 1:nv(G)
            node = get_node_from_id(sched, get_vtx_id(sched, v))
            for v2 in non_upstream_vertices[v] #v+1:nv(G)
                if v2 > v && ~(v in upstream_vertices[v2]) && ~(has_edge(G,v,v2) || has_edge(G,v2,v))
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    common_resources = intersect(resources_reserved(node),resources_reserved(node2))
                    if length(common_resources) > 0
                        println("MILP FORMULATION: adding a job shop constraint between ",v, " (",string(node),") and ", v2, " (",string(node2),")")
                        # @show common_resources
                        # Big M constraints
                        Xj[v,v2] = @variable(model, binary=true) #
                        Xj[v2,v] = @variable(model, binary=true) # need both directions to have a valid adjacency matrix
                        @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                        @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                    end
                end
            end
        end
    end

    # Full adjacency matrix
    @expression(model, X, Xa .+ Xj) # Make X an expression rather than a variable

    milp = SparseAdjacencyMILP(model,Xa,Xj, milp_model.job_shop) #, job_shop_variables
    cost1 = get_objective_expr(milp,cost_model,milp.model,sched,tF)
    @objective(milp.model, Min, cost1)
    milp
end
function formulate_milp(milp_model::FastSparseAdjacencyMILP,sched::OperatingSchedule,problem_spec::ProblemSpec;
        optimizer=default_milp_optimizer(),
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans(),
        job_shop=milp_model.job_shop,
        kwargs...
    )
    
    model=Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model,default_optimizer_attributes()...)

    G = get_graph(sched);
    cache = preprocess_project_schedule(sched,true)
    Δt = get_min_duration(sched)

    # In order to speed up the solver, we try to reduce the total number of
    # variables in the JuMP Model. Wherever possible, t0[v] and tF[v] are
    # defined as constants or expressions rather than variables.
    t0 = Vector{Union{Real,VariableRef,GenericAffExpr}}(zeros(nv(G)))
    tF = Vector{Union{Real,VariableRef,GenericAffExpr}}(zeros(nv(G)))
    for v in topological_sort(G)
        if indegree(G,v) == cache.n_eligible_predecessors[v] == 0
            n_id = get_vtx_id(sched,v)
            t0[v] = @expression(model,get(t0_,n_id,0.0))
            # @show indegree(G,v), v, string(get_node_from_vtx(sched,v)), typeof(t0[v])
        elseif indegree(G,v) == cache.n_eligible_predecessors[v] == 1
            vp = inneighbors(G,v)[1]
            t0[v] = @expression(model,tF[vp])
            # @show indegree(G,v), v, string(get_node_from_vtx(sched,v)), typeof(t0[v])
        else
            t0[v] = @variable(model)
            for vp in inneighbors(G,v)
                @constraint(model,t0[v] >= tF[vp])
            end
            # @show indegree(G,v), v, string(get_node_from_vtx(sched,v)), typeof(t0[v])
        end
        if outdegree(G,v) == cache.n_eligible_successors[v]
            tF[v] = @expression(model,t0[v] + Δt[v])
        else
            tF[v] = @variable(model)
            # NOTE Cannot use equality constraint in general case, because Δt
            # changes for "assignment" nodes. Therefore, we must use
            # "tF[v] ≧ t0[v] + Δt[v]" rather than "tF[v] == t0[v] + Δt[v]"
            @constraint(model, tF[v] >= t0[v] + Δt[v])
        end
    end
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(sched, id)
        @constraint(model, t0[v] >= t)
    end
    # for (id,t) in tF_
    #     v = get_vtx(sched, id)
    #     @constraint(model, tF[v] >= t)
    # end
    # Precedence relationships
    Xa = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        for v2 in outneighbors(G,v)
            Xa[v,v2] = @variable(model, binary=true) # TODO remove this (MUST UPDATE n_eligible_successors, etc. accordingly)
            @constraint(model, Xa[v,v2] == 1) #TODO this edge already exists--no reason to encode it as a decision variable
        end
    end

    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        potential_match = false
        if outdegree(G,v) < cache.n_eligible_successors[v] # NOTE: Trying this out to save time on formulation
            for v2 in cache.non_upstream_vertices[v] # for v2 in vertices(G)
                if indegree(G,v2) < cache.n_eligible_predecessors[v2]
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    for (template, val) in cache.missing_successors[v]
                        if !matches_template(template, typeof(node2)) # possible to add an edge
                            continue
                        end
                        for (template2, val2) in cache.missing_predecessors[v2]
                            if !matches_template(template2, typeof(node)) # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                potential_match = true
                                new_node = align_with_successor(node,node2)
                                dt_min = generate_path_spec(sched,problem_spec,new_node).min_duration
                                Xa[v,v2] = @variable(model, binary=true) # initialize a new binary variable in the sparse adjacency matrix
                                @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                                @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                                break
                            end
                        end
                    end
                end
            end
        end
        # if potential_match == false && job_shop == false
        #     @constraint(model, tF[v] == t0[v] + Δt[v]) # adding this constraint may provide some speedup
        # end
    end

    # In the sparse implementation, these constraints must come after all possible edges are defined by a VariableRef
    @constraint(model, Xa * ones(nv(G))  .<= cache.n_eligible_successors);
    @constraint(model, Xa * ones(nv(G))  .>= cache.n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= cache.n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= cache.n_required_predecessors);
    for i in 1:nv(G)
        for j in i:nv(G)
            # prevent self-edges and cycles
            @constraint(model, Xa[i,j] + Xa[j,i] <= 1)
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    Xj = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    if job_shop
        for v in 1:nv(G)
            node = get_node_from_id(sched, get_vtx_id(sched, v))
            for v2 in cache.non_upstream_vertices[v] #v+1:nv(G)
                if v2 > v && ~(v in cache.upstream_vertices[v2]) && ~(has_edge(G,v,v2) || has_edge(G,v2,v))
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    common_resources = intersect(resources_reserved(node),resources_reserved(node2))
                    if length(common_resources) > 0
                        println("MILP FORMULATION: adding a job shop constraint between ",v, " (",string(node),") and ", v2, " (",string(node2),")")
                        # @show common_resources
                        # Big M constraints
                        Xj[v,v2] = @variable(model, binary=true) #
                        Xj[v2,v] = @variable(model, binary=true) # need both directions to have a valid adjacency matrix
                        @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                        @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                    end
                end
            end
        end
    end

    # Full adjacency matrix
    @expression(model, X, Xa .+ Xj) # Make X an expression rather than a variable

    milp = FastSparseAdjacencyMILP(model,Xa,Xj, milp_model.job_shop) #, job_shop_variables
    cost1 = get_objective_expr(milp,cost_model,milp.model,sched,tF)
    @objective(milp.model, Min, cost1)
    milp
end
function get_objective_expr(milp,f::SumOfMakeSpans,model,sched,tF)
    terminal_vtxs = sched.terminal_vtxs
    @variable(model, T[1:length(terminal_vtxs)])
    for (i,project_head) in enumerate(terminal_vtxs)
        for v in project_head
            @constraint(model, T[i] >= tF[v])
        end
    end
    cost1 = @expression(model, sum(map(v->tF[v]*get(sched.weights,v,0.0), terminal_vtxs)))
end
function get_objective_expr(milp,f::MakeSpan,model,sched,tF)
    @variable(model, T)
    @constraint(model, T .>= tF) # TODO Maybe the number of constraints here causes a slowdown that could be addressed by only adding constraints on terminal nodes?
    cost1 = @expression(model, T)
end

export
    GreedyAssignment

abstract type GreedyCost end
struct GreedyPathLengthCost <: GreedyCost end
struct GreedyFinalTimeCost  <: GreedyCost end
struct GreedyLowerBoundCost <: GreedyCost end

"""
    GreedyAssignment{C,M} <: TaskGraphsMILP

GreedyAssignment maintains three sets: The "satisfied set" `C`, the "required
incoming" set `Ai`, and the "available outgoing" set `Ao`.

At each step, the algorithm identifies the nodes `v1 ∈ Ai` and `v2 ∈ Ao` with
shortest "distance" (in the context of `OperatingSchedule`s, this distance
refers to the duration of `v1` if an edge `v1 → v2` is added) and adds an edge
between them. The distance corresponding to an ineligible edge is set to Inf.

After adding the edge, the algorithm sweeps through a topological ordering of
the vertices and updates `C`, `Ai`, and `Ao`. In order for `v` to be placed in
`C`, `v` must have enough incoming edges and all of `v`'s predecessors must
already be in `C`. In order to be added to `Ai`, `v` must have less than the
required number of incoming edges and all of `v`'s predecessors must
already be in `C`. In order for `v` to be added to `Ao`, `v` must have less than
the allowable number of outgoing edges, and must be in `C`.
"""
@with_kw struct GreedyAssignment{C,M} <: TaskGraphsMILP
    schedule::OperatingSchedule = OperatingSchedule()
    problem_spec::ProblemSpec   = ProblemSpec()
    cost_model::C               = SumOfMakeSpans()
    greedy_cost::M              = GreedyPathLengthCost()
    t0::Vector{Int}             = zeros(Int,nv(schedule))
end
exclude_solutions!(model::GreedyAssignment) = nothing # exclude most recent solution in order to get next best solution
JuMP.termination_status(model::GreedyAssignment)    = MOI.OPTIMAL
JuMP.primal_status(model::GreedyAssignment)         = MOI.FEASIBLE_POINT
get_assignment_matrix(model::GreedyAssignment)      = adjacency_matrix(get_graph(model.schedule))
function JuMP.objective_function(model::GreedyAssignment{SumOfMakeSpans,M}) where {M}
    t0,tF,slack,local_slack = process_schedule(model.schedule,model.t0)
    return sum(tF[model.schedule.terminal_vtxs] .* map(v->model.schedule.weights[v], model.schedule.terminal_vtxs))
end
function JuMP.objective_function(model::GreedyAssignment{MakeSpan,M}) where {M}
    t0,tF,slack,local_slack = process_schedule(model.schedule,model.t0)
    return maximum(tF[model.schedule.terminal_vtxs])
end
function JuMP.objective_function(model::GreedyAssignment)
    println("UNKNOWN COST FUNCTION!")
    return Inf
end
JuMP.objective_bound(model::GreedyAssignment)       = objective_function(model)
JuMP.value(c::Real) = c
# JuMP.set_optimizer_attribute(milp::GreedyAssignment,k,v) = nothing
# JuMP.set_optimizer_attribute(milp::GreedyAssignment,k,v) = nothing
# JuMP.set_time_limit_sec(milp::GreedyAssignment,val) = nothing
JuMP.set_silent(milp::GreedyAssignment) = nothing
for op in [
    :(JuMP.set_optimizer_attribute),
    :(JuMP.set_optimizer_attributes),
    :(JuMP.set_time_limit_sec),
    ]
    @eval $op(milp::GreedyAssignment,args...) = nothing
end
function formulate_milp(milp_model::GreedyAssignment,sched::OperatingSchedule,problem_spec::ProblemSpec;
        t0_ = Dict{AbstractID,Float64}(),
        cost_model = SumOfMakeSpans(),
        kwargs...
    )

    model = GreedyAssignment(
        schedule = sched,
        problem_spec = problem_spec,
        cost_model = cost_model
    )
    for (id,t) in t0_
        v = get_vtx(sched, id)
        model.t0[v] = t
    end
    model
end

function construct_schedule_distance_matrix(sched,problem_spec)
    G = get_graph(sched);
    cache = preprocess_project_schedule(sched,true)
    D = Inf * ones(nv(G),nv(G))
    for v in vertices(G)
        if outdegree(G,v) < cache.n_eligible_successors[v]
            node = get_node_from_id(sched, get_vtx_id(sched, v))
            for v2 in cache.non_upstream_vertices[v]
                if indegree(G,v2) < cache.n_eligible_predecessors[v2]
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    if has_robot_id(node) && has_robot_id(node2)
                        if get_id(get_robot_id(node)) != -1 && get_id(get_robot_id(node2)) != -1
                            get_robot_id(node) == get_robot_id(node2) ? continue : nothing
                        end
                    end
                    for (template, val) in cache.missing_successors[v]
                        if !matches_template(template, typeof(node2)) # possible to add an edge
                            continue
                        end
                        for (template2, val2) in cache.missing_predecessors[v2]
                            if !matches_template(template2, typeof(node)) # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                new_node = align_with_successor(node,node2)
                                D[v,v2] = generate_path_spec(sched,problem_spec,new_node).min_duration
                            end
                        end
                    end
                end
            end
        end
    end
    D
end

function update_greedy_sets!(model,G,C,Ai,Ao,n_required_predecessors,n_eligible_successors,frontier=get_all_root_nodes(G))
    while !isempty(frontier)
        v = pop!(frontier)
        if issubset(inneighbors(G,v),C)
            if indegree(G,v) >= n_required_predecessors[v]
                push!(C,v)
                union!(frontier,outneighbors(G,v))
            else
                push!(Ai,v)
            end
        end
        if (outdegree(G,v) < n_eligible_successors[v]) && (v in C)
            push!(Ao,v)
        end
    end
end

get_edge_cost(model::GreedyAssignment{M,GreedyPathLengthCost},D,v,v2) where M = D[v,v2]
get_edge_cost(model::GreedyAssignment{M,GreedyFinalTimeCost},D,v,v2) where M = model.t0[v] + D[v,v2]

"""
Identifies the nodes `v ∈ Ai` and `v2 ∈ Ao` with the shortest distance
`D[v,v2]`.
"""
function select_next_edges(model,D,Ao,Ai)
    c = Inf
    a = -1
    b = -2
    for (v,v2) in Base.Iterators.product(sort(collect(Ao)),sort(collect(Ai)))
        if D[v,v2] < c
            c = D[v,v2]
            a = v
            b = v2
        end
    end
    if a < 0 || b < 0
        println("debugging edge selection for model ",typeof(model))
        for v in Ai
            node = get_node_from_vtx(model.schedule,v)
            println(string("node ",string(node), " needs assignment"))
            @show required_predecessors(node)
            @show indegree(model.schedule,v)
        end
    end
    return [(a,b)]
end

"""
GreedyAssignment maintains three sets: The "satisfied set" `C`, the "required
incoming" set `Ai`, and the "available outgoing" set `Ao`.

At each step, the algorithm identifies the nodes `v1 ∈ Ai` and `v2 ∈ Ao` with
shortest "distance" (in the context of `OperatingSchedule`s, this distance
refers to the duration of `v1` if an edge `v1 → v2` is added) and adds an edge
between them. The distance corresponding to an ineligible edge is set to Inf.

After adding the edge, the algorithm sweeps through a topological ordering of
the vertices and updates `C`, `Ai`, and `Ao`. In order for `v` to be placed in
`C`, `v` must have enough incoming edges and all of `v`'s predecessors must
already be in `C`. In order to be added to `Ai`, `v` must have less than the
required number of incoming edges and all of `v`'s predecessors must
already be in `C`. In order for `v` to be added to `Ao`, `v` must have less than
the allowable number of outgoing edges, and must be in `C`.
"""
function JuMP.optimize!(model::GreedyAssignment)
    sched    = model.schedule
    problem_spec        = model.problem_spec
    G                   = get_graph(sched);
    cache = preprocess_project_schedule(sched,true)
    C = Set{Int}() # Closed set (these nodes have enough predecessors)
    Ai = Set{Int}() # Nodes that don't have enough incoming edges
    Ao = Set{Int}() # Nodes that can have more outgoing edges
    update_greedy_sets!(model,G,C,Ai,Ao,
        cache.n_required_predecessors,cache.n_eligible_successors)
    D = construct_schedule_distance_matrix(sched,problem_spec)
    while length(Ai) > 0
        for (v,v2) in select_next_edges(model,D,Ao,Ai)
            setdiff!(Ao,v)
            setdiff!(Ai,v2)
            add_edge!(G,v,v2)
        end
        update_greedy_sets!(model,G,C,Ai,Ao,
            cache.n_required_predecessors,cache.n_eligible_successors)
    end
    set_leaf_operation_vtxs!(sched)
    propagate_valid_ids!(sched,problem_spec)
    model
end

export compute_lower_bound
"""
    compute_lower_bound(env,[starts,assigned,dist_mtx,pairs])

Computes a lower bound on makespan for `sched::OperatingSchedule` by assuming
    that any robot can be simultaneously assigned to multiple tasks.

Args:
- `env`      SearchEnv
- `starts`   the set of vertices whose outgoing edges are available
- `assigned` the set of vertices whose incoming edges are already assigned
- `dist_mtx` encodes the cost of each edge v -> vp as `dist_mtx[v,vp]`
- `pairs`    specifies eligible edges
"""
function compute_lower_bound(env,
    starts=Set{Int}(vertices(get_schedule(env))), # starting vtxs
    assigned=Set{Int}(), # closed set
    dist_mtx = construct_schedule_distance_matrix(get_schedule(env),get_problem_spec(env)),
    pairs=[BOT_GO=>BOT_COLLECT],
    )
    sched = deepcopy(get_schedule(env))
    for p in pairs
        for vp in topological_sort_by_dfs(get_graph(sched))
            if (vp in assigned) || !isa(get_node_from_vtx(sched,vp), p.second)
                continue
            end
            t0 = typemax(Int)
            v = -1
            for src in starts # topological_sort_by_dfs(get_graph(sched))
                if !isa(get_node_from_vtx(sched,src), p.first)
                    continue
                end
                t0_ = get_t0(sched,src)+dist_mtx[src,vp]
                if t0_ < t0
                    v = src
                    t0 = t0_
                end
            end
            add_edge!(get_graph(sched),v,vp)
            set_t0!(sched,vp,t0)
            @log_info(1,global_verbosity(),"$v=>$vp, $(string(get_node_from_vtx(sched,v)))=>$(string(get_node_from_vtx(sched,vp))), t0=$t0")
        end
    end
    process_schedule!(sched)
    return cache
end

export
    propagate_valid_ids!

function propagate_valid_ids!(sched::OperatingSchedule,problem_spec::ProblemSpec)
    @assert(is_cyclic(sched) == false, "is_cyclic(sched)") # string(sparse(adj_matrix))
    # Propagate valid IDs through the schedule
    for v in topological_sort_by_dfs(sched)
        n_id = get_vtx_id(sched, v)
        node = get_node_from_id(sched, n_id)
        for v2 in inneighbors(sched,v)
            node = align_with_predecessor(node,get_node_from_vtx(sched, v2))
        end
        for v2 in outneighbors(sched,v)
            node = align_with_successor(node,get_node_from_vtx(sched, v2))
        end
        path_spec = get_path_spec(sched, v)
        if path_spec.fixed
            replace_in_schedule!(sched, path_spec, node, n_id)
        else
            replace_in_schedule!(sched, problem_spec, node, n_id)
        end
    end
    return true
end

"""
    update_project_schedule!

Args:
- solver
- sched
- adj_matrix - adjacency_matrix encoding the edges that need to be added to
    the project schedule

Adds all required edges to the project graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(solver,sched::OperatingSchedule,problem_spec,adj_matrix)
    mtx = adjacency_matrix(sched)
    val = update_project_schedule!(sched,problem_spec,adj_matrix)
    @log_info(1,verbosity(solver),"Assignment: Adding edges \n",
        map(idx->string("\t",
                string(get_node_from_vtx(sched,idx.I[1]))," → ",
                string(get_node_from_vtx(sched,idx.I[2])),"\n"
                ),
            findall(adj_matrix .- mtx .!= 0))...
    )
    val
end
function update_project_schedule!(sched::OperatingSchedule,problem_spec,adj_matrix)
    # Add all new edges to project sched
    G = get_graph(sched)
    # remove existing edges first, so that there is no carryover between consecutive MILP iterations
    for e in collect(edges(G))
        rem_edge!(G, e)
    end
    # add all edges encoded by adjacency matrix
    for v in vertices(G)
        for v2 in vertices(G)
            if adj_matrix[v,v2] >= 1
                add_edge!(G,v,v2)
            end
        end
    end
    try
        propagate_valid_ids!(sched,problem_spec)
        @assert validate(sched)
    catch e
        if isa(e, AssertionError)
            showerror(stdout,e,catch_backtrace())
        else
            rethrow(e)
        end
        return false
    end
    process_schedule!(sched)
    return true
end

"""
    update_project_schedule!(solver,milp_model::M,sched,problem_spec,
        adj_matrix) where {M<:TaskGraphsMILP}

Args:
- milp_model <: TaskGraphsMILP
- sched::OperatingSchedule
- problem_spec::ProblemSpec
- adj_matrix : an adjacency_matrix or (in the case where
    `milp_model::AssignmentMILP`), an assignment matrix

Adds all required edges to the schedule graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(solver,model::TaskGraphsMILP,sched,prob_spec,
        adj_matrix=get_assignment_matrix(model),
    )
    update_project_schedule!(solver,sched,prob_spec,adj_matrix)
end
function update_project_schedule!(solver,model::AssignmentMILP,sched,prob_spec,
        assignment_matrix=get_assignment_matrix(model),
    )
    adj_matrix = adj_mat_from_assignment_mat(sched,assignment_matrix)
    update_project_schedule!(solver,sched,prob_spec,adj_matrix)
end
