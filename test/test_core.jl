let
    id = ActionID(1)
    id += 1
end
# ProjectSpec
let
    M = 3
    object_ICs = Vector{OBJECT_AT}([
        OBJECT_AT(1,1),
        OBJECT_AT(2,2),
        OBJECT_AT(3,3)
        ])
    object_FCs = Vector{OBJECT_AT}([
        OBJECT_AT(1,4),
        OBJECT_AT(2,5),
        OBJECT_AT(3,6)
        ])
    robot_ICs = Dict{Int,ROBOT_AT}(
        1=>ROBOT_AT(1,7),
        2=>ROBOT_AT(2,8),
        3=>ROBOT_AT(3,9)
        )
    # Testing root nodes
    let
        project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
        @test project_spec.root_nodes == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.root_nodes == Set{Int}([2])
    end
    let
        project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [], 1.0))
        @test project_spec.root_nodes == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.root_nodes == Set{Int}([1,2])
    end
    let
        project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        delivery_graph = construct_delivery_graph(project_spec,M)
    end
end
let
    problem_spec = ProblemSpec()
end
# Simple Hand Crafted Problem
let
    project_spec, problem_spec, robot_ICs, optimal_assignments, env_graph = initialize_toy_problem_1()
    N = problem_spec.N
    M = problem_spec.M

    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL

    assignment_matrix = get_assignment_matrix(model)
    # assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M)
    assignment_dict, assignments = get_assignment_dict(assignment_matrix,N,M)
    # @test assignments == optimal_assignments

    project_schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments);

    t0,tF,slack,local_slack = process_schedule(project_schedule)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 0

    # try with perturbed start times
    t0[get_vtx(project_schedule,RobotID(2))] = 1
    t0,tF,slack,local_slack = process_schedule(project_schedule;t0=t0)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 1

    @test length(get_vtx_ids(project_schedule)) == nv(get_graph(project_schedule))
    for (v,id) in enumerate(project_schedule.vtx_ids)
        @test get_vtx(project_schedule, id) == v
    end
end
# combining two project specs
let
    Random.seed!(0)
    N = 4                  # num robots
    M = 6                  # num delivery tasks
    # env_graph, vtx_grid = initialize_grid_graph_with_obstacles([10,10]);
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
    # N = 40                  # num robots
    # M = 60                  # num delivery tasks
    # env_graph, vtx_grid = initialize_grid_graph_with_obstacles([50,50]);
    pickup_zones = collect(1:M)
    dropoff_zones = collect(M+1:2*M)
    free_zones = collect(2*M+1:nv(env_graph))
    dist_matrix = get_dist_matrix(env_graph)

    r0 = free_zones[1:N]
    s0 = pickup_zones[1:M]
    sF = dropoff_zones[1:M]
    # r0,s0,sF = get_random_problem_instantiation(
    #     N,M,pickup_zones,dropoff_zones,free_zones)

    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    # Drs, Dss = cached_pickup_and_delivery_distances(pts[r0],pts[s0],pts[sF])
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    object_ICs1 = Vector{OBJECT_AT}([object_ICs[o] for o in 1:Int(M/2)])
    object_FCs1 = Vector{OBJECT_AT}([object_FCs[o] for o in 1:Int(M/2)])
    project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    object_FCs2 = Vector{OBJECT_AT}([object_FCs[o] for o in Int(M/2)+1:M])
    object_ICs2 = Vector{OBJECT_AT}([object_ICs[o] for o in Int(M/2)+1:M])
    project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    project_spec = combine_project_specs([project_spec1, project_spec2])

    delivery_graph = construct_delivery_graph(project_spec,M)

    filename = "project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec_mod = read_project_spec(filename)
    # @test project_spec_mod == project_spec
end
let
    N = 4                  # num robots
    M = 6                  # num delivery tasks
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
    # N = 40                  # num robots
    # M = 60                  # num delivery tasks
    # env_graph, vtx_grid = initialize_grid_graph_with_obstacles([50,50]);
    pickup_zones = collect(1:M)
    dropoff_zones = collect(M+1:2*M)
    free_zones = collect(2*M+1:nv(env_graph))
    dist_matrix = get_dist_matrix(env_graph)

    r0 = free_zones[1:N]
    s0 = pickup_zones[1:M]
    sF = dropoff_zones[1:M]

    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    # Drs, Dss = cached_pickup_and_delivery_distances(pts[r0],pts[s0],pts[sF])
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    object_ICs1 = Vector{OBJECT_AT}([object_ICs[o] for o in 1:Int(M/2)])
    object_FCs1 = Vector{OBJECT_AT}([object_FCs[o] for o in 1:Int(M/2)])
    project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    object_FCs2 = Vector{OBJECT_AT}([object_FCs[o] for o in Int(M/2)+1:M])
    object_ICs2 = Vector{OBJECT_AT}([object_ICs[o] for o in Int(M/2)+1:M])
    project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    project_spec = combine_project_specs([project_spec1, project_spec2])

    filename = "project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec = read_project_spec(filename)

    problem_def = SimpleProblemDef(project_spec,r0,s0,sF)
    filename = "problem_def.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(problem_def))
    end
    problem_def = read_problem_def(filename)

    # for spec in [project_spec1, project_spec2, project_spec]
    #     let
    #         project_spec = spec
    #         delivery_graph = construct_delivery_graph(project_spec,M)
    #         project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
    #         model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
    #         optimize!(model)
    #         @test termination_status(model) == MathOptInterface.OPTIMAL
    #         assignment = get_assignment_matrix(model);
    #         assignments = map(j->findfirst(assignment[:,j] .== 1),1:M)
    #         for r in N+1:N+M
    #             robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    #         end
    #         project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);
    #         o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    #         input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    #         @test_skip o_keys == input_ids
    #         rg = get_display_metagraph(project_schedule)
    #     end
    # end
end
let
    N = 4                  # num robots
    M = 6                  # num delivery tasks
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
    dist_matrix = get_dist_matrix(env_graph)
    pickup_zones = collect(1:M)
    dropoff_zones = collect(M+1:2*M)
    free_zones = collect(2*M+1:nv(env_graph))

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_random_task_graphs_problem(
        N,M,pickup_zones,dropoff_zones,free_zones,dist_matrix)

    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);

    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL

    assignment_matrix = get_assignment_matrix(model);
    # assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M)
    assignment_dict, assignments = get_assignment_dict(assignment_matrix,N,M)
    project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);

    o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    @test o_keys == Set(input_ids)
end
