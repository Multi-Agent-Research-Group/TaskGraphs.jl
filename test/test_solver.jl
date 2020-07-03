
let
    SolverLogger{Int}()
    SolverLogger{Float64}()
    logger = SolverLogger{NTuple{3,Float64}}()
    iterations(logger)
    iterations(logger)
    iteration_limit(logger)
    start_time(logger)
    runtime_limit(logger)
    deadline(logger)
    CRCBS.lower_bound(logger)
    best_cost(logger)
    verbosity(logger)
    debug(logger)
    increment_iteration_count!(logger)
    set_lower_bound!(logger,(0.0,0.0,0.0))
    set_best_cost!(logger,(0.0,0.0,0.0))
    reset_solver!(logger)
end
let
    @test (1,0,0,0,0) < 2
    @test (1,0,0,0,0) <= 1
    @test (1,0,0,0,0) >= 1
    @test (1,0,0,0,0) > 0

    @test 2 > (1,0,0,0,0)
    @test 1 >= (1,0,0,0,0)
    @test 1 <= (1,0,0,0,0)
    @test 0 < (1,0,0,0,0)
end
# test solver initialization
let
    AStarSC()
    AStarSC{Int}()
    ISPS()
    CBS_Solver()
    TaskGraphsMILPSolver()
    NBSSolver()
    solver = NBSSolver()
    TaskGraphs.Solvers.best_cost(solver)
    NBSSolver(TaskGraphsMILPSolver(),CBS_Solver())
end
let
    solver = NBSSolver()
    increment_iteration_count!(solver)
    increment_iteration_count!(route_planner(solver))
    @test max_iterations(solver) == 1
    @test iterations(solver) == 1
    reset_solver!(solver)
    @test max_iterations(solver) == 1
    @test iterations(solver) == 0
    hard_reset_solver!(solver)
    @test max_iterations(solver) == 0
    @test max_iterations(route_planner(solver)) == 0
    @test iterations(solver) == 0
end
# in-depth testing of solver stages on a single problem
let
    # init search env
    f = initialize_toy_problem_4
    cost_model = SumOfMakeSpans()
    project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false)
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )
    prob = formulate_assignment_problem(solver.assignment_model,base_search_env;
        optimizer=Gurobi.Optimizer,
    )
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,base_search_env)
    @test validate(sched)
    # Test AStarSC
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = AStarSC()
        node = initialize_root_node(search_env)
        # plan for Robot Start
        node_id = RobotID(1)
        schedule_node = get_node_from_id(search_env.schedule, node_id)
        println(string(schedule_node))
        v = get_vtx(search_env.schedule, node_id)
        @test plan_path!(path_planner, search_env, node, schedule_node, v)
        # plan for GO
        v2 = outneighbors(search_env.schedule,v)[1]
        schedule_node = get_node_from_vtx(search_env.schedule, v2)
        println(string(schedule_node))
        @test plan_path!(path_planner, search_env, node, schedule_node, v2)
        # @show convert_to_vertex_lists(node.solution)
    end
    # Test ISPS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = ISPS()
        # node = initialize_root_node(search_env)
        valid_flag = low_level_search!(path_planner,search_env)#,node)
        # @show convert_to_vertex_lists(node.solution)
        # @show convert_to_vertex_lists(search_env.route_plan)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = ISPS()
        solution = solve!(path_planner,PC_MAPF(search_env))
        # @show convert_to_vertex_lists(solution.route_plan)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        low_level_search!(path_planner,PC_MAPF(search_env),node)
        # @show convert_to_vertex_lists(node.solution)
    end
    # Test CBS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = CBS_Solver(ISPS())
        env = solve!(path_planner,PC_MAPF(search_env))
        # @show convert_to_vertex_lists(env.route_plan)
    end
    let
        path_planner = CBS_Solver(ISPS())
        env, cost = plan_route!(path_planner,deepcopy(sched),base_search_env)
        # @show convert_to_vertex_lists(env.route_plan)
    end
    # test NBS
    let
        solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
        # solver = NBSSolver(assignment_model = TaskGraphsMILPSolver())
        TaskGraphs.Solvers.set_iteration_limit!(solver,3)
        env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
        # @show convert_to_vertex_lists(env.route_plan)
        # @show TaskGraphs.Solvers.get_logger(solver)
        # @show TaskGraphs.Solvers.optimality_gap(solver) > 0
        @test validate(env.schedule)
        @test validate(env.schedule,convert_to_vertex_lists(env.route_plan), env.cache.t0, env.cache.tF)
    end
end
# test task assignment
let
    # init search env
    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_4,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false);
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    # @show i, f, solver
                    project_schedule = construct_partial_project_schedule(
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        )
                    search_env = construct_search_env(
                        solver,
                        project_schedule,
                        problem_spec,
                        env_graph
                        )
                    prob = formulate_assignment_problem(solver.assignment_model,search_env;
                        optimizer=Gurobi.Optimizer,
                    )
                    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,search_env)
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost)
                    end
                    @test validate(sched)
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# test full solver
let
    # init search env
    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_4,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false);
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    # @show i, f, solver
                    TaskGraphs.Solvers.set_iteration_limit!(solver,1)
                    project_schedule = construct_partial_project_schedule(
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        )
                    base_search_env = construct_search_env(
                        solver,
                        project_schedule,
                        problem_spec,
                        env_graph
                        )
                    env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
                    # @show convert_to_vertex_lists(env.route_plan)
                    # prob = formulate_assignment_problem(solver.assignment_model,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    # )
                    # sched, cost = solve_assignment_problem!(solver.assignment_model,prob,search_env)
                    # # break down the solution process here:
                    # solution, cost = solve!(solver,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    #     )
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(env.schedule)
                    @test validate(
                        env.schedule,
                        convert_to_vertex_lists(env.route_plan),
                        env.cache.t0,
                        env.cache.tF
                        )
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# Test that optimal collision-free planners obtain the correct costs
let
    cost_model = MakeSpan()
    for (i, (f,expected_cost)) in enumerate([
        (initialize_toy_problem_1,5),
        (initialize_toy_problem_2,8),
        (initialize_toy_problem_3,10),
        (initialize_toy_problem_4,3),
        (initialize_toy_problem_5,4),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false);
                project_schedule = construct_partial_project_schedule(
                    project_spec,
                    problem_spec,
                    robot_ICs,
                    )
                base_search_env = construct_search_env(
                    solver,
                    project_schedule,
                    problem_spec,
                    env_graph
                    )
                env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
                @test expected_cost == cost
            end
        end
    end
    cost_model = SumOfMakeSpans()
    for (i, (f,expected_cost)) in enumerate([
        (initialize_toy_problem_1,5),
        (initialize_toy_problem_2,8),
        (initialize_toy_problem_3,10),
        (initialize_toy_problem_4,3),
        (initialize_toy_problem_5,4),
        (initialize_toy_problem_8,16),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false);
                project_schedule = construct_partial_project_schedule(
                    project_spec,
                    problem_spec,
                    robot_ICs,
                    )
                base_search_env = construct_search_env(
                    solver,
                    project_schedule,
                    problem_spec,
                    env_graph
                    )
                env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
                @test expected_cost == cost
            end
        end
    end
end
# verify that ISPS+A*sc avoids a collision by exploiting slack
let
    solver = NBSSolver()
    cost_model = MakeSpan()
    project_spec, problem_spec, robot_ICs, _, env_graph = initialize_toy_problem_3(;cost_function=cost_model,verbose=false);
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )

    prob = formulate_assignment_problem(assignment_solver(solver),base_search_env)
    project_schedule, cost = solve_assignment_problem!(assignment_solver(solver),prob,base_search_env)
    @test termination_status(prob) == MOI.OPTIMAL

    env = construct_search_env(solver,project_schedule,base_search_env)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    low_level_search!(route_planner(solver).low_level_planner,env,node);

    path1 = convert_to_vertex_lists(node.solution.paths[1])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @test path2[2] == 5 # test that robot 2 will indeed wait for robot 1
    @test path1 == [2, 6, 10, 14, 18, 22, 26, 30, 31, 32, 32]
    @test path2 == [5, 5, 6,  7,  8,  12, 12, 12, 12, 12, 16]
    # @show path1
    # @show path2
end
# Non-zero process time
let
    """
    Verify that the algorithm correctly handles scenarios where process time
    is non-zero
    """
    println("Test non-zero wait time:")
    for Δt in [0,1,4]
        let
            solver = NBSSolver()
            cost_model=MakeSpan()
            project_spec, problem_spec, robot_ICs, _, env_graph = initialize_toy_problem_6(;
                cost_function=cost_model,
                verbose=false,
                Δt_op=Δt
                );
            project_schedule = construct_partial_project_schedule(
                project_spec,
                problem_spec,
                robot_ICs,
                )
            base_search_env = construct_search_env(
                solver,
                project_schedule,
                problem_spec,
                env_graph;
                primary_objective=cost_model,
                )
            env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
            paths = convert_to_vertex_lists(env.route_plan)
            # @show paths[1]
            # @show paths[2]
            @test paths[1][8+Δt] == 13
            @test paths[1][9+Δt] == 17
        end
    end
end
# Non-zero collection time
let
    """
    Verify that the algorithm correctly handles scenarios where collection time
    and deposit time are non-zero.
    """
    println("Test non-zero collection time:")
    Δt_deliver_1 = 1
    for (Δt_collect_2, true_cost) in zip([0,1,2,3,4],[6,7,8,8,8])
        solver = NBSSolver()
        cost_model = MakeSpan()
        project_spec, problem_spec, robot_ICs, _, env_graph = initialize_toy_problem_7(;
            cost_function=cost_model,
            verbose=false,
            Δt_op=0,
            Δt_collect=[0,Δt_collect_2,0],
            Δt_deliver=[Δt_deliver_1,0,0]
            );
        project_schedule = construct_partial_project_schedule(
            project_spec,
            problem_spec,
            robot_ICs,
            )
        base_search_env = construct_search_env(
            solver,
            project_schedule,
            problem_spec,
            env_graph;
            primary_objective=cost_model,
            )
        env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
        paths = convert_to_vertex_lists(env.route_plan)
        @test cost == true_cost
    end
end
# Collaborative transport problems
let
    cost_model = MakeSpan()
    for (i, (f,expected_cost,expected_paths)) in enumerate([
        (initialize_toy_problem_11, 6, [
            [2, 2, 6, 10, 14, 13, 9],
            [4, 3, 7, 11, 15, 15, 15],
            [11, 7, 8, 4, 3, 3, 3]
        ]),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false);
                project_schedule = construct_partial_project_schedule(
                    project_spec,
                    problem_spec,
                    robot_ICs,
                    )
                base_search_env = construct_search_env(
                    solver,
                    project_schedule,
                    problem_spec,
                    env_graph
                    )
                env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
                paths = convert_to_vertex_lists(env.route_plan)
                @test cost == expected_cost
                for (expected_path, path) in zip(expected_paths,paths)
                    @test path == expected_path
                end
            end
        end
    end
end
