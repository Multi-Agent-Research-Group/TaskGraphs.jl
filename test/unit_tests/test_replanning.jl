# Replanner Interface
let
    for model in [
            ReplannerConfig(),
            DeferUntilCompletion(),
            ReassignFreeRobots(),
            MergeAndBalance(),
            Oracle(),
            NullReplanner(),
        ]
        set_commit_threshold!(model,10)
        @test get_commit_threshold(model) == 10
        set_timeout_buffer!(model,10)
        @test get_timeout_buffer(model) == 10
        set_route_planning_buffer!(model,10)
        @test get_route_planning_buffer(model) == 10
    end
    FullReplanner()
    ProjectRequest(OperatingSchedule(),10,10)
end
let
    @test remap_object_id(ObjectID(1),2) == ObjectID(3)
    @test remap_object_id(OBJECT_AT(1,1),2) == OBJECT_AT(3,1)
    remap_object_id(ScheduleNode(
        ObjectID(1),
        OBJECT_AT(1,1),
        PathSpec()
        ),2)
    prob = pctapf_problem_1(NBSSolver())
    sched = get_schedule(prob.env)
    remap_object_ids!(sched,10)
    for (k,v) in get_object_ICs(sched)
        @test get_id(k) > 10
        @test get_id(get_object_id(v)) > 10
   end
end
# Test break_assignments(...)
let
    sched = OperatingSchedule()
    n1 = add_node!(sched,make_node(sched,GO(1,1,1)))
    n2 = add_child!(sched,n1.id,make_node(sched,GO(1,1,2)))
    n3 = add_child!(sched,n2.id,make_node(sched,COLLECT(1,1,2)))
    # Break assignments should only remove BOT_GO-->BOT_COLLECT edges
    break_assignments!(sched,ProblemSpec())
    @test has_edge(sched,n1,n2)
    @test !has_edge(sched,n2,n3)
end
let
    planner = FullReplanner(
        solver = NBSSolver(),
        replanner = MergeAndBalance()
    )
    set_real_time_flag!(planner,false) # turn off real-time op constraints
    # set_commit_threshold!(replan_model,40) # setting high commit threshold to allow for warmup
    prob = replanning_problem_1(planner.solver)
    env = prob.env
    stage = 0

    stage += 1
    request = prob.requests[stage]
    remap_object_ids!(request.schedule,env.schedule)
    base_env = replan!(planner,env,request)
    for v in vertices(request.schedule)
        node = get_node(request.schedule,v)
        @test get_t0(node) >= request.t_request
        @test get_t0(base_env,node) >= get_commit_time(planner,env,request)
    end
    # env, cost = solve!(planner.solver,PC_TAPF(base_env))
end
let

    features=[
        RunTime(),SolutionCost(),OptimalFlag(),FeasibleFlag(),OptimalityGap(),
        IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
        RobotPaths()
        ]
    replan_model = MergeAndBalance()
    set_real_time_flag!(replan_model,false) # turn off real time constraints
    solvers = [
        NBSSolver(),
        NBSSolver(assignment_model=TaskGraphsMILPSolver(ExtendedAssignmentMILP())),
        NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}()),
    ]
    for solver in solvers
        set_verbosity!(solver,0)
        set_iteration_limit!(solver,1)
        set_iteration_limit!(route_planner(solver),5000)
    end
    problem_generators = replanning_test_problems()

    for f in problem_generators
        for solver in solvers
            # @show f
            cache = ReplanningProfilerCache(features=features)
            prob = f(solver)
            env, cache = profile_replanner!(solver,replan_model,prob,cache)
            @show cache.stage_results[end]["OptimalityGap"]
            @show cache.stage_results[end]["OptimalFlag"]
            reset_solver!(solver)
        end
    end
end
let
    planner = FullReplanner(
        solver = NBSSolver(),
        replanner = ConstrainedMergeAndBalance(max_problem_size=20)
    )
    planner2 = FullReplanner(
        solver = NBSSolver(),
        replanner = MergeAndBalance()
    )
    set_commit_threshold!(planner,2)
    set_commit_threshold!(planner2,2)
    set_real_time_flag!(planner,false) # turn off real-time op constraints
    set_real_time_flag!(planner2,false) # turn off real-time op constraints
    # set_commit_threshold!(replan_model,40) # setting high commit threshold to allow for warmup
    prob = replanning_problem_1(planner.solver)
    env = prob.env
    stage = 0

    stage += 1
    request = prob.requests[stage]
    remap_object_ids!(request.schedule,env.schedule)
    base_env = replan!(planner,env,request)
    env, cost = solve!(planner.solver,PC_TAPF(base_env))

end
# Test loading saved problems
let
    loader = ReplanningProblemLoader()
    add_env!(loader,"env_2",init_env_2())

    MAX_TIME_LIMIT = 40
    MAX_BACKUP_TIME_LIMIT = 20
    COMMIT_THRESHOLD = 10
    ASTAR_ITERATION_LIMIT = 5000
    PIBT_ITERATION_LIMIT = 5000
    CBS_ITERATION_LIMIT = 1000
    MAX_ASSIGNMENT_ITERATIONS = 1

    reset_all_id_counters!()
    # # write problems
    Random.seed!(0)
    # write_problems!(loader,problem_configs,base_problem_dir)

    feats = [
        RunTime(),IterationCount(),LowLevelIterationCount(),
        TimeOutStatus(),IterationMaxOutStatus(),
        SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
        # ObjectPathSummaries(),
        ]
    final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
    planner_configs = []
    for (primary_replanner, backup_replanner) in [
            (ConstrainedMergeAndBalance(max_problem_size=300),     DeferUntilCompletion()),
        ]
        # Primary planner
        path_finder = DefaultAStarSC()
        set_iteration_limit!(path_finder,ASTAR_ITERATION_LIMIT)
        primary_route_planner = CBSSolver(ISPS(path_finder))
        set_iteration_limit!(primary_route_planner,CBS_ITERATION_LIMIT)
        primary_assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())
        primary_planner = FullReplanner(
            solver = NBSSolver(
                assignment_model=primary_assignment_model,
                path_planner=primary_route_planner,
                ),
            replanner = primary_replanner,
            cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
            )
        set_iteration_limit!(primary_planner,MAX_ASSIGNMENT_ITERATIONS)
        set_max_time_limit!(primary_planner,MAX_TIME_LIMIT)
        set_commit_threshold!(primary_planner,COMMIT_THRESHOLD)
        # Backup planner
        backup_planner = FullReplanner(
            solver = NBSSolver(
                assignment_model = TaskGraphsMILPSolver(GreedyAssignment(
                    greedy_cost = GreedyFinalTimeCost(),
                )),
                path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true)
                ),
            replanner = backup_replanner,
            cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
            )
        set_iteration_limit!(backup_planner,1)
        set_iteration_limit!(route_planner(backup_planner.solver),PIBT_ITERATION_LIMIT)
        set_commit_threshold!(backup_planner,COMMIT_THRESHOLD)
        set_debug!(backup_planner,true)
        # set_debug!(route_planner(backup_planner.solver),true)
        # Full solver
        planner = ReplannerWithBackup(primary_planner,backup_planner)
        planner_name=string(string(typeof(primary_replanner)),"-",string(typeof(backup_replanner)))
        planner_config = (
            planner=planner,
            planner_name=planner_name,
            results_path=joinpath(base_results_dir,planner_name),
            objective = SumOfMakeSpans(),
        )
        push!(planner_configs,planner_config)
    end

    # warm up to precompile replanning code
    planner_config = planner_configs[1]
    planner = planner_config.planner

    @info "Warming up $(planner_config.planner_name)"
    # warmup(planner_config.planner,loader)
    base_dir            = joinpath("/tmp","warmup")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")
    config = Dict(
        :N => 10,
        :M => 5,
        :num_projects => 3,
        :env_id => sort(collect(keys(loader.envs)))[1],
        )
    Random.seed!(0)
    write_problems!(loader,config,base_problem_dir)
    env = get_env!(loader,config[:env_id])
    set_real_time_flag!(planner,false)
    set_deadline!(planner,Inf)
    set_runtime_limit!(planner,Inf)

    # profile_replanner!(loader,planner,base_problem_dir,base_results_dir)
    plannerA = planner.primary_planner
    reset_cache!(plannerA)
    hard_reset_solver!(plannerA)

    f = joinpath(base_problem_dir,"problem0001")
    problem_name = splitdir(f)[end]
    outpath = joinpath(base_results_dir,problem_name)
    # ispath(outpath) ? continue : nothing # skip over this if results already exist
    @log_info(-1,0,"PROFILING: testing on problem ",problem_name)
    simple_prob_def = read_simple_repeated_problem_def(f)
    prob = RepeatedPC_TAPF(simple_prob_def,planner.primary_planner.solver,loader)

    # search_env, planner = profile_replanner!(planner,prob)
    reset_cache!(planner)
    plannerA = planner.primary_planner
    plannerB = planner.backup_planner
    hard_reset_solver!(planner.primary_planner.solver)
    hard_reset_solver!(planner.backup_planner.solver)
    env = get_env(prob)

    stage = 1
    request = prob.requests[stage]
    request = ProjectRequest(request.schedule,0,0)
    remap_object_ids!(request.schedule,get_schedule(env))

    base_envA = replan!(plannerA,env,request)

    pctapf = TaskGraphs.construct_pctapf_problem(prob,base_envA)

    # envA,cost = solve!(plannerA.solver,pctapf)
    solver = plannerA.solver
    pcta = formulate_assignment_problem(solver.assignment_model,pctapf)
    sched, cost = solve_assignment_problem!(solver.assignment_model,pcta,pctapf)
    termination_status(solver.assignment_model.milp.model)
    sched, cost = solve_assignment_problem!(TaskGraphsMILPSolver(ExtendedAssignmentMILP()),pcta,pctapf)
    @assert cost == maximum(get_tF(sched)) "Why is cost $(cost) but max tF $(maximum(get_tF(sched)))?"
    @test validate(sched)

    search_env = construct_search_env(solver,deepcopy(sched),pctapf.env)
    path_planner = CRCBS.low_level(solver.path_planner)
    node = initialize_root_node(search_env)
    # OptimalityGap shows up here
    set_verbosity!(low_level(path_planner),3)
    @test compute_route_plan!(path_planner,PC_MAPF(search_env),node)

    compile_replanning_results!(plannerA.cache,plannerA.solver,envA,
        resultsA,prob,stage,request)

    Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))
    # set_real_time_flag!(planner_config.planner,false)
    # @info "Profiling $(planner_config.planner_name)"
    # # profile
    # profile_replanner!(loader,
    #     planner_config.planner,
    #     base_problem_dir,
    #     planner_config.results_path)
end