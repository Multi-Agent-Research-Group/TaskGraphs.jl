using Pkg
Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using ImageFiltering
using Gurobi
using JuMP, MathOptInterface
using TOML
using JLD2, FileIO
using Random
using Test
using GraphPlottingBFS
using Compose
# load rendering tools
include(joinpath(pathof(TaskGraphs),"../..","test/notebooks/render_tools.jl"))
# for f in *.svg; do inkscape -z $f -e $f.png; done

let
    project_spec, problem_spec, robot_ICs, env_graph, assignments = pctapf_problem_1(;verbose=false);
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    print_project_schedule(project_schedule,"project_schedule1";mode=:leaf_aligned)
end
# let

# Backtracking motivating example
let
    f = pctapf_problem_10
    cost_model = MakeSpan
    project_spec, problem_spec, robot_ICs, env_graph, assignments = f(;cost_function=cost_model,verbose=true)
    solver = PC_TAPF_Solver(nbs_model=AssignmentMILP(),l1_verbosity=2,l2_verbosity=2,l3_verbosity=0)

    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    model = formulate_milp(solver.nbs_model,project_schedule,problem_spec;cost_model=cost_model)
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    cost = Int(round(value(objective_function(model))))
    adj_matrix = get_assignment_matrix(model)
    update_project_schedule!(solver.nbs_model,project_schedule,problem_spec,adj_matrix)
    set_leaf_operation_vtxs!(project_schedule)
    @test validate(project_schedule)
    print_project_schedule(project_schedule,"backtracking_schedule";mode=:root_align)

    solution, assignment, cost, env = high_level_search!(
        solver,
        env_graph,
        project_spec,
        problem_spec,
        robot_ICs,
        Gurobi.Optimizer;
        primary_objective=cost_model,
        )
    convert_to_vertex_lists(solution), cost

    # robot_paths = convert_to_vertex_lists(solution)
    # object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,env.schedule,env.cache)
    # object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)
    #
    # # # Render video clip
    # tf = maximum(map(p->length(p),robot_paths))
    # set_default_plot_size(24cm,24cm)
    # record_video(joinpath(VIDEO_DIR,string("toy_example.webm")),
    #     t->render_paths(t,env_graph,robot_paths,object_paths;
    #         object_intervals=object_intervals,
    #         colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
    #         show_paths=false);tf=tf)
end
# end

# For catching troublesome problem instances
# let
#     env_id = 2
#     env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
#     factory_env = read_env(env_filename)
#     env_graph = factory_env.graph
#     dist_matrix = get_dist_matrix(env_graph)
#     logfile = "log.txt"
#
#     TimeLimit = 40
#     OutputFlag = 0
#     problem_dir = PROBLEM_DIR
#
#     problematic_ids = [
#         43, # one of the solvers gets stuck after A* returns an infeasible path (twice)
#         146, # A* infeasible, again.
#         197, # can't remember why I put this on here
#         255, # more A* infeasible. These always seem to terminate with "bounds error"
#         267, # just pausing here--nothing necessarily wrong.
#         146, # TODO why does this fail for SparseAdjacencyMILP?
#         ]
#
#     ##
#     # for problem_id in problematic_ids[end]+1:384
#     for problem_id in 1:10
#         problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
#         problem_def = read_problem_def(problem_filename)
#         project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
#         project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)
#         println("PROBLEM ID: ", problem_id)
#         for cost_model in [SumOfMakeSpans, MakeSpan]
#             costs = Float64[]
#             for milp_model in [AdjacencyMILP(),SparseAdjacencyMILP()]
#                 try
#                     solver = PC_TAPF_Solver(verbosity=0)
#                     solution, assignment, cost, env = high_level_search!(
#                         milp_model,
#                         solver,
#                         env_graph,
#                         project_spec,
#                         problem_spec,
#                         robot_ICs,
#                         Gurobi.Optimizer;
#                         primary_objective=cost_model,
#                         TimeLimit=TimeLimit
#                         )
#                     push!(costs, cost[1])
#                     @assert validate(env.schedule)
#                     @assert cost[1] != Inf
#                 catch e
#                     open(logfile, "a") do io
#                         write(io, string("PROBLEM ", problem_id, " - ",
#                             "cost model: ", cost_model, " - ",
#                             typeof(milp_model), " - ", e.msg, "\n"))
#                     end
#                 end
#             end
#             try
#                 @assert all(costs .== costs[1])
#             catch e
#                 open(logfile, "a") do io
#                     write(io, string("PROBLEM ", problem_id, " - ",
#                         "cost model: ", cost_model, " - ",
#                          e.msg, " costs: ", costs, "\n"))
#                 end
#             end
#         end
#     end
#     ##
# end

# for REPLANNING
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    # factory_env = construct_regular_factory_world(;
    #     n_obstacles_x=2,
    #     n_obstacles_y=2,
    #     obs_width = [2;2],
    #     obs_offset = [2;2],
    #     env_pad = [1;1],
    #     env_offset = [1,1],
    #     env_scale = 1 # this is essentially the robot diameter
    # )
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)

    # Experimental params

    # seed = 0
    solver_template = PC_TAPF_Solver(
        nbs_model=SparseAdjacencyMILP(),
        DEBUG=true,
        l1_verbosity=1,
        l2_verbosity=1,
        l3_verbosity=0,
        l4_verbosity=0,
        LIMIT_assignment_iterations=5,
        LIMIT_A_star_iterations=8000
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model=GreedyAssignment(),
        astar_model = PrioritizedAStarModel(),
        DEBUG=true,
        l1_verbosity=1,
        l2_verbosity=1,
        l3_verbosity=0,
        l4_verbosity=0,
        LIMIT_assignment_iterations=2,
        LIMIT_A_star_iterations=8000
        );

    # replan_model = MergeAndBalance()
    # replan_model = ReassignFreeRobots()
    replan_model = DeferUntilCompletion()
    fallback_model = ReassignFreeRobots()

    primary_objective = SumOfMakeSpans

    ################################################################################
    ############################## Define Project List #############################
    ################################################################################
    seed = 0
    Random.seed!(seed)
    # seed = seed + 1
    # N = 10
    # M = 10
    N = 4
    M = 6
    max_parents = 3
    depth_bias = 0.4
    Δt_min = 0
    Δt_max = 0
    task_sizes = (1=>1.0,2=>0.0,4=>0.0) # all single agent tasks for now
    stream_length = 5
    project_list = SimpleProblemDef[]
    for i in 1:stream_length
        r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
                get_free_zones(factory_env))
        project_spec = construct_random_project_spec(M,s0,sF;max_parents=max_parents,depth_bias=depth_bias,Δt_min=Δt_min,Δt_max=Δt_max)
        shapes = choose_random_object_sizes(M,Dict(task_sizes...))
        push!(project_list, SimpleProblemDef(project_spec,r0,s0,sF,shapes))
    end

    arrival_interval            = 30 # new project requests arrive every `arrival_interval` timesteps
    warning_time                = 20 # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold            = 5 # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    fallback_commit_threshold   = 2 # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + fallback_commit_threshold--just in case the solver fails

    ################################################################################
    ############################## Simulate Replanning #############################
    ################################################################################

    solver = PC_TAPF_Solver(solver_template);
    fallback_solver = PC_TAPF_Solver(fallback_solver_template);

    idx = 1
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, dist_matrix);
    partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(solver,partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)

    # store solution for plotting
    project_ids = Dict{Int,Int}()
    for (object_id, pred) in get_object_ICs(partial_schedule)
        project_ids[get_id(object_id)] = idx
    end
    object_path_dict = Dict{Int,Vector{Vector{Int}}}()
    object_interval_dict = Dict{Int,Vector{Int}}()

    print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule,base_search_env.cache;mode=:leaf_aligned)
    (solution, _, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
    print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)

    while idx < length(project_list)
        project_schedule = search_env.schedule
        cache = search_env.cache
        idx += 1
        t_request = arrival_interval * (idx - 1) # time when request reaches command center
        t_arrival = t_request + warning_time # time when objects become available
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix);
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        print_project_schedule(string("next_schedule",idx),next_schedule;mode=:leaf_aligned)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache,object_path_dict,object_interval_dict)
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Fallback
        fallback_search_env = replan(fallback_solver, fallback_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival;commit_threshold=fallback_commit_threshold)
        reset_solver!(fallback_solver)
        (fallback_solution, _, fallback_cost, fallback_search_env, _), fallback_elapsed_time, _, _, _ = @timed high_level_search!(
            fallback_solver, fallback_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"A"),fallback_search_env.schedule;mode=:leaf_aligned)
        # Replanning outer loop
        # TODO update fall_back plan
        # base_search_env = replan(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=commit_threshold)
        base_search_env = replan(solver, replan_model, fallback_search_env, env_graph, problem_spec, fallback_solution, nothing, t_request, t_arrival;commit_threshold=commit_threshold)
        print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule;mode=:leaf_aligned)
        # plan for current project
        reset_solver!(solver)
        (solution, _, cost, search_env, _), elapsed_time, _, _, _ = @timed high_level_search!(solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)
        m = maximum(map(p->length(p), get_paths(solution)))
        @show m
    end

    robot_paths = convert_to_vertex_lists(solution)
    object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,search_env.schedule,search_env.cache,object_path_dict,object_interval_dict)
    object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)
    project_idxs = map(k->project_ids[k], sort(collect(keys(project_ids))))

    # @show project_ids
    # @show project_idxs

    # # Render video clip
    # tf = maximum(map(p->length(p),robot_paths))
    # set_default_plot_size(24cm,24cm)
    # record_video(joinpath(VIDEO_DIR,string("replanning4.webm")),
    #     t->render_paths(t,factory_env,robot_paths,object_paths;
    #         object_intervals=object_intervals,
    #         colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
    #         project_idxs=project_idxs,
    #         show_paths=false);tf=tf)

end

# for REPLANNING
let

    env_id=2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)
    dist_mtx_map = DistMatrixMap(factory_env.vtx_map,factory_env.vtxs)

    base_solver_configs = [
        Dict(
        :nbs_time_limit=>8,
        :route_planning_buffer=>2,
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-105,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,replan_configs,fallback_configs)
        push!(solver_configs,merge(dicts...))
    end
    base_configs = [
        Dict(
            :warning_time=>20,
            :commit_threshold=>10,
            :fallback_commit_threshold=>10,
            :num_trials => 4,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10, :num_projects=>10, :arrival_interval=>40, ),
        Dict(:N=>30, :M=>15, :num_projects=>10, :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20, :num_projects=>10, :arrival_interval=>60, ),
        Dict(:N=>30, :M=>25, :num_projects=>10, :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30, :num_projects=>10, :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        astar_model                 = PrioritizedAStarModel(),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 2,
        LIMIT_A_star_iterations     = 8000
        );


    base_dir            = joinpath(EXPERIMENT_DIR,"replanning")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    solver_config = solver_configs[4]
    problem_config = problem_configs[1]
    primary_objective = SumOfMakeSpans
    folder_id = 5
    # reset_operation_id_counter!()
    # reset_action_id_counter!()
    # run_replanner_profiling(:write;
    #     problem_configs=problem_configs,
    #     base_problem_dir=base_problem_dir,
    #     )
    modes = [:solve]
    # for solver_config in solver_configs
    replan_model = get(solver_config,:replan_model,  MergeAndBalance())
    fallback_model  = get(solver_config,:fallback_model,ReassignFreeRobots())
    results_dir     = get(solver_config,:results_dir,   joinpath(
        base_results_dir, string(typeof(replan_model),"-",typeof(fallback_model))))
        # for mode in modes
        #     reset_operation_id_counter!()
        #     reset_action_id_counter!()
        #     run_replanner_profiling(mode;
        #         solver_config=solver_config,
        #         problem_configs=problem_configs,
        #         base_problem_dir=base_problem_dir,
        #         base_results_dir=results_dir,
        #         solver_template=solver_template,
        #         fallback_solver_template=fallback_solver_template,
        #         primary_objective=SumOfMakeSpans,
        #         )
        # end
    # end

    Random.seed!(1)
    # folder_id = initial_problem_id-1;
    # run tests and push results into table
    # for problem_config in problem_configs

    num_trials          = get(problem_config,:num_trials,1)
    max_parents         = get(problem_config,:max_parents,3)
    depth_bias          = get(problem_config,:depth_bias,0.4)
    dt_min              = get(problem_config,:dt_min,0)
    dt_max              = get(problem_config,:dt_max,0)
    dt_collect          = get(problem_config,:dt_collect,0)
    dt_deliver          = get(problem_config,:dt_deliver,0)
    task_sizes          = get(problem_config,:task_sizes,(1=>1.0,2=>0.0,4=>0.0))
    N                   = get(problem_config,:N,30)
    M                   = get(problem_config,:M,10)
    num_projects        = get(problem_config,:num_projects,10)
    arrival_interval    = get(problem_config,:arrival_interval,40)
    # for trial in 1:num_trials
    # try
    # folder_id += 1 # moved this to the beginning so it doesn't get skipped
    problem_dir = joinpath(base_problem_dir,string("stream",folder_id))

    project_list = SimpleProblemDef[]
    for problem_id in 1:num_projects
        problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
        config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
        # config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
        problem_def = read_problem_def(problem_filename)
        push!(project_list, problem_def)
    end

    solver = PC_TAPF_Solver(solver_template,start_time=time());
    fallback_solver = PC_TAPF_Solver(fallback_solver_template,start_time=time());

    arrival_interval            = problem_config[:arrival_interval] # new project requests arrive every `arrival_interval` timesteps
    warning_time                = problem_config[:warning_time] # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold            = problem_config[:commit_threshold] # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    fallback_commit_threshold   = problem_config[:fallback_commit_threshold] # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + fallback_commit_threshold--just in case the solver fails

    local_results = map(p->Dict{String,Any}(),project_list)
    final_times = map(p->Dict{AbstractID,Int}(),project_list)

    idx = 1
    t_arrival = 0
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, dist_matrix;
        Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
        Δt_deliver=map(i->get(problem_config,:dt_deliver,0), def.s0),
        cost_function=primary_objective,
        task_shapes=def.shapes,
        shape_dict=env_graph.expanded_zones,
        );
    partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(solver,partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)

    # store solution for plotting
    project_ids = Dict{Int,Int}()
    for (object_id, pred) in get_object_ICs(partial_schedule)
        project_ids[get_id(object_id)] = idx
    end
    object_path_dict = Dict{Int,Vector{Vector{Int}}}()
    object_interval_dict = Dict{Int,Vector{Int}}()

    # print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule;mode=:leaf_aligned)
    (solution, _, cost, search_env, optimality_gap), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
    # print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)
    local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
    merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>t_arrival for k in keys(get_operations(partial_schedule))))
    for i in 1:idx
        for node_id in keys(final_times[i])
            v = get_vtx(search_env.schedule,node_id)
            final_times[i][node_id] = max(final_times[i][node_id], get(search_env.cache.tF, v, -1))
        end
    end

    while idx < length(project_list)
        project_schedule = search_env.schedule
        cache = search_env.cache
        idx += 1
        t_request = arrival_interval * (idx - 1) # time when request reaches command center
        t_arrival = t_request + warning_time # time when objects become available
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix;
            Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
            Δt_deliver=map(i->get(problem_config,:dt_deliver,0), def.s0),
            cost_function=primary_objective,
            task_shapes=def.shapes,
            shape_dict=env_graph.expanded_zones
        )
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        print_project_schedule(string("next_schedule",idx),next_schedule;mode=:leaf_aligned)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache,object_path_dict,object_interval_dict)
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Fallback
        println("Computing Fallback plan at idx = ",idx)
        fallback_search_env = replan(fallback_solver, fallback_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival;commit_threshold=fallback_commit_threshold)
        reset_solver!(fallback_solver)
        (fallback_solution, _, fallback_cost, fallback_search_env, fallback_optimality_gap), fallback_elapsed_time, _, _, _ = @timed high_level_search!(
            fallback_solver, fallback_search_env, Gurobi.Optimizer;primary_objective=primary_objective)

        print_project_schedule(string("schedule",idx,"A"),fallback_search_env;mode=:leaf_aligned)
        # Replanning outer loop
        # t_commit = get_commit_time(replan_model, search_env, t_request, commit_threshold)
        base_search_env = replan(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=commit_threshold)
        # base_search_env = replan(solver, replan_model, fallback_search_env, env_graph, problem_spec, fallback_solution, nothing, t_request, t_arrival;commit_threshold=commit_threshold)
        print_project_schedule(string("schedule",idx,"B"),base_search_env;mode=:leaf_aligned)
        # plan for current project
        println("Computing plan at idx = ",idx)
        reset_solver!(solver)
        (solution, _, cost, search_env, optimality_gap), elapsed_time, _, _, _ = @timed high_level_search!(solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"C"),search_env;mode=:leaf_aligned)
        # m = maximum(map(p->length(p), get_paths(solution)))
        # @show

        fallback_results = compile_solver_results(fallback_solver, fallback_solution, fallback_cost, fallback_search_env, fallback_optimality_gap, fallback_elapsed_time, t_arrival)
        local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
        merge!(local_results[idx],Dict(string("fallback_",k)=>v for (k,v) in fallback_results))
        # Switch to fallback if necessary
        if (elapsed_time > solver.time_limit) || ~local_results[idx]["feasible"]
            if local_results[idx]["feasible"]
                println("TIMEOUT! Using feasible solver output",idx)
            elseif local_results[idx]["fallback_feasible"]
                println("TIMEOUT! Solver failed to find feasible solution. Resorting to fallback plan on ",idx)
                solution = fallback_solution
                search_env = fallback_search_env
                cost = fallback_cost
            else
                println("TIMEOUT! No feasible solution found by fallback solver or regular solver. Terminating...",idx)
                merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>-1 for k in keys(get_operations(next_schedule))))
                break
            end
        end
        merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>t_arrival for k in keys(get_operations(next_schedule))))
        for i in 1:idx
            for node_id in keys(final_times[i])
                v = get_vtx(search_env.schedule,node_id)
                final_times[i][node_id] = max(final_times[i][node_id], get(search_env.cache.tF, v, -1))
            end
        end
    end
    results_dict = Dict{String,Any}()
    println("DONE REPLANNING - Aggregating results")
    results_dict["time"]                = map(i->local_results[i]["time"], 1:idx)
    results_dict["optimality_gap"]      = map(i->local_results[i]["optimality_gap"], 1:idx)
    results_dict["optimal"]             = map(i->local_results[i]["optimal"], 1:idx)
    results_dict["feasible"]            = map(i->local_results[i]["feasible"], 1:idx)
    results_dict["cost"]                = map(i->local_results[i]["cost"], 1:idx)
    results_dict["arrival_time"]        = map(i->local_results[i]["arrival_time"], 1:idx)

    results_dict["fallback_feasible"]   = map(i->local_results[i]["fallback_feasible"], 2:idx)

    println("COMPUTING MAKESPANS")
    results_dict["final_time"]         = map(i->maximum(collect(values(final_times[i]))),1:idx)
    results_dict["makespans"]          = [b-a for (a,b) in zip(results_dict["arrival_time"],results_dict["final_time"])]
    @show results_dict["makespans"]

    if cost[1] < Inf
        println("SAVING PATHS")
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,search_env.schedule,search_env.cache,object_path_dict,object_interval_dict)
        object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)

        results_dict["robot_paths"]         = convert_to_vertex_lists(solution)
        results_dict["object_paths"]        = object_paths
        results_dict["object_intervals"]    = object_intervals
        results_dict["project_idxs"]        = map(k->project_ids[k], sort(collect(keys(project_ids))))
    end
    results_dict
end

# Visualizing A star
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)

    validate_edge_cache(factory_env,factory_env.vtxs,factory_env.edge_cache)

    file_name = joinpath(DEBUG_PATH,"A_star_dump_54.jld2")
    dict = load(file_name)
    agent_id = dict["agent_id"]
    history = dict["history"]
    start   = dict["start"]
    goal    = dict["goal"]
    robot_paths = map(p->p.nzval, dict["paths"])
    state_constraints = dict["state_constraints"]
    action_constraints = dict["action_constraints"]

    # findall(map(p->p[212],robot_paths) .== 550)
    # robot_paths[24][170:end]

    t0 = start[2]
    history = history[1:400]
    empty!(robot_paths[agent_id])
    start,goal

    # Render video clip
    set_default_plot_size(24cm,24cm)
    record_video(joinpath(VIDEO_DIR,string("A_star_debug.webm")),
        k->render_search(history[k][2],factory_env;
            robot_paths=robot_paths,
            search_patterns=[history],
            goals=[goal[1]],
            search_idx=k,
            show_search_paths=true,
            # search_size=1.0pt,
            # goal_size=1.0pt,
            colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
            show_paths=false,
            );
            t_history=1:length(history)
            )

end

# Prioritized DFS
let

    i = 1
    f = pctapf_problem_1
    cost_model = MakeSpan
    for (i, f) in enumerate([
                pctapf_problem_1,
                pctapf_problem_2,
                pctapf_problem_3,
                pctapf_problem_4,
                pctapf_problem_5,
                pctapf_problem_6,
                pctapf_problem_7,
                pctapf_problem_8,
            ])
        project_spec, problem_spec, robot_ICs, env_graph, assignments = f(;verbose=false);
        milp_model = GreedyAssignment()
        project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
        model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
        optimize!(model)
        adj_matrix = get_assignment_matrix(model)
        update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)

        solver = PC_TAPF_Solver(
            cbs_model = PrioritizedDFSPlanner(),
            astar_model = DFS_PathFinder()
        )
        env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
        pc_mapf = PC_MAPF(env)

        route_plan, cache, cost = CRCBS.solve!(solver,pc_mapf)
        @test length(cache.active_set) == 0
        @test length(cache.closed_set) == nv(project_schedule)
    end
end
let

    # init env
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env
    # set up problem
    primary_objective = MakeSpan
    problem_filename="/scratch/task_graphs_experiments/problem_instances/problem180.toml"
    problem_def = read_problem_def(problem_filename)
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        project_spec, r0, s0, sF,
        factory_env.dist_function;
        cost_function=primary_objective,
        task_shapes=problem_def.shapes,
        shape_dict=factory_env.expanded_zones,
        );
    # define solver
    solver = PC_TAPF_Solver(
        # nbs_model = SparseAdjacencyMILP(),
        nbs_model = GreedyAssignment(),
        cbs_model = PrioritizedDFSPlanner(max_iters=400),
        astar_model = DFS_PathFinder(),
        LIMIT_assignment_iterations = 1,
        l1_verbosity=4,
        l2_verbosity=1
    )
    # solve
    solution, assignment, cost, search_env, optimality_gap = high_level_search!(
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        primary_objective=primary_objective);

    # record video
    robot_paths = convert_to_vertex_lists(solution)
    object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,search_env)
    tf = maximum(map(p->length(p),robot_paths))
    tf = max(tf,200)
    set_default_plot_size(24cm,24cm)
    record_video(joinpath(VIDEO_DIR,string("greedy_dfs.webm")),
        t->render_paths(t,factory_env,robot_paths,object_paths;
            object_intervals=object_intervals,
            colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
            show_paths=false);tf=tf)

end
# Visualizing dumped DFS solution
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)

    file_name = joinpath(DEBUG_PATH,"DFS_demo3.jld2")
    dict = load(file_name)
    robot_paths = dict["robot_paths"]
    object_paths = dict["object_paths"]
    object_intervals = dict["object_intervals"]
    object_ids = dict["object_ids"]
    path_idxs = dict["path_idxs"]


    # record video
    tf = maximum(map(p->length(p),robot_paths))
    t0 = 1500
    tf = min(tf,1700)
    robot_paths = map(p->p[min(t0+1,length(p)):end],robot_paths)
    object_paths = map(p->p[min(t0+1,length(p)):end],object_paths)
    object_intervals = map(p->p.-t0,object_intervals)
    set_default_plot_size(24cm,24cm)
    colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths))
    colors_vec[1] = LCHab(60,80,150)
    colors_vec[3] = LCHab(60,80,60)
    project_idxs=ones(Int,length(object_paths))
    project_idxs[findall(object_ids.==751)[1]] = 2
    project_idxs[findall(object_ids.==774)[1]] = 3
    record_video(joinpath(VIDEO_DIR,string("dfs_stuck.webm")),
        t->render_paths(t,factory_env,robot_paths,object_paths;
            object_intervals=object_intervals,
            project_idxs=project_idxs,
            colors_vec=colors_vec,
            show_paths=false);tf=tf-t0)

end

# Video for ICRA presentation
let

    # init env
    project_spec, problem_spec, robot_ICs, env_graph, assignments = pctapf_problem_3(;verbose=false);
    primary_objective=MakeSpan
    # define solver
    solver = PC_TAPF_Solver()
    # solver = PC_TAPF_Solver(
    #     # nbs_model = SparseAdjacencyMILP(),
    #     nbs_model = GreedyAssignment(),
    #     cbs_model = PrioritizedDFSPlanner(max_iters=400),
    #     astar_model = DFS_PathFinder(),
    #     LIMIT_assignment_iterations = 1,
    #     l1_verbosity=4,
    #     l2_verbosity=1
    # )
    # solve
    solution, assignment, cost, search_env, optimality_gap = high_level_search!(
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        primary_objective=primary_objective);

    # record video
    robot_paths = convert_to_vertex_lists(solution)
    object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,search_env)
    tf = maximum(map(p->length(p),robot_paths))
    tf = min(tf,200)
    set_default_plot_size(8cm,16cm)
    record_video(joinpath(VIDEO_DIR,string("toy_problem_3.webm")),
        t->render_paths(t,env_graph,robot_paths,object_paths;
            object_intervals=object_intervals,
            rsize=20pt,
            osize=15pt,
            cell_width=1.0,
            label_objects=true,
            label_robots=true,
            point_label_font="CMU Serif",
            point_label_color="white",
            point_label_font_size=20pt,
            colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
            active_object_colors=map(j->"black",object_paths),
            show_paths=false);tf=tf,s=(8inch,8inch))

end
# Load video for ICRA presentation
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    env_graph = read_env(env_filename)

    problem_id = 290
    base_path = "/scratch/task_graphs_experiments"
    file_path = joinpath(base_path,"assignment_solver/final_results/full_solver","results$problem_id.toml")
    toml_dict = TOML.parsefile(file_path)
    robot_paths = toml_dict["robot_paths"]
    object_paths = toml_dict["object_paths"]
    object_intervals = toml_dict["object_intervals"]
    object_ids = collect(1:length(object_paths))
    path_idxs = map(o->1,object_ids)

    tf = maximum(map(p->length(p),robot_paths))
    # tf = min(tf,200)
    set_default_plot_size(8cm,8cm)
    record_video(joinpath(VIDEO_DIR,string("demo.webm")),
        t->render_paths(t,env_graph,robot_paths,object_paths;
            object_intervals=object_intervals,
            rsize=7pt,
            osize=5pt,
            pickup_color= "light gray",
            dropoff_color="light gray",
            show_inactive_objects=false,
            cell_width=1.0,
            # label_objects=true,
            # label_robots=true,
            # point_label_font="CMU Serif",
            # point_label_color="white",
            # point_label_font_size=20pt,
            colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
            active_object_colors=map(j->"black",object_paths),
            show_paths=false);tf=tf,s=(8inch,8inch))


    render_paths(0,env_graph,robot_paths,object_paths;
        object_intervals=object_intervals,
        rsize=7pt,
        osize=5pt,
        pickup_color= "light gray",
        dropoff_color="light gray",
        show_inactive_objects=false,
        cell_width=1.0,
        # label_objects=true,
        # label_robots=true,
        # point_label_font="CMU Serif",
        # point_label_color="white",
        # point_label_font_size=20pt,
        colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
        active_object_colors=map(j->"black",object_paths),
        show_paths=false);tf=tf,s=(8inch,8inch) |> SVG("render.png", 8inch, 8inch)
end
