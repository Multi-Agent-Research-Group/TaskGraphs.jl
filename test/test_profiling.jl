let

    println("WARMING UP")

    dummy_problem_dir = "dummy_problem_dir"
    dummy_results_dir = "dummy_results_dir"
    modes = [
        :write,
        :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        :full_solver
        ]
    milp_models = [
        # AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP()
    ]
    for milp_model in milp_models
        for mode in modes
            run_profiling(mode;
                num_tasks=[18],
                num_robots=[24],
                depth_biases=[0.1],
                task_size_distributions = [
                    # ( 1=>1.0, 2=>0.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>1.0 ),
                    # ( 1=>0.0, 2=>1.0, 4=>1.0 ),
                    ],
                num_trials=1,
                problem_dir = dummy_problem_dir,
                results_dir = joinpath(dummy_results_dir, string(typeof(milp_model))),
                milp_model = milp_model,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                TimeLimit = 20,
                solver_template = PC_TAPF_Solver(
                    verbosity=0,
                    l2_verbosity=2,
                    l3_verbosity=0,
                    l4_verbosity=0,
                    LIMIT_A_star_iterations=8000,
                    time_limit=25
                    )
                )
        end
        run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
        run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
    end
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        # :full_solver
        ]
    results_dirs = [
        # joinpath(EXPERIMENT_DIR,"assignment_solver/results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        # joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/results")
    ]
    milp_models = [
        AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP()
    ]
    for (milp_model, results_dir) in zip(milp_models, results_dirs)
        for mode in modes
            run_profiling(mode;
                num_tasks = [10,20,30,40,50,60],
                num_robots = [10,20,30,40],
                depth_biases = [0.1,0.4,0.7,1.0],
                max_parent_settings = [3],
                num_trials = 4,
                env_id = 2,
                initial_problem_id = 1,
                problem_dir = PROBLEM_DIR,
                results_dir = results_dir,
                TimeLimit=100,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                milp_model = milp_model
                )
        end
    end
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        # :write,
        # :assignment_only,
        :low_level_search_without_repair,
        :low_level_search_with_repair,
        :full_solver
        ]
    problem_dir = joinpath(PROBLEM_DIR,"collaborative_transport")
    results_dirs = [
        # joinpath(EXPERIMENT_DIR,"assignment_solver/results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/collaborative_transport_dist_maps/results")
    ]
    milp_models = [
        # AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP()
    ]
    for (milp_model, results_dir) in zip(milp_models, results_dirs)
        for mode in modes
            run_profiling(mode;
                num_tasks = [12,18,24],
                num_robots = [24],
                depth_biases=[0.1],
                task_size_distributions = [
                    ( 1=>1.0, 2=>0.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>1.0 ),
                    ( 1=>0.0, 2=>1.0, 4=>1.0 ),
                    ],
                num_trials=16,
                milp_model = milp_model,
                problem_dir = problem_dir,
                results_dir = results_dir,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                TimeLimit = 100, # 50
                solver_template = PC_TAPF_Solver(
                    verbosity=1,
                    l1_verbosity=1,
                    l2_verbosity=1,
                    l3_verbosity=3,
                    l4_verbosity=2,
                    LIMIT_A_star_iterations=8000,
                    time_limit=200 # 60
                    )
                )
        end
    end
end

let
    results_dir = joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/collaborative_transport_dist_maps/results")
    for (root, dirs, files) in walkdir(results_dir)
        for file in files
            if splitext(file)[end] == ".toml"
                toml_dict = TOML.parsefile(joinpath(root, file))
                if get(toml_dict, "optimal", true) == false
                    println(joinpath(root, file)) # path to files
                    # rm(joinpath(root,file))
                end
            end
        end
    end

end
