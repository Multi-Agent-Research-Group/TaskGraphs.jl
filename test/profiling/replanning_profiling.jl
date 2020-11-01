using TaskGraphs, CRCBS, TOML

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
base_problem_dir    = joinpath(base_dir,"problem_instances")
# base_results_dir    = joinpath(base_dir,"results")
base_results_dir    = joinpath(base_dir,"results_no_fail")

loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

reset_task_id_counter!()
reset_operation_id_counter!()
# get probem config
problem_configs = replanning_config_3()
# # write problems
# Random.seed!(0)
# write_problems!(loader,problem_configs,base_problem_dir)

feats = [
    RunTime(),IterationCount(),LowLevelIterationCount(),
    TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
    ObjectPathSummaries(),
    ]
final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
planner_configs = []
for (primary_replanner, backup_replanner) in [
        (NullReplanner(),       DeferUntilCompletion()),
        (MergeAndBalance(),     DeferUntilCompletion()),
        (ReassignFreeRobots(),  DeferUntilCompletion()),
        (DeferUntilCompletion(),DeferUntilCompletion()),
    ]
    # Primary planner
    path_finder = DefaultAStarSC()
    set_iteration_limit!(path_finder,5000)
    primary_route_planner = CBSSolver(ISPS(path_finder))
    set_iteration_limit!(primary_route_planner,1000)
    primary_planner = FullReplanner(
        solver = NBSSolver(path_planner=primary_route_planner),
        replanner = primary_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_max_time_limit!(primary_planner,50)
    set_commit_threshold!(primary_planner,10)
    # Backup planner
    backup_planner = FullReplanner(
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true)
            ),
        replanner = backup_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_iteration_limit!(backup_planner,1)
    set_iteration_limit!(route_planner(backup_planner.solver),5000)
    set_commit_threshold!(backup_planner,10)
    set_debug!(backup_planner,true)
    # set_debug!(route_planner(backup_planner.solver),true)
    # Full solver
    planner = ReplannerWithBackup(primary_planner,backup_planner)
    planner_config = (
        planner=planner,
        planner_name=string(typeof(primary_replanner)),
        results_path=joinpath(base_results_dir,string(typeof(primary_replanner))),
        objective = SumOfMakeSpans(),
    )
    push!(planner_configs,planner_config)
end

# for planner_config in planner_configs
#     # warm up to precompile replanning code
#     warmup(planner_config.planner,loader)
#     # profile
#     profile_replanner!(loader,
#         planner_config.planner,
#         base_problem_dir,
#         planner_config.results_path)
# end

# if results show ''"CRCBS.Feature" = val', use the following line to convert:
# sed -i 's/\"CRCBS\.\([A-Za-z]*\)\"/\1/g' **/**/*.toml

Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

# # plotting results

df_dict = Dict{String,DataFrame}()
for planner_config in planner_configs
    config_df = construct_config_dataframe(loader,base_problem_dir,problem_configs[1])
    plotting_config = (
        feats = Dict(
                :makespans => Int[],
                :arrival_times => Int[],
                :completion_times => Int[],
                :backup_flags => Bool[],
                :runtime_gaps => Float64[],
                :primary_runtimes => Float64[],
                :backup_runtimes => Float64[],
        ),
        results_path = joinpath(base_results_dir,planner_config.planner_name),
        problem_path = base_problem_dir,
    )
    results_df = TaskGraphs.construct_replanning_results_dataframe(loader,plotting_config,plotting_config.feats)
    df = innerjoin(config_df,results_df;on=:problem_name)
    df.projects_per_second = df.num_projects ./ map(maximum,df.completion_times)
    df.backlog_factor = (df.num_projects .+ 1) .* df.arrival_interval ./ df.projects_per_second
    df.tasks_per_second = df.projects_per_second .* df.M
    df_dict[planner_config.planner_name] = df
end
df_list = Vector{DataFrame}([df_dict[config.planner_name] for config in planner_configs])
for (k,df) in df_dict
    df.backlog_factor = map(maximum, df.completion_times) ./ ((df.num_projects .+ 1) .* df.arrival_interval)
    df.avg_makespan = map(sum, df.makespans) ./ df.num_projects
end

# table of rates
# tab = build_table(df_dict["MergeAndBalance"])
# table_product([build_table(df_dict["MergeAndBalance"]),build_table(df_dict["DeferUntilCompletion"])])
xkey = :M
ykey = :arrival_interval
f = vals -> sum(vals)/length(vals)
table_base_dir = joinpath(pwd(),"tables")
mkpath(table_base_dir)
for obj in [:tasks_per_second,:backlog_factor,:avg_makespan]
    tables = []
    for k in ["MergeAndBalance","ReassignFreeRobots","DeferUntilCompletion","NullReplanner"]
        df = df_dict[k]
        tab = build_table(df;obj=obj,xkey=xkey,ykey=ykey,aggregator=f)
        push!(tables,tab)
        # write_tex_table(joinpath(table_base_dir,"$(string(obj))_$(k).tex"),tab)
    end
    composite_table = table_product(tables)
    write_tex_table(joinpath(table_base_dir,"$(string(obj))_composite.tex"),
        composite_table;
        print_func=print_multi_value_real,
        header_printer=print_my_latex_header,
        row_start_printer=print_my_latex_row_start,
        )
end

plot_histories_pgf(df_list;
    y_key=:makespans,
    x_key=:none,
    include_keys = [:num_projects=>30,:M=>10,:arrival_interval=>40],
    xlabel = "time",
    ylabel = "makespans",
    ytick_show = true,
    ymode="linear",
    lines=false
)

plt = PGFPlots.GroupPlot(3,5)
for (n_vals, m_vals) in [
    ([10,],[20,30,40,]),
    ([15,],[30,40,50,]),
    ([20,],[40,50,60,]),
    ([25,],[50,60,70,]),
    ([30,],[60,70,80,]),
    ]
    gp = group_history_plot(df_list;
        y_key = :makespans,
        x_key=:none,
        n_key = :M,
        n_vals = n_vals,
        m_key = :arrival_interval,
        m_vals = m_vals,
        ymode="log",
        lines=false,
        use_y_lims=true
    )
    append!(plt.axes,gp.axes)
end
display(plt)

for (df,planner_config) in zip(df_list,planner_configs)
    plt = PGFPlots.GroupPlot(3,5)
    for (n_vals, m_vals) in [
        ([10,],[20,30,40,]),
        ([15,],[30,40,50,]),
        ([20,],[40,50,60,]),
        ([25,],[50,60,70,]),
        ([30,],[60,70,80,]),
        ]
        gp = group_history_plot([df];
            y_key = :primary_runtimes,
            x_key=:none,
            n_key = :M,
            n_vals = n_vals,
            m_key = :arrival_interval,
            m_vals = m_vals,
            ymode="log", #"linear"
            lines=false,
        )
        gp2 = group_history_plot([df];
            y_key = :backup_runtimes,
            x_key=:none,
            n_key = :M,
            n_vals = n_vals,
            m_key = :arrival_interval,
            m_vals = m_vals,
            ymode="log",
            lines=false,
            colors=["blue"],
        )
        for (ax,ax2) in zip(gp.axes,gp2.axes)
            append!(ax.plots,ax2.plots)
        end

        append!(plt.axes,gp.axes)
    end
    display(plt)
    save("/home/kylebrown/Desktop/$(planner_config.planner_name)_runtimes.pdf",plt)
end
