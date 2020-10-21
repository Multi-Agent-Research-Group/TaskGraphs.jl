CRCBS.build_env(solver,pc_mapf::AbstractPC_MAPF,env::SearchEnv,node::ConstraintTreeNode,i::Int) = build_env(solver,pc_mapf,env,node,AgentID(i))
function construct_heuristic_model(solver::PIBTPlanner,env_graph;
        ph = PerfectHeuristic(get_dist_matrix(env_graph)),
        kwargs...)
    construct_composite_heuristic(ph,ph,NullHeuristic())
end
function construct_cost_model(solver::PIBTPlanner,
        schedule, cache, problem_spec, env_graph, primary_objective=SumOfMakeSpans(); kwargs...)
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(schedule,cache),
        FullCostModel(maximum,TravelTime()),
        FullCostModel(maximum,TravelDistance())
        )
    heuristic_model = construct_heuristic_model(solver,env_graph;kwargs...)
    cost_model, heuristic_model
end

function CRCBS.is_consistent(cache::PIBTCache,pc_mapf::PC_MAPF)
    search_env = CRCBS.get_solution(cache)
    n_jobless = 0
    # for e in CRCBS.get_envs(cache)
    #     if !CRCBS.is_valid(get_goal(e))
    #         n_jobless += 1
    #     end
    # end
    if length(search_env.cache.closed_set) + n_jobless >= nv(search_env.schedule)
        return true
    elseif !any(CRCBS.is_valid, map(e->CRCBS.get_goal(e),CRCBS.get_envs(cache)))
        # all robots are done with their jobs
        return true
    end
    return false
end

function CRCBS.pibt_priority_law(solver,pc_mapf::PC_MAPF,cache,i)
    env = CRCBS.get_envs(cache)[i]
    return (
        ~CRCBS.is_valid(env,get_goal(env)),
        ~isa(env.schedule_node,DEPOSIT),
        ~isa(env.schedule_node,CARRY),
        ~isa(env.schedule_node,COLLECT),
        minimum(cache.solution.cache.slack[get_vtx(
            cache.solution.schedule,env.node_id)]),
        -CRCBS.get_timers(cache)[i],
        i
    )
end

function CRCBS.pibt_preprocess!(solver,pc_mapf::PC_MAPF,cache)
    update_planning_cache!(solver,CRCBS.get_solution(cache))
    CRCBS.pibt_update_envs!(solver,pc_mapf,cache)
end

"""
    CRCBS.pibt_update_solution!(solver,pc_mapf::PC_MAPF,solution::SearchEnv,cache)

Overridden to update the SearchEnv
"""
function CRCBS.pibt_update_solution!(solver,solution::SearchEnv,cache)
    # update route plan
    CRCBS.pibt_update_solution!(solver,solution.route_plan,cache)
    # NOTE Update planning cache so that all goal times can be updated
    update_planning_cache!(solver,solution)
    solution
end
function pibt_info_string(cache,i,
        env=CRCBS.get_envs(cache)[i],
        s=CRCBS.get_states(cache)[i],
        a=CRCBS.get_actions(cache)[i]
    )
    string("\t",i,": ", get_vtx(cache.solution.schedule,env.node_id)," ",
        string(env.schedule_node)," s=",string(s),", a=",string(a),
        ", goal=",string(env.goal),"\n")
end
function pibt_info_strings(cache)
    info_strings = String[]
    for i in CRCBS.get_ordering(cache)
        # s = CRCBS.get_states(cache)[i]
        # a = CRCBS.get_actions(cache)[i]
        # env = CRCBS.get_envs(cache)[i]
        # env = cache.envs[i]
    # for (i,(s,env)) in enumerate(zip(cache.states,cache.envs))
        # str = "\t$i : $(get_vtx(cache.solution.schedule,env.node_id)) $(string(env.schedule_node)), s=$(string(s)), a=$(string(a)), goal=$(string(env.goal))\n"
        str = pibt_info_string(cache,i)
        push!(info_strings,str)
    end
    return info_strings
end
function CRCBS.pibt_update_envs!(solver,pc_mapf::PC_MAPF,cache)
    @log_info(2,solver,"PIBT iteration $(iterations(solver)) PRE UPDATE:\n",
        pibt_info_strings(cache)...)
    solution = CRCBS.get_solution(cache)
    node = initialize_root_node(solver,pc_mapf)
    done = true
    sweeps = 0
    while true
        done = true
        for (i,p) in enumerate(get_paths(solution))
            v_next = get_next_vtx_matching_agent_id(solution,i)
            if has_vertex(solution.schedule,v_next)
                # rebuild env to ensure that the goal time is up to date
                CRCBS.get_envs(cache)[i] = build_env(solver,pc_mapf,solution,node,AgentID(i))
            end
            env = CRCBS.get_envs(cache)[i]
            v = get_vtx(solution.schedule,env.node_id)
            sp = get_final_state(p)
            if (v in solution.cache.active_set) && is_goal(env,sp) && CRCBS.is_valid(env,get_goal(env))
                done = false
                update_env!(solver,solution,v,p)
                update_planning_cache!(solver,solution)
                v_next = get_next_vtx_matching_agent_id(solution,i)
                @assert v_next != v
                if has_vertex(solution.schedule,v_next)
                    CRCBS.get_timers(cache)[i] = 0
                end
            end
        end
        sweeps += 1
        done ? break : nothing
    end
    # @show sweeps

    # for (i,p) in enumerate(get_paths(solution))
    #     # NOTE rebuild all envs to ensure that goal times are up to date
    #     env = CRCBS.get_envs(cache)[i]
    #     v = get_vtx(solution.schedule,env.node_id)
    #     v_next = get_next_vtx_matching_agent_id(solution,i)
    #     sp = get_final_state(p)
    #     if has_vertex(solution.schedule,v_next)
    #         # @log_info(2,solver,"OLD: ",pibt_info_string(cache,i))
    #         # if v_next != v
    #             CRCBS.get_envs(cache)[i] = build_env(solver,pc_mapf,solution,node,AgentID(i))
    #         # end
    #         # @log_info(2,solver,"NEW: ",pibt_info_string(cache,i))
    #     end
    #     env = CRCBS.get_envs(cache)[i]
    #     # Update the SearchEnv and rebuild any low level envs for which
    #     # the goal has been reached
    #     if is_goal(env,sp) && CRCBS.is_valid(env,get_goal(env))
    #         v = get_vtx(solution.schedule,env.node_id)
    #         # NOTE Is update_env! the reason for the delays? I.e., does it unnecessarily push back solution.cache.tF?
    #         update_env!(solver,solution,v,p)
    #         update_planning_cache!(solver,solution)
    #         # NOTE Even if the current goal is reached, we only want to build a
    #         # new env if there is a valid available next vtx.
    #         if has_vertex(solution.schedule,get_next_vtx_matching_agent_id(solution,i))
    #             CRCBS.get_timers(cache)[i] = 0
    #             CRCBS.get_envs(cache)[i] = build_env(solver,pc_mapf,solution,node,AgentID(i))
    #         end
    #     end
    # end
    @log_info(2,solver,"PIBT iteration $(iterations(solver)) POST UPDATE:\n",
        pibt_info_strings(cache)...)
    return cache
end
