using TaskGraphs

using Dates
## set up the environment
using GraphUtils

indicator_grid = IndicatorGrid(zeros(Bool,7,5))                           # initialize a 10x10 environment with no obstacles
indicator_grid[4,4] = true                                                                # obstacle at position 4,4
indicator_grid[1:6,2] .= true
indicator_grid[1:6,4] .= true                                                          # obstacle covering positions 6:7,6:7
env = construct_factory_env_from_indicator_grid(indicator_grid) # initialize the factory environment
@show env.vtx_map                                                                     # visualize the mapping from grid position to vertex indices

## set up the environment
# vtx_grid = initialize_dense_vtx_grid(7,5) # 4 x 4 grid world
#  1   2   3   4	5
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
# env = construct_factory_env_from_vtx_grid(vtx_grid)
# add_obstacle!(env)
## Define the initial conditions of the robots
robot_ics = [
    ROBOT_AT(1,23), # robot 1 starts at vertex 2
    ROBOT_AT(2,20), # robot 2 starts at vertex 9
]

## Define the manufacturing project
spec = ProjectSpec()
# set initial conditions of "raw materials"
set_initial_condition!(spec,OBJECT_AT(1,23)) # object 1 starts at vertex 4
set_initial_condition!(spec,OBJECT_AT(2,20))  # object 2 starts at vertex 

# set_initial_condition!(spec,OBJECT_AT(5,7))
# set_initial_condition!(spec,OBJECT_AT(6,2))
# set_initial_condition!(spec,OBJECT_AT(7,12)) 

# define the operations that need to take place
# define the operations that need to take place

op1 = Operation(Δt=0) 
op2 = Operation(Δt=0)

# inputs
set_precondition!(op1,OBJECT_AT(1,17))
set_precondition!(op2,OBJECT_AT(2,11)) 

# outputs
set_postcondition!(op1,OBJECT_AT(3,14))
set_postcondition!(op2,OBJECT_AT(4,8))  
add_operation!(spec,op1)
add_operation!(spec,op2)


# add a terminal operation with no outputs
op3 = Operation(Δt=0) 
op4 = Operation(Δt=0)

# outputs
set_precondition!(op3,OBJECT_AT(3,2))
set_precondition!(op4,OBJECT_AT(4,5))  
add_operation!(spec,op3)
add_operation!(spec,op4)

## define solver
# solver = NBSSolver()
solver = NBSSolver(
    assignment_model = TaskGraphsMILPSolver(AssignmentMILP()),
    path_planner = CBSSolver(ISPS())
    )
# finalize problem construction (the solver is passed as an argument here 
# because it determines what cost model is used for the problem)
prob = pctapf_problem(solver,spec,env,robot_ics)
# solve the problem
# solution, cost = solve!(solver,prob)
# println(env.vtx_map)
# println(solution)
# println(cost)
# # check if the problem was solved to optimality
# reset_solver!(solver)
start_time = Dates.format(now(), "HH:MM:SS")
println()
cbs = solver.path_planner
isps = cbs.low_level_planner
astar = isps.low_level_planner
# set_verbosity!(cbs, 0)
# set_verbosity!(isps, 0)
# set_verbosity!(astar, 0)

reset_solver!(solver)
set_iteration_limit!(cbs, 10000000)
set_verbosity!(solver, 0)
# set_verbosity!(cbs, 0)
# set_verbosity!(isps, 0)
# set_verbosity!(astar, 0)

solve!(solver, prob)
println()

println(start_time)
println(Dates.format(now(), "HH:MM:SS"))
@show feasible_status(solver)
@show optimal_status(solver)