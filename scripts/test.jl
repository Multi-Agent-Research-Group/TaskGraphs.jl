using TaskGraphs


## set up the environment
using GraphUtils

indicator_grid = IndicatorGrid(zeros(Bool,7,5))                           # initialize a 10x10 environment with no obstacles
indicator_grid[4,4] = true  
indicator_grid[1,3] = true   
indicator_grid[2,3] = true                                                             # obstacle at position 4,4
indicator_grid[1:6,2] .= true
indicator_grid[1:6,4] .= true                                                          # obstacle covering positions 6:7,6:7
env = construct_factory_env_from_indicator_grid(indicator_grid) # initialize the factory environment
@show env.vtx_map                                                                     # visualize the mapping from grid position to vertex indices

#   1   0   0   0   2
#   3   0   0   0   4
#   5   0   6   0   7
#   8   0   9   0  10
#  11   0  12   0  13
#  14   0  15   0  16
#  17  18  19  20  21

## set up the environment
## Define the initial conditions of the robots
robot_ics = [
    ROBOT_AT(2,12), # robot 1 starts at vertex 2
    ROBOT_AT(1,15), # robot 2 starts at vertex 9
]

## Define the manufacturing project
spec = ProjectSpec()
# set initial conditions of "raw materials"
set_initial_condition!(spec,OBJECT_AT(1,15)) # object 1 starts at vertex 4
set_initial_condition!(spec,OBJECT_AT(2,12))  # object 2 starts at vertex 

# define the operations that need to take place

op1 = Operation(Δt=0) 
op2 = Operation(Δt=0)

# inputs
set_precondition!(op1,OBJECT_AT(1,))
set_precondition!(op2,OBJECT_AT(2,12)) 

# outputs
# set_postcondition!(op1,OBJECT_AT(3,14))
# set_postcondition!(op2,OBJECT_AT(4,8))  
add_operation!(spec,op1)
add_operation!(spec,op2)

# define assignment_dict
assignment_dict = Dict(
  1=>[1],2=>[2],
)
# export 
#     apply_assignment_dict!

## define solver
solver = NBSSolver()
# finalize problem construction (the solver is passed as an argument here 
# because it determines what cost model is used for the problem)
prob = pctapf_problem(solver,spec,env,robot_ics)
TaskGraphs.apply_assignment_dict!(get_schedule(get_env(prob)),assignment_dict,get_problem_spec(get_env(prob)))
prob = PC_MAPF(get_env(prob))
route_planner = solver.path_planner
# solve the problem
solution, cost = solve!(route_planner,prob)
println(env.vtx_map)
println(solution)
println(cost)
# check if the problem was solved to optimality
@show feasible_status(solver)
@show optimal_status(solver)