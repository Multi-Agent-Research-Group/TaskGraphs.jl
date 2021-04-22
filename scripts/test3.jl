using TaskGraphs


## set up the environment
using GraphUtils

indicator_grid = IndicatorGrid(zeros(Bool,11,5))                           # initialize a 10x10 environment with no obstacles
# indicator_grid[4,4] = true                                                                # obstacle at position 4,4
indicator_grid[1:9,2] .= true
indicator_grid[1:9,4] .= true                                                          # obstacle covering positions 6:7,6:7
env = construct_factory_env_from_indicator_grid(indicator_grid) # initialize the factory environment
@show env.vtx_map                                                                     # visualize the mapping from grid position to vertex indices
# exit()
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
    ROBOT_AT(1,27), # robot 1 starts at vertex 2
    ROBOT_AT(2,36), # robot 2 starts at vertex 9
    ROBOT_AT(3,29),
]

## Define the manufacturing project
spec = ProjectSpec()
# set initial conditions of "raw materials"
set_initial_condition!(spec,OBJECT_AT(1,27)) 
set_initial_condition!(spec,OBJECT_AT(2,36)) 
set_initial_condition!(spec,OBJECT_AT(3,29)) 

# set_initial_condition!(spec,OBJECT_AT(5,7))
# set_initial_condition!(spec,OBJECT_AT(6,2))
# set_initial_condition!(spec,OBJECT_AT(7,12)) 

# define the operations that need to take place
# define the operations that need to take place

op1 = Operation(Δt=0) 
op2 = Operation(Δt=0)
op3 = Operation(Δt=0)

# inputs
set_precondition!(op1,OBJECT_AT(1,26))
set_precondition!(op2,OBJECT_AT(2,20))
set_precondition!(op3,OBJECT_AT(3,14)) 

# outputs
set_postcondition!(op1,OBJECT_AT(4,23))
set_postcondition!(op2,OBJECT_AT(5,17))
set_postcondition!(op3,OBJECT_AT(6,11))

add_operation!(spec,op1)
add_operation!(spec,op2)
add_operation!(spec,op3)

# add a terminal operation with no outputs
op4 = Operation(Δt=0) 
op5 = Operation(Δt=0)
op6 = Operation(Δt=0)

# outputs
set_precondition!(op4,OBJECT_AT(4,2))
set_precondition!(op5,OBJECT_AT(5,5))
set_precondition!(op6,OBJECT_AT(6,8))  

add_operation!(spec,op4)
add_operation!(spec,op5)
add_operation!(spec,op6)

## define solver
solver = NBSSolver()
# finalize problem construction (the solver is passed as an argument here 
# because it determines what cost model is used for the problem)
prob = pctapf_problem(solver,spec,env,robot_ics)
# solve the problem
solution, cost = solve!(solver,prob)
println(env.vtx_map)
println(solution)
println(cost)
# check if the problem was solved to optimality
@show feasible_status(solver)
@show optimal_status(solver)