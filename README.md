# Parallel Optimal Control via ADMM
## Abstract
This work proposes a parallel optimization algorithm for cooperative automation of large-scale connected vehicles. The task of cooperative automation is formulated as a centralized optimization problem taking the whole decision space of all vehicles into account. Considering the uncertainty of the environment, the problem is solved in a receding horizon fashion. Then, we employ the alternating direction method of multipliers (ADMM) to solve the centralized optimization in a parallel way, which scales more favorably to large-scale instances. Also, Taylor series is used to linearize nonconvex constraints caused by coupling collision avoidance constraints among interactive vehicles. Simulations with two typical traffic scenes for multiple vehicles demonstrate the effectiveness and efficiency of our method.
The details of this work could be referenced to my paper [Parallel Optimal Control for Cooperative Automation of Large-scale Connected Vehicles via ADMM](https://arxiv.org/pdf/1807.11874.pdf, "my article")
## Demo
Three scenarios are simulated
### cooperative overtaking
![overtake](https://github.com/arkria/Parallel-Optimal-Control-via-ADMM/blob/master/figure/overtake.gif)
### cooperative driving at intersection
![intersection](https://github.com/arkria/Parallel-Optimal-Control-via-ADMM/blob/master/figure/intersection.gif)
### multiple vehicles
![multiple vehicles](https://github.com/arkria/Parallel-Optimal-Control-via-ADMM/blob/master/figure/multiple.gif)
