# TSP-CPP
Matlab code for the two methods introduced in **J. Xie, L. R. G. Carrilo, L. Jin, "Path Planning for UAV to Cover Multiple  Separated Convex Polygonal Regions", IEEE Access, Vol. 8, pp. 51770-51785, 2020.** for solving the TSP-CPP problem, the integrated traveling salesman problem (TSP) and coverage path planning (CPP) problem. 

The TSP-CPP problem aims to find the shortest path for covering multiple non-overlapping regions. 

The first method is based on dynamic programming, which can find (near) optimal solutions. The second method is a heuristic method that can solve large-scale TSP-CPP problems efficiently. 

## Instruction 
To run the dynamic programming based method, open the "DP_TSPCPP" folder, and then run the `main_DP_TSPCPP.m` file. 

To run the heuristic method (Fast NN-2Opt), open the "Fast NN-2Opt_TSPCPP" folder, and then run the `main_FastNNOpt_TSPCPP.m` file.

## Paper citation
Please cite the following paper if you used the code or any of the TSP-CPP methods. 
```
@article{xie2020path,
  title={Path planning for uav to cover multiple separated convex polygonal regions},
  author={Xie, Junfei and Carrillo, Luis Rodolfo Garcia and Jin, Lei},
  journal={IEEE Access},
  volume={8},
  pages={51770--51785},
  year={2020},
  publisher={IEEE}
}
```
