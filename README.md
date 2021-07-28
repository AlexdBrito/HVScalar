# HVScalar: A C++ Dichotomic Search Framework for Solving Biobjective Combinatorial Optimization Problems

This repository contains a C++ framework that implements a dichotomic search algorithm to solve any biobjective combinatorial optimization problem that can be formulated in terms of hypervolume scalarization as described in [[2](#note)]. It is capable of finding the complete set of nondominated points or a subset of it with an approximation guarantee. In the latter case, the size of the subset must be defined by the user. The framework allows to use [SCIP Optimization Suite](https:scipopt.org/#scipoptsuite)(version 7 only) as a MILP solver for each hypervolume scalarization problem, but it can also use a dedicated algorithm, with some more effort. This work is based on Alexandre Brito's Master Thesis [[1](#note)].

## Usage

The framework is currently provided as a single header file. In order to use it, you must copy `hvscalar.h`, include it in the `.cpp` files and follow the implementation instruction that are provided below.

For an implementation example for a biobjective knapsack problem with a cardinality constraint, see the [Example](/Example) folder. An input example and the expected output are also provided. The example can be compiled with:
```
g++ Example/example.cpp Code/hvscalar.h -lscip -std=c++14 -o hvscalar.out
./hvscalar.out < Example/example.in
```

## Overview of Available Functions

The framework currently uses one structure, `HVS`, to store all the necessary data. This structure holds information about all the user implemented functions and basic information about the problem such as the number of nondominated points to find, if set, the objective direction, either maximization or minimization, the initial reference point and the data structure created by the user. It is required as input by all the framework functions.

We consider the following constants and variables

```cpp
#define HVS_Eps     1e-8                 /**< Epsilon value */
#define HVS_Inf     1e+20                /**< Default value considered to be infinity */
#define HVS         struct HVSdata       /**< Main data structure */
#define HVS_Point   std::vector<double>  /**< A point */
#define HVS_Set     std::set<std::shared_ptr<HVS_Point>, Comp2DSetS>  /**< A set of points */
enum HVS_ObjectiveDir {
    HVS_MAX = 1,       /**< sets the objective as maximization */
    HVS_MIN = -1       /**< sets the objective as minimization */
};
enum HVS_SolveType{
    HVS_2D_ALL = 1001,     /**< sets the solving mode to find all nondominated points */
    HVS_2D_NUM,            /**< sets the solving mode to find a subset of all nondominated points */
};
```

The following functions are available:

### HVSstart

```cpp
/** Initiates the HVScalar data structure and sets variables to default values
 * 
 *  Parameters:
 *    HVS - An uninitialized HVS data structure
 */ 
void HVSstart(HVS);
```

### HVSfree

```cpp
/** Frees the HVScalar data structure and all associated data
 * 
 *  Parameters:
 *    HVS - An initialized HVS data structure
 */ 
void HVSfree(HVS);
```

### HVSsetInput

```cpp
/**  Sets the user built function that reads inputs for the problem
 * 
 *  Parameters:
 *    HVS   - An initialized HVS data structure
 *    input - User implemented function that reads the problem input
 */ 
void HVSsetInput(HVS, input);
```

### HVSsetInit

```cpp
/** Sets the user built function that initiates the problem and performs the necessary pre-calculations
 *  
 *  Parameters:
 *    HVS  - An initialized HVS data structure
 *    init - User implemented function that initializes the problem and all the necessary variables
 */ 
void HVSsetInit(HVS, init);
```

### HVSsetSolve

```cpp
/** Sets the user built function that solves an instance of the subproblem
 * 
 *  Parameters:
 *    HVS   - An initialized HVS data structure
 *    solve - User implemented function that solves a version of the problem and returns the best point calculated
 */ 
void HVSsetSolve(HVS, solve);
```

### HVSsetClose

```cpp
/** Sets the user built function that closes the problem and frees all data associated
 *  
 *  Parameters:
 *    HVS   - An initialized HVS data structure
 *    close - User implemented function that frees all variables associated with the problem
 */ 
void HVSsetClose(HVS, close);
```

### HVSsetDataStructure

```cpp
/** Sets the user implemented data structure for the problem
 * 
 *  Parameters:
 *    HVS   - An initialized HVS data structure
 *    data  - User implemented data structure that contains the problems variables
 */ 
void HVSsetDataStructure(HVS, data);
```

### HVSsetIRefPoint

```cpp
/** Sets the initial reference point to be used by the solving process
 * 
 *  Parameters:
 *    HVS        - An initialized HVS data structure
 *    iRefPoint  - A point that is considered as the initial reference point
 */ 
void HVSsetIRefPoint(HVS, iRefPoint);
```

### HVSsetObjectiveDir

```cpp
/** Sets the objective direction of the problem
 * 
 *  Parameters:
 *    HVS              - An initialized HVS data structure
 *    HVS_ObjectiveDir - Value representing the objective of the function, HVS_MAX for maximization and HVS_MIN for minimization,
 */ 
void HVSsetObjectiveDir(HVS, HVS_ObjectiveDir);
```

### HVSsetNumNdPoints

```cpp
/** Sets the number of nondominated points to be calculated
 * 
 *  Parameters:
 *    HVS         - An initialized HVS data structure
 *    numNdPoints - Integer value indicating the number of nondominated points to calculate, only used when HVS_2D_NUM is selected as the solving method
 */ 
void HVSsetNumNdPoints(HVS, numNdPoints);
```

### HVSsetVerbose

```cpp
/** Sets the verbosity mode
 * 
 *  Parameters:
 *    HVS     - An initialized HVS data structure
 *    verbose - A boolean. If set to True, additional information will be displayed during the solving process
 */ 
void HVSsetVerbose(HVS, verbose);
```

### HVSgetIRefPoint

```cpp
/** Obtains the current initial reference point
 * 
 *  Parameters:
 *    HVS - An initialized HVS data structure
 *    
 *  Returns:
 *    HVS_Point - The point set as initial reference point
 */ 
HVS_Point HVSgetIRefPoint(HVS);
```

### HVSgetDataStructure

```cpp
/** Obtains the data structure of the problem.
 * 
 *  Parameters:
 *    HVS   - An initialized HVS data structure
 * 
 *  Returns:
 *    void* - The data structure for the problem
 *  
 *  Note:
 *    As the returned type is void*, it must be cast to the correct data structure type
 */
void* HVSgetDataStructure(HVS);
```

### HVSsolve

```cpp
/** Calculates the nondominated points using the dichotomic scheme 
 * 
 *  Parameters:
 *    HVS           - An initialized HVS data structure
 *    HVS_SolveType - Solving method for the problem
 */ 
void HVSsolve(HVS, HVS_SolveType);
```

### HVSgetSols

```cpp
/** Obtains the set of solutions of the dichotomic scheme
 * 
 *  Parameters:
 *    HVS - An initialized HVS data structure
 * 
 *  Returns:
 *    HVS_Set - The set of nondominated points found
 */ 
HVS_Set HVSgetSols(HVS);
```

### HVSprintSet

```cpp
/** Prints the given set of points
 * 
 *  Parameters:
 *    HVS_Set -  The HVS_Set to be displayed
 */ 
void HVSprintSet(HVS_Set);
```

## Implementing steps

1. Implement functions input, init, solve and close. These functions are responsible for reading all the necessary inputs, initializing the problem, solving an instance of the problem and freeing all the resources allocated to the problem, respectively.

2. If additional variables are needed in order to solve a subproblem, excluding the reference point, create a data structure that holds all the required variables associated to the implemented problem.

3. On the main() function, declare the main data structure HVS and initialize it by calling HVSstart(HVS).

4. Set all the user implemented functions on the framework by calling, in any order, HVSsetInput(HVS, input), HVSsetInit(HVS, init), HVSsetSolve(HVS, solve), HVSsetClose(HVS, close), and HVSsetDataStructure(HVS, data).

5. In order to set the initial reference point, the function HVSsetIRefPoint(HVS, HVSPoint) should be called. The objective of the problem should also be set using function HVSsetObjectiveDir(HVS, HVS_ObjectiveDir).

6. If the user wants to calculate a specific number of nondominated points, they must call function HVSsetNumNdPoints(HVS, numNdPnts), with numNdPnts being an integer of how many points to calculate.

7. With all necessary variables set, function HVSsolve(HVS, HVS_SolveType) should be called. This will run the method chosen.

8. The results can be Obtainsed by using function HVSgetSols(HVS), which returns a HVS_Set, and displayed using function HVSprintSet(HVS_Set).

9. Lastly, the function HVSfree(HVS) must be called to release all the frameworks variables.

<p align="center">
  <img src="Docs/FrameworkFlowDiagram.png">
</p>

A [`pdf` version of the user manual](/Docs/UserManual.pdf) is also available.


## Note

If you plan to write a scientific paper describing research that made use of this program, I kindly ask you to mention that this software was used. The appropriate citation is:

[1] Alexandre Brito, [An hypervolume dichotomic scheme for multiobjective optimization](/Docs/Thesis_Alexandre_Brito_2021.pdf), MSc in Informatics Engineering, University of Coimbra, Portugal, 2021.

as well as

[2] Luís Paquete, Britta Schulze, Michael Stiglmayr, and Ana C. Lourenço, Computing representations with hypervolume scalarizations, Computers & Operations Research, 2021
([DOI](http://doi.org/10.1016/j.cor.2021.105349)).

I would also appreciate it if you would email adbrito@student.dei.uc.pt and paquete@dei.uc.pt with citations of papers referencing this work.

## Acknowledgements

This project was developed under a Research Initiation Grant, which was financed by national funds through FCT – Foundation for Science and Technology, I.P., in the scope of project CISUC – UID/CEC/00326/2020.
