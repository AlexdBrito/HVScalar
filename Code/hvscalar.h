#include <bits/stdc++.h>
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "scip/struct_misc.h"

#ifndef HVScalar
#define HVScalar

#define HVS_X            0                                                     /**< key for the value of a point for the first objective */
#define HVS_Y            1                                                     /**< key for the value of a point for the second objective */
#define HVS_HVC          2                                                     /**< key for the hypervolume contribution on the set S */
#define HVS_Xref         3                                                     /**< key for the value of a point for the first objective of the reference point */
#define HVS_Yref         4                                                     /**< key for the value of a point for the second objective of the reference point */
#define HVS_Eps          1e-8                                                  /**< Epsilon value */
#define HVS_Inf          1e+20                                                 /**< default value considered to be infinity */
#define HVS              struct HVSdata                                     /**< Framework main data structure */
#define HVS_Point        std::vector<double>                                /**< A point */
#define HVS_Set          std::set<std::shared_ptr<HVS_Point>, Comp2DSetS>      /**< A set of points */
#define HVS_Solve        hvs->solve
#define HVS_Verbose      hvs->verbose

enum HVS_ObjectiveDir {
    HVS_MAX = 1,           /**< value that sets the problems objective as maximization */
    HVS_MIN = -1             /**< value that sets the problems objective as minimization */
};
enum HVS_SolveType{
    HVS_2D_ALL = 1001,     /**< value for solving a problem with 2 dimensions */
    HVS_2D_NUM,              /**< value for solving a problem with 2 dimensions considering a maximum number of points to find */
};

struct Comp2DSetS {
    bool operator()(const std::shared_ptr<HVS_Point> &lhs, const std::shared_ptr<HVS_Point> &rhs) const {
        return (*lhs)[0] < (*rhs)[0];
    }
};

struct Comp2DSetP {
    bool operator()(HVS_Point &lhs, HVS_Point &rhs) const {
        return lhs[2] < rhs[2];
    }
};

struct HVSdata{
    int numSols;                                          /**< Number of solutions to get on main while loop */
    bool verbose;                                         /**< Print additional information during solveing process */
    HVS_ObjectiveDir objectiveDir;                         /**< Objective direction for the user problem*/
    HVS_Point iRefPoint;                                  /**< Initial reference point */
    HVS_Set result;                                       /**< Set containing the result of the dichotomic scheme*/
    void (*input) (HVS *hvs);                             /**< Function that reads the necessary input data for the problem */
    SCIP_RETCODE (*init) (HVS *hvs);                      /**< Function that initializes the problem structure and makes necessary precalculations */
    HVS_Point (*solve) (HVS *hvs, HVS_Point refPoint);    /**< Function that solves an instance of a subproblem, returns a new point found */
    SCIP_RETCODE (*close) (HVS *hvs);                     /**< Function that frees and closes the problem structure */
    void *problemVariables;                               /**< Data structure for the problem */
};

/** ======================  Framework Functions  ====================== **/


/** Initiates the framework data structure and sets variables to default values
 *
 */
void HVSstart(
        HVS       **hvs      /**< Default framework data structure */
){

    *hvs = new HVSdata();

    (*hvs)->numSols = INT_MAX;
    (*hvs)->verbose = false;
    (*hvs)->objectiveDir = HVS_MAX;
}

/** Closes the framework data structure, freeing all data
 *
 */
void HVSfree(
        HVS       **hvs      /**< Default framework data structure */
){
    delete *hvs;
}

/** Sets the user built function that reads inputs for the problem
 *
 */
void HVSsetInput(
        HVS       *hvs,      /**< Default framework data structure */
        void (*input) (HVS *hvs)
){
    (hvs)->input = input;
}

/** Sets the user built function that initiates the problem and perform necessary pre-calculations
 *
 */
void HVSsetInit(
        HVS       *hvs,      /**< Default framework data structure */
        SCIP_RETCODE (*init) (HVS *hvs)
){
    (hvs)->init = init;
}

/** Sets the user built function that solves an instance of a sub-problem
 *
 */
void HVSsetSolve(
        HVS       *hvs,      /**< Default framework data structure */
        HVS_Point (*solve) (HVS *hvs, HVS_Point refPoint)
){
    (hvs)->solve = solve;
}

/** Sets the user built function that closes the original problem and frees all data associated
 *
 */
void HVSsetClose(
        HVS       *hvs,      /**< Default framework data structure */
        SCIP_RETCODE (*close) (HVS *hvs)
){
    (hvs)->close = close;
}

/** Sets the objective direction of the problem
 *
 */
void HVSsetObjectiveDir(
        HVS                  *hvs,         /**< Default framework data structure */
        HVS_ObjectiveDir     objectiveDir  /**< Objective direction for the problem */
){
    (hvs)->objectiveDir = objectiveDir;
}

/** Sets the verbosity mode 
 *
 *  @note Verbose mode outputs additional information during the solving process such as the points obtained from
 *        the subproblem, its hypervolume contribution and if the points was inserted in the list of points to use
 *        to create new reference points
 *
 */
void HVSsetVerbose(
        HVS       *hvs,      /**< Default framework data structure */
        bool      verbose    /**< Boolean choice for the verbosity mode */
){
    (hvs)->verbose = verbose;
}

/** Sets the number of nondominated points to be calculated
 *
 */
void HVSsetNumNdPoints(
        HVS       *hvs,          /**< Default framework data structure */
        int       numNdPoints    /**< Number of nondominated points to calculate */
){
    (hvs)->numSols = numNdPoints;
}

/** Sets the initial reference point to be used by the solving process
 *
 *
 */
void HVSsetIRefPoint(
        HVS         *hvs,        /**< Default framework data structure */
        HVS_Point   iRefPoint    /**< Values of the point for all objective */
){
    hvs->iRefPoint = std::move(iRefPoint);
}

/** Obtain set initial reference point
 *
 * @return the initial reference point for the problem
 */
HVS_Point HVSgetIRefPoint(
        HVS       *hvs       /**< Default framework data structure */
){
    return hvs->iRefPoint;
}

/** Obtain the set of solutions of the dichotomic scheme
 *
 * @return the initial reference point for the problem
 */
HVS_Set HVSgetSols(
        HVS       *hvs       /**< Default framework data structure */
){
    return hvs->result;
}

/** Sets the user implemented data structure for the problem
 *
 */
void HVSsetDataStructure(
        HVS       *hvs,              /**< Default framework data structure */
        void*     problemVariables   /**< Data structure for the problem */
){
    hvs->problemVariables = problemVariables;
}

/** Obtain the data structure of the problem.
 *  As the returned type is void*, it must be cast to the correct data structure type
 *
 * @return the data structure for the problem
 */
void* HVSgetDataStructure(
        HVS       *hvs       /**< Default framework data structure */
){
    return hvs->problemVariables;
}

/** Prints the calculated set of points
 *
 */
void HVSprintSet(
        HVS_Set       result        /**< Set containing the optimal points obtained */
){
    for (const auto& p : result) {
        if(std::none_of((*p).begin(), (*p).end(), [](auto coord){return coord == HVS_Inf;})) {
            for (auto c : *p) {
                std::cout << c << " ";
            }
            std::cout << "\n";
        }
    }
}

/** Default message handler, outputs the message given to the corresponding filename
 *
 */
void HVSmessageHandler(
        const std::string&     fileName,   /**< Name of the file to send the message to */
        const std::string&     message     /**< Message to be printed */
){
    std::ofstream file;
    file.open(fileName, std::ios::app | std::ios::out);
    file << message << "\n";
    file.close();
}



/** ======================  2 Dimensions Functions  ====================== **/


/** calculates the hypervolume contribution of a point given a set
 *
 *  @return A non negative real number that indicates the hypervolume contribution of the point using a set of
 *          points as reference
 */
double HVScalcHvContribution(
        HVS_Point          pointP,   /**< Point to calculate the hypervolume contribution */
        HVS_Set            &setS     /**< Set to use as reference */
){

    for(const auto& setSPoint : setS) {
        if(abs(pointP[0] - (*setSPoint)[0]) < HVS_Eps and
           abs(pointP[1] - (*setSPoint)[1]) < HVS_Eps)
            return 0;
    }
    return std::abs((pointP[HVS_X] - pointP[HVS_Xref]) * (pointP[HVS_Y] - pointP[HVS_Yref]));
}

/** Uses a hypervolume dichotomic scheme to calculate a specific number of optimal points
 *
 *  @note The number of points to calculate must be set prior to calling this method.
 *        Use \ref HVSsetNumNdPoints to set the number of points to calculate.
 *
 *  @return \ref SCIP_OKAY is returned if everything worked. Otherwise a suitable error code is passed. See \ref
 *          SCIP_Retcode "SCIP_RETCODE" for a complete list of error codes.
 */
void HVSsolveAll2D(
        HVS       *hvs       /**< Default framework data structure */
){

    HVS_Set setS;
    std::priority_queue<HVS_Point, std::vector<HVS_Point>, Comp2DSetP> setP;
    HVS_Point refPoint = hvs->iRefPoint;
    HVS_Point pointS;
    HVS_Point pointP;
    HVS_Point succ;
    HVS_Point pred;

    setS.insert(std::shared_ptr<HVS_Point> (new HVS_Point({refPoint[HVS_X], HVS_Inf})));
    setS.insert(std::shared_ptr<HVS_Point> (new HVS_Point({HVS_Inf, refPoint[HVS_Y]})));

    pointS = HVS_Solve(hvs, refPoint);
    auto aux = std::shared_ptr<HVS_Point> (new HVS_Point({pointS[HVS_X], pointS[HVS_Y]}));
    setS.insert(aux);

    while(true) {
        succ = *(*(std::next(setS.find(aux))));
        refPoint[HVS_X] = pointS[HVS_X];
        refPoint[HVS_Y] = succ[HVS_Y];
        pointP = HVS_Solve(hvs, refPoint);
        pointP[HVS_HVC] = HVScalcHvContribution(pointP, setS);
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]) + " - " + std::to_string(pointP[HVS_HVC]));
        }
        if(pointP[HVS_HVC] > 0){
            setP.push(pointP);
            if(HVS_Verbose){
                HVSmessageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]));
            }
        }
        pred = *(*(std::prev(setS.find(aux))));
        refPoint[HVS_X] = pred[HVS_X];
        refPoint[HVS_Y] = pointS[HVS_Y];
        pointP = HVS_Solve(hvs, refPoint);
        pointP[HVS_HVC] = HVScalcHvContribution(pointP, setS);
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]) + " - " + std::to_string(pointP[HVS_HVC]));
        }
        if(pointP[HVS_HVC] > 0){
            setP.push(pointP);
            if(HVS_Verbose){
                HVSmessageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]));
            }
        }
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
        }
        if (setP.empty())
            break;
        pointS = setP.top();
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Dequeued " + std::to_string(pointS[HVS_X]) + " " + std::to_string(pointS[HVS_Y]) + " - " + std::to_string(pointS[HVS_HVC]));
        }
        setP.pop();
        aux = std::shared_ptr<HVS_Point> (new HVS_Point({pointS[HVS_X], pointS[HVS_Y]}));
        setS.insert(aux);
    };
    hvs->result = setS;
}

/** Uses a hypervolume dichotomic scheme to calculate a specific number of optimal points
 *
 *  @note The number of points to calculate must be set prior to calling this method.
 *        Use \ref HVSsetNumNdPoints to set the number of points to calculate.
 *
 *  @return \ref SCIP_OKAY is returned if everything worked. Otherwise a suitable error code is passed. See \ref
 *          SCIP_Retcode "SCIP_RETCODE" for a complete list of error codes.
 */
void HVSsolveNum2D(
        HVS       *hvs       /**< Default framework data structure */
){

    HVS_Set setS;
    std::priority_queue<HVS_Point, std::vector<HVS_Point>, Comp2DSetP> setP;
    HVS_Point refPoint = hvs->iRefPoint;
    HVS_Point pointS;
    HVS_Point pointP;
    HVS_Point succ;
    HVS_Point pred;

    setS.insert(std::shared_ptr<HVS_Point> (new HVS_Point({refPoint[HVS_X], HVS_Inf})));
    setS.insert(std::shared_ptr<HVS_Point> (new HVS_Point({HVS_Inf, refPoint[HVS_Y]})));

    pointS = HVS_Solve(hvs, refPoint);
    auto aux = std::shared_ptr<HVS_Point> (new HVS_Point({pointS[HVS_X], pointS[HVS_Y]}));
    setS.insert(aux);

    while(setS.size() - 2 < hvs->numSols) {
        succ = *(*(std::next(setS.find(aux))));
        refPoint[HVS_X] = pointS[HVS_X];
        refPoint[HVS_Y] = succ[HVS_Y];
        pointP = HVS_Solve(hvs, refPoint);
        pointP[HVS_HVC] = HVScalcHvContribution(pointP, setS);
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]) + " - " + std::to_string(pointP[HVS_HVC]));
        }
        if(pointP[HVS_HVC] > 0){
            setP.push(pointP);
            if(HVS_Verbose){
                HVSmessageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]));
            }
        }
        pred = *(*(std::prev(setS.find(aux))));
        refPoint[HVS_X] = pred[HVS_X];
        refPoint[HVS_Y] = pointS[HVS_Y];
        pointP = HVS_Solve(hvs, refPoint);
        pointP[HVS_HVC] = HVScalcHvContribution(pointP, setS);
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]) + " - " + std::to_string(pointP[HVS_HVC]));
        }
        if(pointP[HVS_HVC] > 0){
            setP.push(pointP);
            if(HVS_Verbose){
                HVSmessageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[HVS_X]) + " " + std::to_string(pointP[HVS_Y]));
            }
        }
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
        }
        if (setP.empty())
            break;
        pointS = setP.top();
        if(HVS_Verbose){
            HVSmessageHandler("Output/framework.txt", "Dequeued " + std::to_string(pointS[HVS_X]) + " " + std::to_string(pointS[HVS_Y]) + " - " + std::to_string(pointS[HVS_HVC]));
        }
        setP.pop();
        aux = std::shared_ptr<HVS_Point> (new HVS_Point({pointS[HVS_X], pointS[HVS_Y]}));
        setS.insert(aux);
    };
    hvs->result = setS;
}

/** Calculates the nondominated points using the dichotomic scheme 
 *
 *  @note There are 2 solving types, \ref HVS_ALL that calculates all optimal points
 *         and \ref HVS_NUM that calculates a set number of points.
 */
void HVSsolve(
        HVS      *hvs,       /**< Default framework data structure */
        int      solvingType /**< Solving method for the problem */        
){
    if(hvs->input != nullptr){
        hvs->input(hvs);
    }
    if(hvs->input != nullptr) {
        hvs->init(hvs);
    }

    if(HVS_Verbose){
        std::ofstream file;
        file.open("Output/framework.txt");
        file.close();
    }

    switch(solvingType) {
        case HVS_2D_ALL: {
            HVSsolveAll2D(hvs);
            break;
        }
        case HVS_2D_NUM: {
            HVSsolveNum2D(hvs);
            break;
        }
        default:
            break;
    }
    if(hvs->input != nullptr) {
        hvs->close(hvs);
    }
}
#endif
