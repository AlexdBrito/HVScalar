#include <bits/stdc++.h>
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "scip/struct_misc.h"
#include <memory>
#include <thread>
#include <future>
#include <utility>
#include "nondlib.hpp"

#ifndef FRAMEWORK
#define FRAMEWORK

#define FW_X            0                                                   /**< key for the value of a point for the first objective */
#define FW_Y            1                                                   /**< key for the value of a point for the second objective */
#define FW_HVC          2                                                   /**< key for the hypervolume contribution on the set S */
#define FW_Xref         3                                                   /**< key for the value of a point for the first objective of the reference point */
#define FW_Yref         4                                                   /**< key for the value of a point for the second objective of the reference point */
#define FW_EPS          1e-8                                                /**< Epsilon value */
#define FW_INF          1e+20                                               /**< default value considered to be infinity */
#define FW_STRUCT       struct dataStruct                                   /**< Framework main data structure */
#define FW_TIMENOW      std::chrono::high_resolution_clock::now()           /**< Returns the current clock time */
#define FW_MSEC         std::chrono::milliseconds                           /**< Time in milliseconds */
#define FW_TIMELEFT(a)  std::chrono::duration_cast<FW_MSEC>(a - FW_TIMENOW) /**< Returns the time left, in milliseconds, until a given time limit */
#define FW_SET          std::set<std::shared_ptr<FW_POINT>, Comp2DSetS>     /**< A set of points */
#define FW_SOLVE        data->solve
#define FW_VERBOSE      data->verbose
#define FW_POINT        std::vector<SCIP_Real>

enum {FW_MAX = 1,           /**< value that sets the problems objective as maximization */
    FW_MIN = -1             /**< value that sets the problems objective as minimization */
};
enum {FW_2D = 1001,         /**< value for solving a problem with 2 dimensions */
    FW_2D_PARALLEL,         /**< value for solving a problem with 2 dimensions using parallelization */
    FW_2D_TIME,             /**< value for solving a problem with 2 dimensions considering a time limit*/
    FW_2D_NUM,              /**< value for solving a problem with 2 dimensions considering a maximum number of points to find */
    FW_ND,                  /**< value for solving a problem with 3 dimensions */
    FW_ND_PARALLEL          /**< value for solving a problem with 3 dimensions using parallelization*/
};

struct dataStruct{
    int numSols;                                                              /**< Number of solutions to get on main while loop */
    bool verbose;                                                             /**< Print additional information during solveing process */
    bool printProblem;                                                        /**< Print each subproblem and their solution to a file */
    FW_MSEC timeLimit;                                                        /**< Time limit for problem solving in milliseconds, applicable when /ref FW_TIME is selected */
    std::mutex m;                                                             /**< **/
    void *problemVariables;                                                   /**< Data structure for the problem */
    FW_POINT iRefPoint;                                                       /**< Initial reference point */
    void (*input) (FW_STRUCT *data);                                          /**< Function that reads the necessary input data for the problem */
    SCIP_RETCODE (*init) (FW_STRUCT *data);                                   /**< Function that initializes the problem structure and makes necessary precalculations */
    FW_POINT (*solve) (FW_STRUCT *data, FW_POINT refPoint);                   /**< Function that solves an instance of a subproblem, returns a new point found */
    SCIP_RETCODE (*close) (FW_STRUCT *data);                                  /**< Function that frees and closes the problem structure */
    int numberDimensions;
    int objective;
};

struct Comp2DSetS {
    bool operator()(const std::shared_ptr<FW_POINT> &lhs, const std::shared_ptr<FW_POINT> &rhs) const {
        return (*lhs)[0] < (*rhs)[0];
    }
};

struct Comp2DSetP {
    bool operator()(FW_POINT &lhs, FW_POINT &rhs) const {
        return lhs[2] < rhs[2];
    }
};

class ReferencePoint {
private:
    FW_POINT refPoint;
    std::vector<std::shared_ptr<FW_POINT>> associatedPoints;

public:
    ReferencePoint() = default;

    explicit ReferencePoint(FW_POINT rPoint) {
        refPoint = std::move(rPoint);
        for(int i = 0; i < refPoint.size(); i++)
            associatedPoints.emplace_back();
    }

    FW_POINT const& getPoint() const{
        return refPoint;
    }

    void setPoint(FW_POINT point) {
        refPoint = std::move(point);
    }

    void updateCoord(int index, SCIP_Real newCoord) {
        refPoint[index] = newCoord;
    }

    std::vector<std::shared_ptr<FW_POINT>> &getAssociatedPoints() {
        return associatedPoints;
    }

    void setAssociatedPoints(std::vector<std::shared_ptr<FW_POINT>> assPoints) {
        associatedPoints = std::move(assPoints);
    }

    bool isDominatedByIgnoringIndex(FW_POINT point, int objIndex) {
        for(int i = 0; i < associatedPoints.size(); i++) {
            if(i == objIndex or (associatedPoints[i]).get() == nullptr)
                continue;
            if((*((associatedPoints[i]).get()))[objIndex] < point[objIndex]){
                return false;
            }
        }
        return true;
    }

    bool operator==( const ReferencePoint &rp ) const { return getPoint() == rp.getPoint(); }
    SCIP_Real& operator[](std::size_t idx) { return refPoint[idx]; }
    const SCIP_Real& operator[](std::size_t idx) const { return refPoint[idx]; }

};

template <typename F, typename... Ts>
inline auto asyncSolve(F&& f, Ts&&... params) {
    return std::async(std::launch::async, std::forward<F>(f),
                      std::forward<Ts>(params)...);
}

#if _WIN32||_WIN64
#define VOIDOUTPUT ""
#elif __linux__
#define VOIDOUTPUT "/dev/null"
#define NORMALOUTPUT "/dev/tty"
#endif /*(choice of OS) */


/** ======================  Framework Functions  ====================== **/


/** Set the maximum amount of time for SCIP to solve a subproblem
 *
 *  @note This needs to be called every time a subproblem is going to be solved when using SCIP
 *
 *  @return \ref SCIP_OKAY is returned if everything worked. Otherwise a suitable error code is passed. See \ref
 *          SCIP_Retcode "SCIP_RETCODE" for a complete list of error codes.
 */
SCIP_RETCODE setSCIPtimeout(
        SCIP            *scip,      /**< Default SCIP structure */
        SCIP_Real       timeLimit   /**< Time limit for the solving process */
){
    SCIPsetRealParam(scip, "limits/time", timeLimit);

    return SCIP_OKAY;
}


/** Initiates the framework data structure and sets variables to default values
 *
 */
void startFramework(
        FW_STRUCT       **data      /**< Default framework data structure */
){

    *data = new dataStruct();

    (*data)->numSols = INT_MAX;
    (*data)->verbose = false;
    (*data)->printProblem = false;
    (*data)->timeLimit = FW_MSEC(INT_MAX);
}


/** Closes the framework data structure, freeing all data
 *
 */
void closeFramework(
        FW_STRUCT       **data      /**< Default framework data structure */
){
    delete *data;
}


/** Sets the user built function that reads inputs for the problem
 *
 */
void setInputProblem(
        FW_STRUCT       *data,      /**< Default framework data structure */
        void (*input) (FW_STRUCT *data)
){
    (data)->input = input;
}


/** Sets the user built function that initiates the problem and perform necessary pre-calculations
 *
 */
void setInitProblem(
        FW_STRUCT       *data,      /**< Default framework data structure */
        SCIP_RETCODE (*init) (FW_STRUCT *data)
){
    (data)->init = init;
}


/** Sets the user built function that solves an instance of a sub-problem
 *
 */
void setSolveProblem(
        FW_STRUCT       *data,      /**< Default framework data structure */
        FW_POINT (*solve) (FW_STRUCT *data, FW_POINT refPoint)
){
    (data)->solve = solve;
}


/** Sets the user built function that closes the original problem and frees all data associated
 *
 */
void setCloseProblem(
        FW_STRUCT       *data,      /**< Default framework data structure */
        SCIP_RETCODE (*close) (FW_STRUCT *data)
){
    (data)->close = close;
}


/**
 *
 */
void setProblemObjective(
        FW_STRUCT       *data,      /**< Default framework data structure */
        int             objective
){
    (data)->objective = objective;
}


/** Sets verbose mode according to the choice given
 *
 *  @note Verbose mode outputs additional information during the solving process such as the points obtained from
 *        the subproblem, its hypervolume contribution and if the points was inserted in the list of points to use
 *        to create new reference points
 *
 */
void setVerbose(
        FW_STRUCT       *data,      /**< Default framework data structure */
        bool            choice      /**< Boolean choice for the verbose mode */
){
    (data)->verbose = choice;
}


/** Sets the number of optimal points to calculate when using \ref hvDichotomicSchemeNum()
 *
 */
void setNumSols(
        FW_STRUCT       *data,      /**< Default framework data structure */
        int             numSols     /**< Number of optimal points to calculate */
){
    (data)->numSols = numSols;
}


/** Sets the time limit for the entire solving process when using \ref hvDichotomicSchemeTime()
 *
 *  @note This time limit does not take into consideration the input and pre-calculation phase,
 *        nor the closing of the problems and output of the obtained results.
 *
 */
void setTimeLimit(
        FW_STRUCT       *data,      /**< Default framework data structure */
        double          seconds     /**< Time limit in seconds for the solving process */
){
    (data)->timeLimit = FW_MSEC((int)seconds*1000);
}


/** Sets the printing of sub-problems mode depending on the choice given
 *
 *  @note This mode sends the output given during the solving process of SCIP (or any other
 *        output created by the user in \ref solve()) and sends it to files. If set to false all
 *        outputs are considered voided
 *
 */
void setPrintProblem(
        FW_STRUCT       *data,      /**< Default framework data structure */
        bool            choice      /**< Boolean choice for the problem printing mode */
){
    (data)->printProblem = choice;
}


/** Sets the initial reference point to use for the solving process for 2D problems
 *
 *
 */
void setInitialRefPoint(
        FW_STRUCT       *data,      /**< Default framework data structure */
        FW_POINT        coords      /**< Values of the point for all objective */
){
    data->iRefPoint = std::move(coords);
}


/** Obtain set initial reference point
 *
 * @return the initial reference point for the problem
 */
FW_POINT getInitialRefPoint(
        FW_STRUCT       *data       /**< Default framework data structure */
){
    return data->iRefPoint;
}


/** Default message handler, outputs the message given to the corresponding filename
 *
 */
void messageHandler(
        const std::string&     fileName,   /**< Name of the file to send the message to */
        const std::string&     message     /**< Message to be printed */
){
    std::ofstream file;
    file.open(fileName, std::ios::app | std::ios::out);
    file << message << "\n";
    file.close();
}


/** Prints the calculated set of points
 *
 */
void printResult(
        FW_SET       result        /**< Set containing the optimal points obtained */
){
    for (const auto& p : result) {
        if(std::none_of((*p).begin(), (*p).end(), [](auto coord){return coord == FW_INF;})) {
            for (auto c : *p) {
                std::cout << c << " ";
            }
            std::cout << "\n";
        }
    }
}


/** Sets the data structure for the problem
 *
 */
void setProblemDS(
        FW_STRUCT       *data,              /**< Default framework data structure */
        void*           problemVariables    /**< Data structure for the problem */
){
    data->problemVariables = problemVariables;
}


/** Obtain the data structure for the problem
 *
 * @return the data structure for the problem
 */
void* getProblemDS(
        FW_STRUCT       *data       /**< Default framework data structure */
){
    return data->problemVariables;
}


/** Lock the mutex for parallelization
 *
 * @return the built-in mutex
 */
void FW_MutexLock(
        FW_STRUCT       *data       /**< Default framework data structure */
){
    (data)->m.lock();
}


/** Unlock the mutex for parallelization
 *
 * @return the built-in mutex
 */
void FW_MutexUnlock(
        FW_STRUCT       *data       /**< Default framework data structure */
){
    (data)->m.unlock();
}

template <typename T>
void printPoints(T const& points) {
    for (auto const& p : points) {
        for (auto c : *p) {
            std::cout << c << " ";
        }
        std::cout << "\n";
    }
}


/** ======================  2 Dimensions Functions  ====================== **/


/** calculates the hypervolume contribution of a point given a set
 *
 *  @return A non negative real number that indicates the hypervolume contribution of the point using a set of
 *          points as reference
 */
double calcHvContribution(
        FW_POINT          pointP,   /**< Point to calculate the hypervolume contribution */
        FW_SET            &setS     /**< Set to use as reference */
){

    for(const auto& setSPoint : setS) {
        if(abs(pointP[0] - (*setSPoint)[0]) < FW_EPS and
           abs(pointP[1] - (*setSPoint)[1]) < FW_EPS)
            return 0;
    }
    return std::abs((pointP[FW_X] - pointP[FW_Xref]) * (pointP[FW_Y] - pointP[FW_Yref]));
}


/** Uses a hypervolume dichotomic scheme to calculate a specific number of optimal points
 *
 *  @note The number of points to calculate must be set prior to calling this method.
 *        Use \ref setNumSols to set the number of points to calculate.
 *
 *  @return \ref SCIP_OKAY is returned if everything worked. Otherwise a suitable error code is passed. See \ref
 *          SCIP_Retcode "SCIP_RETCODE" for a complete list of error codes.
 */
FW_SET hvDichotomicSchemeAll2D(
        FW_STRUCT       *data       /**< Default framework data structure */
){

    FW_SET setS;
    std::priority_queue<FW_POINT, std::vector<FW_POINT>, Comp2DSetP> setP;
    FW_POINT refPoint = data->iRefPoint;
    FW_POINT pointS;
    FW_POINT pointP;
    FW_POINT succ;
    FW_POINT pred;

    setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({refPoint[FW_X], FW_INF})));
    setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({FW_INF, refPoint[FW_Y]})));

    pointS = FW_SOLVE(data, refPoint);
    auto aux = std::shared_ptr<FW_POINT> (new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
    setS.insert(aux);

    while(true) {
        succ = *(*(std::next(setS.find(aux))));
        refPoint[FW_X] = pointS[FW_X];
        refPoint[FW_Y] = succ[FW_Y];
        pointP = FW_SOLVE(data, refPoint);
        pointP[FW_HVC] = calcHvContribution(pointP, setS);
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " + std::to_string(pointP[FW_HVC]));
        }
        if(pointP[FW_HVC] > 0){
            setP.push(pointP);
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
            }
        }
        pred = *(*(std::prev(setS.find(aux))));
        refPoint[FW_X] = pred[FW_X];
        refPoint[FW_Y] = pointS[FW_Y];
        pointP = FW_SOLVE(data, refPoint);
        pointP[FW_HVC] = calcHvContribution(pointP, setS);
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " + std::to_string(pointP[FW_HVC]));
        }
        if(pointP[FW_HVC] > 0){
            setP.push(pointP);
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
            }
        }
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
        }
        if (setP.empty())
            break;
        pointS = setP.top();
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Dequeued " + std::to_string(pointS[FW_X]) + " " + std::to_string(pointS[FW_Y]) + " - " + std::to_string(pointS[FW_HVC]));
        }
        setP.pop();
        aux = std::shared_ptr<FW_POINT> (new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
        setS.insert(aux);
    };
    return setS;
}

 
/** Uses a hypervolume dichotomic scheme to calculate all optimal points
 *
 *  @return The set of all optimal points
 */
FW_SET hvDichotomicSchemeAll2DP(
        FW_STRUCT       *data       /**< Default framework data structure */
){

    FW_SET setS;
    std::priority_queue<FW_POINT, std::vector<FW_POINT>, Comp2DSetP> setP;
    FW_POINT refPoint = data->iRefPoint;
    FW_POINT pointS;
    FW_POINT pointP;
    FW_POINT succ;
    FW_POINT pred;
    std::vector<std::future<FW_POINT>> threads;

    setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({refPoint[FW_X], FW_INF})));
    setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({FW_INF, refPoint[FW_Y]})));

    pointS = FW_SOLVE(data, refPoint);
    auto aux = std::shared_ptr<FW_POINT> (new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
    setS.insert(aux);

    while(true) {
        succ = *(*(std::next(setS.find(aux))));
        refPoint[FW_X] = pointS[FW_X];
        refPoint[FW_Y] = succ[FW_Y];
        threads.push_back(asyncSolve(FW_SOLVE, data, refPoint));
        pred = *(*(std::prev(setS.find(aux))));
        refPoint[FW_X] = pred[FW_X];
        refPoint[FW_Y] = pointS[FW_Y];
        threads.push_back(asyncSolve(FW_SOLVE, data, refPoint));
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
        }
        while(setP.empty() and !threads.empty()){
            try {
                pointP = threads.at(0).get();
                threads.erase(threads.begin());
            } catch (const std::future_error& e) {
                std::cout << "Caught a future_error with code \"" << e.code()
                          << "\"\nMessage: \"" << e.what() << "\"\n";
            }
            pointP[FW_HVC] = calcHvContribution(pointP, setS);
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " + std::to_string(pointP[FW_HVC]));
            }
            if(pointP[FW_HVC] > 0){
                setP.push(pointP);
                if(FW_VERBOSE){
                    messageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
                }
            }
        }
        if (setP.empty() and threads.empty())
            break;
        pointS = setP.top();                                                                            //     s ← dequeue(P)
        if(FW_VERBOSE){
            messageHandler("Output/framework.txt", "Dequeued " + std::to_string(pointS[FW_X]) + " " + std::to_string(pointS[FW_Y]) + " - " + std::to_string(pointS[FW_HVC]));
        }
        setP.pop();
        aux = std::shared_ptr<FW_POINT> ((new FW_POINT({pointS[FW_X], pointS[FW_Y]})));
        setS.insert(aux);                                                                               //     insert(s, S)
    }
    return setS;                                                                                // return S
}


/** Uses a hypervolume dichotomic scheme to calculate optimal points within a time limit
 *
 *  @note The time limit for the execution of the algorithm must be set prior to calling this method.
 *        Use \ref setTimeLimit to set the time limit for the algorithm and \ref setSCIPtimeout for each
 *        subproblem if using SCIP
 *
 *  @return The set of optimal points calculated until the time limit was reached
 */
FW_SET hvDichotomicSchemeTime2D(
        FW_STRUCT       *data       /**< Default framework data structure */
) {

    auto limit = (FW_TIMENOW + data->timeLimit);

    FW_SET setS;
    std::priority_queue<FW_POINT, std::vector<FW_POINT >, Comp2DSetP> setP;
    FW_POINT refPoint = data->iRefPoint;
    FW_POINT pointS;
    FW_POINT pointP;
    FW_POINT succ;
    FW_POINT pred;

    setS.insert(std::shared_ptr<FW_POINT>(new FW_POINT({refPoint[FW_X], FW_INF})));
    setS.insert(std::shared_ptr<FW_POINT>(new FW_POINT({FW_INF, refPoint[FW_Y]})));

    pointS = FW_SOLVE(data, refPoint);
    auto aux = std::shared_ptr<FW_POINT >(new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
    setS.insert(aux);
    data->timeLimit = FW_TIMELEFT(limit);

    while (data->timeLimit.count() > 0) {
        succ = *(*(std::next(
                setS.find(aux))));
        refPoint[FW_X] = pointS[FW_X];
        refPoint[FW_Y] = succ[FW_Y];
        data->timeLimit = FW_TIMELEFT(limit);
        pointP = FW_SOLVE(data, refPoint);
        pointP[FW_HVC] = calcHvContribution(pointP, setS);
        if (FW_VERBOSE) {
            messageHandler("Output/framework.txt",
                           "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " +
                           std::to_string(pointP[FW_HVC]));
        }
        if (pointP[FW_HVC] >0) {
            setP.push(pointP);
            if (FW_VERBOSE) {
                messageHandler("Output/framework.txt",
                               "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
            }
        }
        pred = *(*(std::prev(
                setS.find(aux))));
        refPoint[FW_X] = pred[FW_X];
        refPoint[FW_Y] = pointS[FW_Y];
        pointP = FW_SOLVE(data, refPoint);
        pointP[FW_HVC] = calcHvContribution(pointP, setS);
        if (FW_VERBOSE) {
            messageHandler("Output/framework.txt",
                           "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " +
                           std::to_string(pointP[FW_HVC]));
        }
        if (pointP[FW_HVC] > 0) {
            setP.push(pointP);
            if (FW_VERBOSE) {
                messageHandler("Output/framework.txt",
                               "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
            }
        }
        if (FW_VERBOSE) {
            messageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
        }
        if (setP.empty())
            break;
        pointS = setP.top();
        if (FW_VERBOSE) {
            messageHandler("Output/framework.txt",
                           "Dequeued " + std::to_string(pointS[FW_X]) + " " + std::to_string(pointS[FW_Y]) + " - " +
                           std::to_string(pointS[FW_HVC]));
        }
        setP.pop();
        aux = std::shared_ptr<FW_POINT >(new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
        setS.insert(aux);

    }
    return setS;
}


/** Uses a hypervolume dichotomic scheme to calculate a specific number of optimal points
 *
 *  @note The number of points to calculate must be set prior to calling this method.
 *        Use \ref setNumSols to set the number of points to calculate.
 *
 *  @return \ref SCIP_OKAY is returned if everything worked. Otherwise a suitable error code is passed. See \ref
 *          SCIP_Retcode "SCIP_RETCODE" for a complete list of error codes.
 */
    FW_SET hvDichotomicSchemeNum2D(
            FW_STRUCT       *data       /**< Default framework data structure */
    ){

        FW_SET setS;
        std::priority_queue<FW_POINT, std::vector<FW_POINT>, Comp2DSetP> setP;
        FW_POINT refPoint = data->iRefPoint;
        FW_POINT pointS;
        FW_POINT pointP;
        FW_POINT succ;
        FW_POINT pred;

        setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({refPoint[FW_X], FW_INF})));
        setS.insert(std::shared_ptr<FW_POINT> (new FW_POINT({FW_INF, refPoint[FW_Y]})));

        pointS = FW_SOLVE(data, refPoint);
        auto aux = std::shared_ptr<FW_POINT> (new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
        setS.insert(aux);

        while(setS.size() - 2 < data->numSols) {
            succ = *(*(std::next(setS.find(aux))));
            refPoint[FW_X] = pointS[FW_X];
            refPoint[FW_Y] = succ[FW_Y];
            pointP = FW_SOLVE(data, refPoint);
            pointP[FW_HVC] = calcHvContribution(pointP, setS);
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " + std::to_string(pointP[FW_HVC]));
            }
            if(pointP[FW_HVC] > 0){
                setP.push(pointP);
                if(FW_VERBOSE){
                    messageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
                }
            }
            pred = *(*(std::prev(setS.find(aux))));
            refPoint[FW_X] = pred[FW_X];
            refPoint[FW_Y] = pointS[FW_Y];
            pointP = FW_SOLVE(data, refPoint);
            pointP[FW_HVC] = calcHvContribution(pointP, setS);
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Solved hv: " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]) + " - " + std::to_string(pointP[FW_HVC]));
            }
            if(pointP[FW_HVC] > 0){
                setP.push(pointP);
                if(FW_VERBOSE){
                    messageHandler("Output/framework.txt", "Queued " + std::to_string(pointP[FW_X]) + " " + std::to_string(pointP[FW_Y]));
                }
            }
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Set P Size " + std::to_string(setP.size()));
            }
            if (setP.empty())
                break;
            pointS = setP.top();
            if(FW_VERBOSE){
                messageHandler("Output/framework.txt", "Dequeued " + std::to_string(pointS[FW_X]) + " " + std::to_string(pointS[FW_Y]) + " - " + std::to_string(pointS[FW_HVC]));
            }
            setP.pop();
            aux = std::shared_ptr<FW_POINT> (new FW_POINT({pointS[FW_X], pointS[FW_Y]}));
            setS.insert(aux);
        };
        return setS;
    }


/** calculates the hypervolume contribution of a point given a set
 *
 *  @note There are 3 solving types, \ref FW_ALL that calculates all optimal points,
 *        \ref FW_TIME that calculates points whithin a time limit and  \ref FW_NUM
 *        that calculates a set number of points.
 *
 *  @return The set of optimal points calculated depending on the solving type
 */
    FW_SET hvDichotomicScheme(
            int             solvingType,/**< Solving type for the problem */
            FW_STRUCT       *data       /**< Default framework data structure */
    ){

        data->input(data);
        data->init(data);

        if(FW_VERBOSE){
            std::ofstream file;
            file.open("Output/framework.txt");
            file.close();
        }

        if(data->printProblem)
            freopen("Output/subproblems.txt","w",stdout);
        else
            freopen(VOIDOUTPUT,"w",stdout);

        FW_SET result;

        switch(solvingType) {
            case FW_2D: {
                result = hvDichotomicSchemeAll2D(data);
                break;
            }
            case FW_2D_PARALLEL: {
                result = hvDichotomicSchemeAll2DP(data);
                break;
            }
            case FW_2D_TIME: {
                result = hvDichotomicSchemeTime2D(data);
                break;
            }
            case FW_2D_NUM: {
                result = hvDichotomicSchemeNum2D(data);
                break;
            }
            default:
                break;
        }

        data->close(data);
        freopen("/dev/tty","w",stdout);
        return result;
    }
#endif
