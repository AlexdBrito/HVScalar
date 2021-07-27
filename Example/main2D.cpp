#include "hvscalar.h"

struct problemVariables {
    int numElems;                                 /**< Number of elements for each objective */
    int maxElems;                                 /**< Max number of elements in each solution */
    std::vector<SCIP_Real> arrayA;                /**< Gain of all elements for objective A */
    std::vector<SCIP_Real> arrayB;                /**< Gain of all elements for objective B */
    SCIP *scip;                                   /**< Pointer for SCIP*/
    std::vector<std::vector<SCIP_Real>> matrixQ;  /**< */
    std::vector<SCIP_VAR *> matrixY;              /**< */
    SCIP_CONS *cons1 = nullptr;                   /**< sum{i=1.n} (a{i}y{ii}) >= r1 */
    SCIP_CONS *cons2 = nullptr;                   /**< sum{i=1.n} (b{i}y{ii}) >= r2 */
    SCIP_CONS *cons3 = nullptr;                   /**< sum{i=1.n} (y{ii}) = k */
    std::vector<SCIP_CONS *> cons4;               /**< sum{i=1.n; i != j} (y{ij}) <= (k-1)y{jj} for all j in {1.n} */
    std::vector<SCIP_CONS *> cons5;               /**< y{ij} = y{ji} for all i,j in {1.n}, i<j */
    std::vector<SCIP_CONS *> cons6;               /**< y{ij}j}  <= y{ii} for all i,j in {1.n}, i!=j */
    std::vector<SCIP_CONS *> cons7;               /**< y{i>= y{ii} + y{ij} - 1 for all i,j in {1.n}, i<j */
};

void readInput(HVS *hvs) {
    SCIP_Real aux;

    HVS_Point refPoint;

    struct problemVariables* pVars = ((problemVariables*)(HVSgetDataStructure(hvs)));

    std::cin >> pVars->numElems;

    pVars->arrayA.reserve(pVars->numElems);
    pVars->arrayB.reserve(pVars->numElems);

    for (int i = 0; i < pVars->numElems; i++) {
        std::cin >> aux;
        pVars->arrayA.emplace_back(aux);
    }
    for (int i = 0; i < pVars->numElems; i++) {
        std::cin >> aux;
        pVars->arrayB.emplace_back(aux);
    }

    std::cin >> aux;
    refPoint.emplace_back(aux);
    std::cin >> aux;
    refPoint.emplace_back(aux);

    HVSsetIRefPoint(hvs, refPoint);

    std::cin >> pVars->maxElems;
}

SCIP_RETCODE initProblem(HVS *hvs) {

    struct problemVariables* pVars = ((problemVariables*)(HVSgetDataStructure(hvs)));

    HVS_Point iRefPoint = HVSgetIRefPoint(hvs);

    /* init and calculate matrix Q for initial reference point */
    pVars->matrixQ.resize(pVars->numElems);
    for(int i = 0; i < pVars->numElems; i++) {
        pVars->matrixQ[i].resize(pVars->numElems);
        for(int j = 0; j < pVars->numElems; j++) {
            if(i == j){
                pVars->matrixQ[i][j] = pVars->arrayA[i] * pVars->arrayB[j] - (iRefPoint[HVS_Y] * pVars->arrayA[i] + iRefPoint[HVS_X] * pVars->arrayB[j]) + iRefPoint[HVS_X] * iRefPoint[HVS_Y];
            } else {
                pVars->matrixQ[i][j] = pVars->arrayA[i] * pVars->arrayB[j];
            }
        }
    }

    /* init SCIP */
    SCIPcreate(&pVars->scip);

    /* include default plugins */
    SCIP_CALL(SCIPincludeDefaultPlugins(pVars->scip));

    /* define problem */
    SCIP_CALL(SCIPcreateProbBasic(pVars->scip, "Rectangular Knapsack"));

    /* set objective function direction */
    SCIP_CALL(SCIPsetObjsense(pVars->scip, SCIP_OBJSENSE_MAXIMIZE));

    // SCIPsetMessagehdlrLogfile(pVars->scip, "filename"); //for printing to file
    SCIPsetMessagehdlrQuiet(pVars->scip, true); //for no output

    /* init and add binary variable matrixY to scip */
    pVars->matrixY.resize(pVars->numElems * pVars->numElems + 1);
    for (int i = 0; i < pVars->numElems; i++) {
        for (int j = 0; j < pVars->numElems; j++) {
            SCIP_CALL(SCIPcreateVarBasic(pVars->scip, &(pVars->matrixY.at(pVars->numElems * i + j)), ("y" + std::to_string(i) + std::to_string(j)).c_str(), 0.0, 1.0, pVars->matrixQ[i][j], SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(pVars->scip, pVars->matrixY.at(pVars->numElems * i + j)));
        }
    }
    SCIP_CALL(SCIPcreateVarBasic(pVars->scip, &(pVars->matrixY.back()), ("r1r2"), 1.0, 1.0, iRefPoint[HVS_X] * iRefPoint[HVS_Y], SCIP_VARTYPE_BINARY));
    SCIP_CALL(SCIPaddVar(pVars->scip, pVars->matrixY.back()));

    /* init and add all constraints to scip */
    pVars->cons4.resize(pVars->numElems, nullptr);
    pVars->cons5.resize((int)((pVars->numElems - 1) * (pVars->numElems)) / 2, nullptr);
    pVars->cons6.resize(pVars->numElems * pVars->numElems - pVars->numElems, nullptr);
    pVars->cons7.resize((int)((pVars->numElems - 1) * (pVars->numElems)) / 2, nullptr);

    SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons1, "cons1", 0, nullptr, nullptr, iRefPoint[HVS_X], HVS_Inf));
    SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons2, "cons2", 0, nullptr, nullptr, iRefPoint[HVS_Y], HVS_Inf));
    SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons3, "cons3", 0, nullptr, nullptr, pVars->maxElems, pVars->maxElems));

    for (int i = 0; i < pVars->numElems; i++) {
        SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons1, pVars->matrixY.at(pVars->numElems * i + i), pVars->arrayA[i]));
        SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons2, pVars->matrixY.at(pVars->numElems * i + i), pVars->arrayB[i]));
        SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons3, pVars->matrixY.at(pVars->numElems * i + i), 1));
    }

    for (int j = 0; j < pVars->numElems; j++) {
        SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons4.at(j), ("cons4_" + std::to_string(j + 1)).c_str(), 0, nullptr, nullptr, -HVS_Inf, 0));
        for (int i = 0; i < pVars->numElems; i++) {
            if (i != j) {
                SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons4.at(j), pVars->matrixY.at(pVars->numElems * i + j), 1));
            }
        }
        SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons4.at(j), pVars->matrixY.at(pVars->numElems * j + j), - (pVars->maxElems - 1)));
    }
    int k = 0;
    for (int j = 1; j < pVars->numElems; j++) {
        for (int i = 0; i < j; i++) {
            SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons5.at(k), ("cons5_" + std::to_string(k + 1)).c_str(), 0, nullptr, nullptr, 0, 0));
            SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons5.at(k), pVars->matrixY.at(i * pVars->numElems + j), 1));
            SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons5.at(k), pVars->matrixY.at(j * pVars->numElems + i), -1));

            SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons7.at(k), ("cons7_" + std::to_string(k + 1)).c_str(), 0, nullptr, nullptr, -1, HVS_Inf));
            SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons7.at(k), pVars->matrixY.at(i * pVars->numElems + j), 1));
            SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons7.at(k), pVars->matrixY.at(i * pVars->numElems + i), -1));
            SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons7.at(k), pVars->matrixY.at(j * pVars->numElems + j), -1));

            k++;
        }
    }
    k = 0;
    for (int i = 0; i < pVars->numElems; i++) {
        for (int j = 0; j < pVars->numElems; j++) {
            if (i != j) {
                SCIP_CALL(SCIPcreateConsBasicLinear(pVars->scip, &pVars->cons6.at(k), ("cons6_" + std::to_string(k + 1)).c_str(), 0, nullptr, nullptr, -HVS_Inf, 0));
                SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons6.at(k), pVars->matrixY.at(i * pVars->numElems + j), 1));
                SCIP_CALL(SCIPaddCoefLinear(pVars->scip, pVars->cons6.at(k), pVars->matrixY.at(i * pVars->numElems + i), -1));
                k++;
            }
        }
    }

    SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons1));
    SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons2));
    SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons3));


    for (int i = 0; i < pVars->cons4.size(); i++) {
        SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons4.at(i)));
    }

    for (int i = 0; i < pVars->cons5.size(); i++) {
        SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons5.at(i)));
    }

    for (int i = 0; i < pVars->cons6.size(); i++) {
        SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons6.at(i)));
    }

    for (int i = 0; i < pVars->cons7.size(); i++) {
        SCIP_CALL(SCIPaddCons(pVars->scip, pVars->cons7.at(i)));
    }

    return SCIP_OKAY;
}

HVS_Point solveProblem(HVS *hvs, HVS_Point refPoint) {

    struct problemVariables* pVars = ((problemVariables*)(HVSgetDataStructure(hvs)));

    HVS_Point calculatedPoint;

    SCIP *scipCp = nullptr;

    /* creating the SCIP that will be used to solve the problem  */
    SCIPcreate(&scipCp);

    /* updating matrixQ in position (i,i) due to new reference point  */
    for(int i = 0; i < pVars->numElems; i++) {
        pVars->matrixQ[i][i] = pVars->arrayA[i] * pVars->arrayB[i] - (refPoint[HVS_Y] * pVars->arrayA[i] + refPoint[HVS_X] * pVars->arrayB[i]);
        SCIPchgVarObj(pVars->scip, pVars->matrixY.at(pVars->numElems * i + i), pVars->matrixQ[i][i]);
    }
    /* updating r1r2 due to new reference point */
    SCIPchgVarObj(pVars->scip, pVars->matrixY.back(), refPoint[HVS_X] * refPoint[HVS_Y]);

    /* updating constraints due to new reference point  */
    (SCIPchgLhsLinear(pVars->scip, pVars->cons1, refPoint[HVS_X]));
    (SCIPchgLhsLinear(pVars->scip, pVars->cons2, refPoint[HVS_Y]));

    /* copying the problem to the new SCIP env  */
    (SCIPcopy(pVars->scip, scipCp, nullptr, nullptr, (std::to_string(refPoint[HVS_X]) + "," + std::to_string(refPoint[HVS_Y])).c_str(), 1, 1, 0, 1, nullptr));

    /* solving the problem  */
    (SCIPsolve(scipCp));

    /* getting the best solution  */
    SCIP_SOL *sol = SCIPgetBestSol(scipCp);

    /* extracting the matrix of binary variables  */
    int numVars = SCIPgetNOrigVars(scipCp);
    SCIP_VAR **vars = SCIPgetOrigVars(scipCp);
    double *results = (double *)malloc(sizeof(double)*numVars);
    SCIPgetSolVals(scipCp, sol, numVars, vars, results);

    /* calculating the hypervolume coordinates of the solution  */
    SCIP_Real hvX = 0;
    SCIP_Real hvY = 0;
    for(int i = 0; i < pVars->numElems; i++) {
        hvX += pVars->arrayA[i] * results[i * pVars->numElems + i];
        hvY += pVars->arrayB[i] * results[i * pVars->numElems + i];
    }

    /* extracting the value of the solution  */
    calculatedPoint = {hvX, hvY, 0, refPoint[HVS_X], refPoint[HVS_Y]};

    free(results);

    /* freeing the SCIP environment */
    (SCIPfree(&scipCp));

    return calculatedPoint;
}

SCIP_RETCODE closeProblem(HVS *hvs) {

    struct problemVariables* pVars = ((problemVariables*)(HVSgetDataStructure(hvs)));

    /* freeing the variables */
    for (int i = 0; i < pVars->matrixY.size(); i++) {
        SCIP_CALL(SCIPreleaseVar(pVars->scip, &pVars->matrixY.at(i)));
    }

    /* freeing the constraints */
    SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons1));
    SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons2));
    SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons3));


    for (int i = 0; i < pVars->cons4.size(); i++) {
        SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons4.at(i)));
    }

    for (int i = 0; i < pVars->cons5.size(); i++) {
        SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons5.at(i)));
    }

    for (int i = 0; i < pVars->cons6.size(); i++) {
        SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons6.at(i)));
    }

    for (int i = 0; i < pVars->cons7.size(); i++) {
        SCIP_CALL(SCIPreleaseCons(pVars->scip, &pVars->cons7.at(i)));
    }

    /* freeing the SCIP environment */
    SCIP_CALL(SCIPfree(&pVars->scip));

    return SCIP_OKAY;
}

int main(int argc, const char *argv[]) {

    HVS *hvs;
    HVSstart(&hvs);

    HVSsetInput(hvs, readInput);
    HVSsetInit(hvs, initProblem);
    HVSsetSolve(hvs, solveProblem);
    HVSsetClose(hvs, closeProblem);
    HVSsetNumNdPoints(hvs, 5);
    HVSsetVerbose(hvs, true);
    HVSsetDataStructure(hvs, new problemVariables());
    HVSsolve(hvs, HVS_2D_NUM);
    HVS_Set results = HVSgetSols(hvs);
    HVSprintSet(results);

    HVSfree(&hvs);

    return 0;
}
