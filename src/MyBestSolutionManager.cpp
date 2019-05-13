//
// Created by wsc on 19-5-9.
//

#include "MyBestSolutionManager.h"

#include <list>
#include <VRP_Solution.h>
#include "util.hpp"
using namespace std;

MyBestSolutionManager::MyBestSolutionManager(ALNS_Parameters &param, const std::string &write_path): writePath(std::move(write_path)){
    parameters = &param;
}

MyBestSolutionManager::~MyBestSolutionManager() {
    for(auto & bestSol : bestSols)
    {
        delete bestSol;
    }
}

bool MyBestSolutionManager::isNewBestSolution(ISolution& sol)
{
    for(list<ISolution*>::iterator it = bestSols.begin(); it != bestSols.end(); it++)
    {
        ISolution& currentSol = *(*it);
        if(currentSol<sol)
        {
            return false;
        }
        else if(sol<currentSol)
        {
            delete *it;
            it = bestSols.erase(it);
            if(it == bestSols.end())
            {
                break;
            }
        }
        else if(currentSol.getHash() == sol.getHash())
        {
            return false;
        }
    }
    ISolution* copy = sol.getCopy();
    bestSols.push_back(copy);
    return true;
}

ISolution* MyBestSolutionManager::reloadBestSolution(ISolution* currSol, ALNS_Iteration_Status& status)
{
    if(status.getNbIterationWithoutImprovementSinceLastReload() > 0 &&
       ((status.getNbIterationWithoutImprovementSinceLastReload() % parameters->getReloadFrequency()) == 0)) {
        status.setNbIterationWithoutImprovementSinceLastReload(0);
        delete currSol;
        return bestSols.back()->getCopy();
    }
    else {
        return currSol;
    }
}

void MyBestSolutionManager::saveBestAnswer() {
    if (!bestSols.empty())
        write_answer(bestSols.front(), writePath);
    else {
        cout << "No Feasible Answers yet." << endl;
    }
}


