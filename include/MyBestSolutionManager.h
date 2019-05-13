//
// Created by wsc on 19-5-9.
//

#ifndef ALNS_EFSMTW_MT_MYBESTSOLUTIONMANAGER_H
#define ALNS_EFSMTW_MT_MYBESTSOLUTIONMANAGER_H
#include <list>
#include "ALNS_inc.h"
#include "MyBestSolutionManager.h"
#include "MyLocalSearchManager.h"

class MyBestSolutionManager: public IBestSolutionManager {
public:
    MyBestSolutionManager(ALNS_Parameters &param, const std::string &write_path);

    ~MyBestSolutionManager();

    bool isNewBestSolution(ISolution& sol) override ;

    //! Return a pointer to a best solution.
    std::list<ISolution*>::iterator begin(){return bestSols.begin();};

    //! Return a pointer to a best solution.
    std::list<ISolution*>::iterator end(){return bestSols.end();};

    void saveBestAnswer() override;

    //! This function take care of reloading the best known
    //! solution, as the current solution, if needed.
    //! \param currSol a pointer to the current solution.
    //! \param status the status of the current iteration.
    //! \return a pointer to the current solution.
    ISolution* reloadBestSolution(ISolution* currSol, ALNS_Iteration_Status& status) override;

    //! Simple getter.
    std::list<ISolution*>& getBestSols(){return bestSols;};
private:
    std::list<ISolution*> bestSols;
    std::string writePath;
    ALNS_Parameters* parameters;
};


#endif //ALNS_EFSMTW_MT_MYBESTSOLUTIONMANAGER_H
