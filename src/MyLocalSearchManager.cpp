//
// Created by wsc on 19-5-5.
//

#include <chrono>
#include "MyLocalSearchManager.h"
using namespace std;
bool MyLocalSearchManager::useLocalSearch(ISolution& sol, ALNS_Iteration_Status& status)
{
    if(status.getNewBestSolution()!=ALNS_Iteration_Status::TRUE
       || status.getAcceptedAsCurrentSolution()!=ALNS_Iteration_Status::UNKNOWN)
    {
        return false;
    }
    else
    {
        status.setLocalSearchUsed(ALNS_Iteration_Status::TRUE);
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        shuffle(localSearchOperators.begin(), localSearchOperators.end(), default_random_engine(seed)); // todo shuffle 在那个位置比较好
        bool improvement, ever_improved = false;
        do
        {
            improvement = false;
            for(auto & localSearchOperator : localSearchOperators)
            {
                improvement = localSearchOperator->performLocalSearch(sol)||improvement;
                ever_improved = improvement || ever_improved;
            }
        }while(improvement);
        if(ever_improved)
        {
            status.setImproveByLocalSearch(ALNS_Iteration_Status::TRUE);
            return true;
        }
        else
        {
            status.setImproveByLocalSearch(ALNS_Iteration_Status::FALSE);
            return false;
        }
    }
}

void MyLocalSearchManager::addLocalSearchOperator(ILocalSearch& ls)
{
    //TODO find out why the set.find() == set.end() does not work.
    bool ok = true;
    for(size_t i=0; i< param->getForbidenLsOperators().size() && ok; i++)
    {
        if(param->getForbidenLsOperators()[i] == ls.getName())
        {
            std::cout << "NO " << ls.getName() << std::endl;
            ok = false;
        }
    }
    if(ok)
    {
        localSearchOperators.push_back(&ls);
    }

}
