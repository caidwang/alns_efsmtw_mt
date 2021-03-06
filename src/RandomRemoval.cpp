//
// Created by wsc on 19-5-1.
//

#include <VRP_Solution.h>
#include "RandomRemoval.h"
#include <cstdlib>
using namespace std;

void RandomRemoval::destroySolution(ISolution &sol) {
    auto &vrp_s = dynamic_cast<VRP_Solution&>(sol);
    // todo Q的基数是customer好还是nodes好 现在是customer
    int randomQ = (rand() % static_cast<int>((MAX_Q_RATE - MIN_Q_RATE) * static_cast<double>(vrp_s.getNCustomers()))) + static_cast<int>(MIN_Q_RATE * static_cast<double>(vrp_s.getNCustomers()));
    vector<int> removeList;
    set<int> removeSet;
    while(removeSet.size() < randomQ) {
        int node = 1 + rand() % vrp_s.getNCustomers();
        if (removeSet.find(node) == removeSet.end()) {
            removeSet.insert(node);
        }
    }
    for (auto i :removeSet) {
        removeList.push_back(i);
    }
    vrp_s.remove(removeList); // todo implement remove
}

