#include <utility>

//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
#define ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
#ifndef INF
#define INF 1000000
#endif
#include "ALNS_inc.h"
#include "logistics.hpp"

class SequentialNodeInsertion: public ARepairOperator {
public:
    explicit SequentialNodeInsertion(std::string s) : ARepairOperator(std::move(s)) {}

    ~SequentialNodeInsertion() override = default;

    void repairSolution(ISolution& sol) override ;
private:
    insert_info evaluate_insert_with_rs(Route &route, int cur_route, int cur_position, int node_id);
    void do_insert_from_info(Route &route, insert_info &info, int node_id);
};


#endif //ALNS_EFSMTW_MT_SEQUENTIALNODEINSERTION_H
