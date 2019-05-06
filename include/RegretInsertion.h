//
// Created by wsc on 19-5-2.
//

#ifndef ALNS_EFSMTW_MT_REGRETINSERTION_H
#define ALNS_EFSMTW_MT_REGRETINSERTION_H

#include "ALNS_inc.h"
#include "logistics.hpp"

class RegretInsertion: public ARepairOperator {
public:
    RegretInsertion(std::string s, int k) : ARepairOperator(std::move(s)), k(k) {}
    ~RegretInsertion() override = default;
    void repairSolution(ISolution& sol) override ;

private:
    int k;
    std::vector<bool> visited;
    void update_position_cost(std::vector<int> &NonInsert, std::vector<Route> &routes,
                              std::vector<std::vector<InsertInfo>> &pos_cost);
    void calculate_regret_value(std::vector<std::vector<InsertInfo>> &pos_cost,
            std::vector<double> &regret_value);
};

#endif //ALNS_EFSMTW_MT_REGRETINSERTION_H
