//
// Created by wsc on 19-4-27.
//

#ifndef ALNS_EFSMTW_MT_VRP_SOLUTION_H
#define ALNS_EFSMTW_MT_VRP_SOLUTION_H

#include <vector>
#include <list>
#include "ALNS_inc.h"
#include "logistics.hpp"

class VRP_Solution: public ISolution {
public:
    // 默认0为depot, customer_id为1~n_customers, 充电站id为n_customers+1~n_node
    VRP_Solution(const std::vector<Node> *node_list,
            const std::vector<std::vector<int>> *dist_mat,
            const std::vector<std::vector<int>> *time_mat,
            int n_customers, int n_nodes);
    ~VRP_Solution() override= default;
    //! A getter for the value of the objective function.
    //! \return the value of the objective function of this solution.
    double getObjectiveValue() override ;
    //! \return a penalized version of the objective value if the solution
    //! is infeasible.
    double getPenalizedObjectiveValue() override ;
    //! A getter for the feasibility of the current solution.
    //! \return true if the solution is feasible, false otherwise.
    bool isFeasible() override ;
    //! A comparator.
    //! \return true if this solution is "better" than the solution it is compared to.
    bool operator<(ISolution&) override ;
    //! Compute the "distance" between solution.
    //! This feature can be used as part of the ALNS to favor the
    //! diversification process. If you do not plan to use this feature
    //! just implement a method returning 0.
    int distance(ISolution&) override ;
    //! This method create a copy of the solution.
    ISolution* getCopy() override ;
    //! Compute a hash key of the solution.
    long long getHash() override ;
    // getters
    const std::vector<Route> &getRoutes() const;
    const std::vector<int> &getNonInserted() const;
    int getNNodes() const;
    int getNCustomers() const;

private:
    const std::vector<Node> *node_list;
    const std::vector<std::vector<int>> *dist_mat;
    const std::vector<std::vector<int>> *time_mat;
    std::vector<Route> routes;
    std::vector<int> nonInserted;
    int cost, n_customers, n_nodes;

};


#endif //ALNS_EFSMTW_MT_VRP_SOLUTION_H
