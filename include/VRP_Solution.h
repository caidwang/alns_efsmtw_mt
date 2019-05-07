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
    friend void read_vrp_solution_from_file(const std::string &file_path, VRP_Solution &solution);
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
    std::vector<Route> &getRoutes();
    std::vector<int> &getNonInserted();
    int getNNodes() const;
    int getNCustomers() const;
    void remove(std::vector<int> &remove_list);

private:
    const std::vector<Node> *node_list;
    const std::vector<std::vector<int>> *dist_mat;
    const std::vector<std::vector<int>> *time_mat;
    const int n_customers, n_nodes;

    std::vector<Route> routes;
    std::vector<int> nonInserted; // 是否应当只保存客户集?
};

// 从文件读solution到Solution对象, 用于断点继续和初始解载入
void read_vrp_solution_from_file(const std::string &file_path, VRP_Solution &solution);

#endif //ALNS_EFSMTW_MT_VRP_SOLUTION_H
