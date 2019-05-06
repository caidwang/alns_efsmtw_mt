//
// Created by wsc on 19-5-6.
//

#include <iostream>
#include "ALNS_inc.h"
#include <vector>
#include <VRP_Solution.h>
#include <LS_Relocate.h>
#include <LS_InsertRemoveRS.h>
#include "logistics.hpp"
#include "util.hpp"
#include "RandomRemoval.h"
#include "RandomAndRelatedRemoval.h"
#include "SequentialNodeInsertion.h"
#include "RegretInsertion.h"
#include "LS_Two_opt.h"
#include "MyLocalSearchManager.h"

using namespace std;
void test_route();
void test_concatenate();
void test_evaluate_coherent(Route &route);

int main() {
    srand(time(nullptr));
    // 0. 准备数据 读dist,time的矩阵 读节点信息 计算relatedness
    const int depot = 0;
    const int n_customer = 1000;
    const int n_recharge_station = 100;
    const double delta = 0.001;
    constexpr int total_nodes = n_customer + n_recharge_station + 1;
    vector<vector<int>> dist_mat;
    vector<vector<int>> time_mat;
    read_dist_time_mat("../data/input_distance-time.txt", dist_mat, time_mat, total_nodes);
    vector<Node> node_list;
    read_nodes("../data/input_node.csv", node_list);
    Relatedness relatedness(node_list, dist_mat, time_mat, n_customer);
    Route::set_graph_info(dist_mat, time_mat, node_list);

    // 构造初始解
    VRP_Solution initialSol(&node_list, &dist_mat, &time_mat, n_customer, total_nodes);

    // 从缓存文件读取初始解 todo 根据文件时间的新旧, 选择最新的文件建初始解
    read_vrp_solution_from_file("../data/init_solution_00", initialSol);

    int randomQ = 50;
    vector<int> removeList;
    set<int> removeSet;
    while(removeList.size() < randomQ) {
        int node = 1 + rand() % initialSol.getNCustomers();
        if (removeSet.find(node) == removeSet.end()) {
            removeSet.insert(node);
            removeList.push_back(node);
        }
    }
    initialSol.remove(removeList); // todo implement remove
//    RandomRemoval randR("Random Removal");
//    RandomAndRelatedRemoval randRelR("Random and Related Removal", relatedness);

//    randR.destroySolution(initialSol);
//    print_solution(initialSol, cout);


    return 0;
}

void test_route_gets() {
    Route route(0, 1, 480);
    assert(route.get_object_cost() == 0 && route.get_penalized_cost() == 0 && route.total_weight == 0 && route.total_volume == 0 && route.total_dist == 0);
    assert(route.get_node_by_position(0) == 0 && route.get_node_by_position(1) == 0);
    assert(route.get_start_time() == route.get_back_time());
    assert(route.isFeasible());

    route.insert(1,1);
    // 0, 1, 0
    assert(route.total_dist == 127072);
    assert(route.get_back_time() == 634);
    assert(abs(route.get_object_cost() - 1724.864) <= 0.001);
//    assert(abs(route.get_penalized_cost() - 272444.864) <= 0.001);

    route.remove(1);

    assert(route.get_object_cost() == 0 && route.get_penalized_cost() == 0 && route.total_weight == 0 && route.total_volume == 0 && route.total_dist == 0);
    assert(route.get_node_by_position(0) == 0 && route.get_node_by_position(1) == 0);
    assert(route.get_start_time() == route.get_back_time());
    assert(route.isFeasible());
}

void test_evaluate_coherent(VRP_Solution &sol) {
    const double delta = 0.001;
    Route route0 = sol.getRoutes()[0];
    vector<int> seq_to_insert;
    for (int i = 4; i < 7; ++i) {
        seq_to_insert.push_back(route0.get_node_by_position(i));
    }
    Route route1 = route0;
    Route route2 = route0;
    for (int i = 4; i < 7; ++i) {
        route1.remove(4);
        route2.remove(4);
    }
    assert(abs(route1.get_penalized_cost() + route1.evaluateInsert(4, seq_to_insert) - route0.get_penalized_cost()) < delta);

    for (int i = 0; i < 10; ++i ) {
        int index = rand() % sol.getRoutes().size();
        Route route = sol.getRoutes()[index];

        double penalty_cost = route.get_penalized_cost();
        for (int position = 1; position < route.size() - 2; ++position) {
            int test_node = route.get_node_by_position(position);
            double remove_cost = route.evaluateRemove(position);
            route.remove(position);
            double penalty_cost1 = route.get_penalized_cost();
            assert(abs(penalty_cost1 + route.evaluateInsert(position, test_node) - penalty_cost) < delta);
            assert(abs(penalty_cost + remove_cost - penalty_cost1) < delta);
            route.insert(test_node, position);
            int end = position + min(rand() % route.size(), route.size() - 2 - position);
            double twoopt_cost = route.evaluateTwoOpt(position, end);
            route.execTwoOpt(position, end);
            double penalty_cost3 = route.get_penalized_cost();
            assert(abs(penalty_cost + twoopt_cost - penalty_cost3) < delta);
            route.execTwoOpt(position, end);
            assert(abs(route.get_penalized_cost() - penalty_cost) < delta);
        }
    }
}
