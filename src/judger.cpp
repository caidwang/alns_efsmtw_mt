//
// Created by wsc on 19-5-18.
//
#include <iostream>
#include <vector>
#include "ALNS_inc.h"
#include <vector>
#include <VRP_Solution.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "logistics.hpp"
#include "util.hpp"
using namespace std;

int main1(int argc, char* argv[]) {
    const int depot = 0;
    const int n_customer = 1000;
    const int n_recharge_station = 100;
    constexpr int total_nodes = n_customer + n_recharge_station + 1;
    vector<vector<int>> dist_mat;
    vector<vector<int>> time_mat;
    read_dist_time_mat("../data/input_distance-time.txt", dist_mat, time_mat, total_nodes);
    vector<Node> node_list;
    read_nodes("../data/input_node.csv", node_list);
    Route::set_graph_info(dist_mat, time_mat, node_list);
    VRP_Solution initialSol(&node_list, &dist_mat, &time_mat, n_customer, total_nodes);

    string fileName;
    if (argc < 2) throw invalid_argument("need file name");

    read_vrp_solution_from_file(argv[1], initialSol);
    cout << "the solution is " << (initialSol.isFeasible() ? "feasible" : "unfeasible") << " the cost is " << fixed << setprecision(2) << initialSol.getObjectiveValue() << endl;
    return 0;
}