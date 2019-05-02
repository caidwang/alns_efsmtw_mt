#include <iostream>
#include "ALNS_inc.h"
#include <vector>
#include <VRP_Solution.h>
#include "logistics.hpp"
#include "util.hpp"
#include "RandomRemoval.h"
#include "RandomAndRelatedRemoval.h"
#include "SequentialNodeInsertion.h"
#include "RegretInsertion.h"
#include "VRP_LS.h"

using namespace std;

int main() {
    // 0. 准备数据 读dist,time的矩阵 读节点信息 计算relatedness
    const int depot = 0;
    const int n_customer = 1000;
    const int n_recharge_station = 100;
    constexpr int total_nodes = n_customer + n_recharge_station + 1;
    vector<vector<int>> dist_mat;
    vector<vector<int>> time_mat;
    read_dist_time_mat("../data/input_distance-time.txt", dist_mat, time_mat, total_nodes);
    vector<Node> node_list;
    read_nodes("../data/input_node.csv", node_list);
    Relatedness relatedness(node_list, dist_mat, time_mat);

    // 1. 创建alns必要的对象和导入参数
    // 注册destroy repair操作子
//    TSP_Best_Insert bestI("Best Insertion");
//    TSP_Random_Insert randomI("Random Insertion");
//    TSP_Random_Removal randomR("Random Removal");
//    TSP_Worst_Removal worstR("Worst Removal");
//    TSP_History_Removal historyR("History Removal",CITY_SIZE);
    SequentialNodeInsertion sequentialI("Sequential Node Insertion");
    RegretInsertion regretI("Regret Insertion");
    RandomRemoval randR("Random Removal");
    RandomAndRelatedRemoval randRelR("Random and Related Removal", &relatedness);
    // 构造初始解
    VRP_Solution initialSol(&node_list, &dist_mat, &time_mat, n_customer, total_nodes);

    // 从缓存文件读取初始解 todo 根据文件时间的新旧, 选择最新的文件建初始解
    read_vrp_solution_from_file("../data/init_solution_00", initialSol);
    // 保证初始解可行
    SequentialNodeInsertion.repairSolution(dynamic_cast<ISolution&>(initialSol));


    ALNS_Parameters alnsParam;
    alnsParam.loadXMLParameters("./param.xml");

    CoolingSchedule_Parameters csParam(alnsParam);
    csParam.loadXMLParameters("./param.xml");
    ICoolingSchedule* cs = CoolingScheduleFactory::makeCoolingSchedule(dynamic_cast<ISolution&>(initialSol),csParam);
    SimulatedAnnealing sa(*cs);



    OperatorManager opMan(alnsParam);
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randR));
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randRelR));
//    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(historyR));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(sequentialI));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(regretI));

    SimpleBestSolutionManager bestSM(alnsParam);
    SimpleLocalSearchManager simpleLsManager(alnsParam);

    VRP_LS ls("My LS");

    simpleLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(ls));

    ALNS alns("efsmtw",dynamic_cast<ISolution&>(initialSol),dynamic_cast<IAcceptanceModule&>(sa),alnsParam,dynamic_cast<AOperatorManager&>(opMan),dynamic_cast<IBestSolutionManager&>(bestSM),dynamic_cast<ILocalSearchManager&>(simpleLsManager));

    alns.addUpdatable(dynamic_cast<IUpdatable&>(historyR));

    alns.solve();
    delete cs;
    return 0;
}