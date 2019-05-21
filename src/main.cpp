#include <iostream>
#include "ALNS_inc.h"
#include <vector>
#include <VRP_Solution.h>
#include <LS_InterRelocate.h>
#include <LS_InsertRemoveRS.h>
#include <fstream>
#include <sstream>
#include "logistics.hpp"
#include "util.hpp"
#include "RandomRemoval.h"
#include "RandomAndRelatedRemoval.h"
#include "SequentialNodeInsertion.h"
#include "RegretInsertion.h"
#include "LS_Two_opt.h"
#include "MyLocalSearchManager.h"
#include "MyBestSolutionManager.h"
#include "DetermineAcceptance.h"

using namespace std;

int main() {
    srand(time(nullptr));
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

    ALNS_Parameters alnsParam;
    alnsParam.loadXMLParameters("../param.xml");

    Relatedness relatedness(node_list, dist_mat, time_mat, n_customer);
    Route::set_graph_info(dist_mat, time_mat, node_list);
    // 设置带惩罚的目标函数的惩罚系数
    PenaltyParam::initialPenaltyParam(alnsParam);
    // 1. 创建alns必要的对象和导入参数
    // 注册destroy repair操作子
//    TSP_Best_Insert bestI("Best Insertion");
//    TSP_Random_Insert randomI("Random Insertion");
//    TSP_Random_Removal randomR("Random Removal");
//    TSP_Worst_Removal worstR("Worst Removal");
//    TSP_History_Removal historyR("History Removal",CITY_SIZE);
    SequentialNodeInsertion sequentialI("Sequential Node Insertion");
    RegretInsertion regret2I("Regret-2 Insertion", 2);
    RegretInsertion regret3I("Regret-3 Insertion", 3);
    RandomRemoval randR("Random Removal");
    RandomAndRelatedRemoval randRelR("Random and Related Removal", relatedness);
    // 构造初始解
    VRP_Solution initialSol(&node_list, &dist_mat, &time_mat, n_customer, total_nodes);

    // 从缓存文件读取初始解 根据文件时间的新旧, 选择最新的文件建初始解
    string fileName;
     if (!lastest_modified_file("../results", fileName)) throw runtime_error("no file in results.");
    string path = "../results/";
    read_vrp_solution_from_file(path + fileName, initialSol);

    // 构造初始可行解
    VRP_Solution initialFSol(&node_list, &dist_mat, &time_mat, n_customer, total_nodes);

    // 从缓存文件读取初始可行解 根据文件时间的新旧, 选择最新的文件建初始解
    if (!lastest_modified_file("../feasibleResults/", fileName)) throw runtime_error("no file in results.");
    path = "../feasibleResults/";
    read_vrp_solution_from_file(path + fileName, initialFSol);
    // 保证初始解可行
    sequentialI.repairSolution(dynamic_cast<ISolution&>(initialSol));




//    CoolingSchedule_Parameters csParam(alnsParam);
//    csParam.loadXMLParameters("../param.xml");
//    ICoolingSchedule* cs = CoolingScheduleFactory::makeCoolingSchedule(dynamic_cast<ISolution&>(initialSol),csParam);
//    SimulatedAnnealing sa(*cs);
    DetermineAcceptance da;


    OperatorManager opMan(alnsParam);
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randR));
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randRelR));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(sequentialI));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(regret2I));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(regret3I));

    MyLocalSearchManager myLsManager(alnsParam);
    MyBestSolutionManager bestSM(alnsParam, "../results/");
    MyBestSolutionManager bestFSM(alnsParam, "../feasibleResults/");
    // 这里用最新的可行解初始化bestFSM
    bestFSM.isNewBestSolution(initialFSol);
    // 用第一个可行解初始化bestFSM
    LS_Two_opt lsTwoOpt("My LS Two Opt");
    LS_InterRelocate lsRelocate("Ls Relocated");
    LS_InsertRemoveRS lsInsertRemoveRs("LS InsertRemove RS");
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsTwoOpt));
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsRelocate));
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsInsertRemoveRs));

    ALNS alns("efsmtw",dynamic_cast<ISolution&>(initialSol),dynamic_cast<IAcceptanceModule&>(da),alnsParam,dynamic_cast<AOperatorManager&>(opMan),dynamic_cast<IBestSolutionManager&>(bestSM),
              dynamic_cast<IBestSolutionManager&>(bestFSM),dynamic_cast<ILocalSearchManager&>(myLsManager));

//    alns.addUpdatable(dynamic_cast<IUpdatable&>(historyR));
    PenaltyParam p;
    cout << p.getWeightW() << " " << p.getVolumeW() << " " << p.getTimeWinW() << " " << p.getEnergyW() << endl;
    alns.solve();
    bestFSM.saveBestAnswer();
    bestSM.saveBestAnswer();
    return 0;
}