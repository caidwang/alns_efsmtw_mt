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
    // 从缓存文件读取初始可行解 根据文件时间的新旧, 选择最新的文件建初始可行解
    if (!lastest_modified_file("../feasibleResults/", fileName)) throw runtime_error("no file in results.");
    path = "../feasibleResults/";
    read_vrp_solution_from_file(path + fileName, initialFSol);
    // 保证初始解可行
    sequentialI.repairSolution(dynamic_cast<ISolution&>(initialSol));
    // 使用退火算法作为接受准则
//    CoolingSchedule_Parameters csParam(alnsParam);
//    csParam.loadXMLParameters("../param.xml");
//    ICoolingSchedule* cs = CoolingScheduleFactory::makeCoolingSchedule(dynamic_cast<ISolution&>(initialSol),csParam);
//    SimulatedAnnealing sa(*cs);
    
    // 使用绝对标准作为接受准则
    DetermineAcceptance da;

    // ALNS算子管理器
    OperatorManager opMan(alnsParam);
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randR));
    opMan.addDestroyOperator(dynamic_cast<ADestroyOperator&>(randRelR));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(sequentialI));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(regret2I));
    opMan.addRepairOperator(dynamic_cast<ARepairOperator&>(regret3I));
    
    // LS算子管理器
    MyLocalSearchManager myLsManager(alnsParam);
    // 最优解管理器
    MyBestSolutionManager bestSM(alnsParam, "../results/");
    // 最优可行解管理器
    MyBestSolutionManager bestFSM(alnsParam, "../feasibleResults/");
    // 用最新的可行解初始化bestFSM
    bestFSM.isNewBestSolution(initialFSol); // bestSM的初始化在solver中进行

    // 创建邻域算子实例
    LS_Two_opt lsTwoOpt("My LS Two Opt");
    LS_InterRelocate lsRelocate("Ls Relocated");
    LS_InsertRemoveRS lsInsertRemoveRs("LS InsertRemove RS");
    // 注册邻域算子
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsTwoOpt));
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsRelocate));
    myLsManager.addLocalSearchOperator(dynamic_cast<ILocalSearch&>(lsInsertRemoveRs));
    
    // 创建ALNS实例
    ALNS alns("efsmtw",dynamic_cast<ISolution&>(initialSol),dynamic_cast<IAcceptanceModule&>(da),alnsParam,dynamic_cast<AOperatorManager&>(opMan),dynamic_cast<IBestSolutionManager&>(bestSM),
              dynamic_cast<IBestSolutionManager&>(bestFSM),dynamic_cast<ILocalSearchManager&>(myLsManager));

//    alns.addUpdatable(dynamic_cast<IUpdatable&>(historyR)); // 暂时没有注册update内容
    
    // 打印默认惩罚系数
    PenaltyParam p;
    cout << p.getWeightW() << " " << p.getVolumeW() << " " << p.getTimeWinW() << " " << p.getEnergyW() << endl;
    // 实际求解
    alns.solve();

    // 保存最终的最优解
    bestFSM.saveBestAnswer();
    bestSM.saveBestAnswer();
    return 0;
}