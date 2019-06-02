# ifndef LOGISTICS_H
# define LOGISTICS_H
# include <vector>
# include <queue>
# include <string>
# include <cmath>
# include<algorithm>
# include <iostream>
# include<fstream>
#include <alns/ALNS_Parameters.h>

#ifndef INF
#define INF 1000000
#endif


// Node object
const int OPERATION_TIME = 30; // 客户节点和充电站操作时间相同
const int CHARGE_COST = 50; // 一次
const int WAITING_COST = 24; // 元/h
class Node {
public:
    int id;
    int type;
    float demand_weight{0}, demand_volume{0};
    int early_time, last_time;
    Node(): id(0), type(1), early_time(480), last_time(1440) {};
    explicit Node(int id): Node() {this->id = id; this->type = (id == 1)?1 : (id > 1000)? 3: 2; };
    Node(int id, int type, float w, float v, int et, int lt): id(id), type(type), demand_volume(v), demand_weight(w), early_time(et), last_time(lt){ }
};

// vehicle object
enum {IVECO, TRUCK}; //Vehicle Type

class Vehicle {
private:
        int type, v_id;
public:
    // constructor
    Vehicle(int id, int v_type) : v_id(id), type(v_type) { }
    explicit Vehicle(int id) : v_id(id), type(IVECO) { }

    // attribution getter
    int get_vehicle_id() const { return v_id; };
    int get_type() const {return type; }
    int capacity() const;
    double max_weight() const;
    int max_distance() const;
    int cost_pkm() const;
    int fix_cost() const;
};
int inline Vehicle::capacity() const {
    return type == IVECO? 12 : 16;
}
double inline Vehicle::max_weight() const{
    return type == IVECO? 2.0 : 2.5;
}
int inline Vehicle::max_distance() const{
    return type == IVECO? 100000: 120000;
}
int inline Vehicle::cost_pkm() const{
    return type == IVECO? 12 : 14;
}
int inline Vehicle::fix_cost() const{
    return type == IVECO? 200 : 300;
}

// 序列信息快 (用于评价方法)
struct SeqProperty {
    int Dur;    // 经过松弛的seq通过时间
    int E;      // 使得Dur最小的最早到达第一个点时间
    int L;      // 使得Dur最小的最晚到达第一个点时间
    int TW;     // 在E(seq)出发时依然需要的回溯时间
    int WT;     // 在L(seq)出发时依然需要的等待时间
    bool f;     // true为包含充电站 false为不包含充电站
    int Y_prev; // 第一个节点到第一个充电站的里程要求
    int Y_rear; // 最后一个充电站到最后一个节点的历程需求
    int EY;     // 虚拟里程, 即电量违背
};

/**
 * 从节点构建序列信息
 * n: 用于构建序列的节点
 * Return: SeqProperty对象
 */
const SeqProperty make_property_from_node(const Node &n);

/**
 * 惩罚系数管理类
 */
class PenaltyParam {
public:
    //constructor
    PenaltyParam(double v = vw, double w = ww, double t = tw, double e = ew) : volumeW(v), weightW(w), timeWinW(t), energyW(e) , times(1){ }
    // 修改惩罚系数的倍数
    void Raise(int time) {
        times = time;
    }
    // 恢复惩罚系数为默认值
    void setDefault() {
        times = 1;
    }
    // getters
    double getVolumeW() const {
        return times * volumeW;
    }
    double getWeightW() const {
        return times * weightW;
    }
    double getTimeWinW() const {
        return times * timeWinW;
    }
    double getEnergyW() const {
        return times * energyW;
    }
    // 设置默认惩罚系数值
    static void initialPenaltyParam(ALNS_Parameters &param);
private:
    double volumeW, weightW, timeWinW, energyW;
    static double vw, ww, tw,ew; // 用于在初始化参数阶段设置PenaltyParam的默认参数 与initalPaneltyParam 共同使用
    int times; // 当前惩罚系数的倍数
};

class Route {
public:
    Vehicle vehicle;
    std::vector<int> route_seq; // 路线经过的节点的id序列
    double total_weight, total_volume;
    int total_dist;

    Route(int v_id, int v_type, int start_time);
    size_t size() const {return route_seq.size(); }
    int get_node_by_position(int position) const;
    int get_back_time() const;
    int get_start_time() const;
    int get_waiting_time() const;
    double get_penalized_cost() {return penalizedCost; }
    double get_object_cost() {return objCost; }
    void setPenalty(int times); // 设置penalty的倍数, 负责设置后新值的更新
    // route操作子
    void lazy_insert(int id, int position); // 懒惰插入 与update成对使用
    void insert(int id, int position); // 积极插入 O(n)复杂度
    void lazy_remove(int position); // 懒惰删除 O(1)复杂度 与update成对使用
    void remove(int position); // 积极删除 O(n) 复杂度
    void execTwoOpt(int begin, int end); // begin和end是反转序列的开始点和结束点
    // 在insert, remove和execute 之后对属性信息进行更新 failure_begin 原有失效开始的位置和failure_end 原有失效结束的位置, 对于单点插入, 删除 应该是两者应该相等
    void update(int failure_begin, int failure_end) { update_whole_route();} 

    // route评价函数
    double ACUT(); // 计算路径的ACUT, 直观: 单位有效载重的估算价格
    double evaluateInsert(int position, int node_id); // 需要考虑插入客户点和插入RS两种情况
    double evaluateInsert(int position, std::vector<int> node_seq); // 对序列插入的情况 计算结果
    double evaluateRemove(int position); // 需要考虑移除RS和移除客户点两种情况
    double evaluateTwoOpt(int begin, int end); // begin和end是反转序列的开始点和结束点
    //evaluate 系列方法给的是动作发生前后后路径penalizedCost的变化值
    bool isFeasible();

    static void set_graph_info(std::vector<std::vector<int>> &d_mat, std::vector<std::vector<int>> &t_mat, std::vector<Node> &nl);
    static const std::vector<std::vector<int>>* get_dist_mat() {return &dist_mat; }
    static const std::vector<std::vector<int>>* get_time_mat() {return &time_mat; }
    static const std::vector<Node>* get_node_list() {return &node_list; }

private:
    static std::vector<std::vector<int>> dist_mat;
    static std::vector<std::vector<int>> time_mat;
    static std::vector<Node> node_list;

    int start_time, cnt_SR;
    double penalizedCost, objCost;
    std::vector<SeqProperty> forward;
    std::vector<SeqProperty> backward;
    PenaltyParam penaltyParam;
    // 链接操作子 lianjie a子列和b子列 返回合成子列的属性
    SeqProperty property_concatenate(const SeqProperty &a, const SeqProperty &b, int id_last_node_in_a, int id_first_node_in_b, int max_distance);
    // 对seq序列计算用于评价的值
    SeqProperty calPropertyForSeq(const std::vector<int> &seq, double &weight, double &volume, int &distance, int &SR);
    
    void update_whole_route();
    // 从failure position向前向后更新属性值
    void update_partial(int failure_position);

};
/**
 * 计算惩罚目标函数
 * vehicle: 负责这个sequence的车型
 * total_dist: sequence的总里程
 * cnt_SR: 序列中包含的充电站数量
 * total_weight/volume: 序列的总容量
 * seqInfo: 序列的序列信息块
 * paneltyParam: 当前的惩罚系数
 * Return: 带惩罚目标函数值
 */
double calculate_penalized_cost(const Vehicle &vehicle,
                                int total_dist,
                                int cnt_SR,
                                double total_weight,
                                double total_volume,
                                const SeqProperty &seqInfo, const PenaltyParam &penaltyParam);

// 记录插入node点时, 尝试前后充电站的信息
struct InsertInfo {
    int cur_route;
    int cur_position;
    int RS_ahead;       // 节点前插入的RS下标
    int RS_post;        // 节点后插入的RS下标
    double cost;        // 插入成本
    bool change_type;   // 是否改变车型
};

// 用于RCL的比较函数, RCL中, 最大的总是在最前, 当有更小的加入时, 最大的被pop出
struct RCLLess {
    bool operator()(const InsertInfo& x, const InsertInfo& y) const {return x.cost < y.cost; }
    typedef InsertInfo first_argument_type;
    typedef InsertInfo second_argument_type;
    typedef bool result_type;
};

// 在插入中使用的RCL 保存的是插入cost最小的前max_number个插入动作的InsertInfo
class NodePositionRCL {
public:
    explicit NodePositionRCL(int max_number) : maxNumber(max_number) { }
    void push(InsertInfo item) {
        RCL.push(item);
        if (RCL.size() > maxNumber) RCL.pop();
    }
    InsertInfo randGet() {
        int index = std::min(static_cast<unsigned long>(rand() % maxNumber), RCL.size() -1);
        while (index) {
            RCL.pop();
            --index;
        }
        return RCL.top();
    }
private:
    int maxNumber;
    std::priority_queue<InsertInfo, std::vector<InsertInfo>, RCLLess> RCL;
};

// 针对每个节点在每个位置的插入, 进行四种尝试 {v}, {f,v}, {v, g}, {f, v, g}
// 返回带惩罚的cost最小的插入方式的info
InsertInfo evaluate_insert_with_rs(Route &route, int cur_route, int cur_position, int node_id);

// 与evaluate_insert_with_rs一起使用, 在插入时按照info的信息, 确定实际插入时如何处理RS
void do_insert_from_info(Route &route, InsertInfo &info, int node_id);

/**
 * ahead为true时, 返回node_id与cur_position-1节点之间插入的最优充电站
 * ahead为false时, 返回node_id与cur_position节点之间插入的最优充电站
 * 当node_id为已经在i_position位置的节点时, ahead不变, 非ahead的情况, cur_position应为i_position+1
 */
int find_best_charger(Route &cur_route, int cur_position, int node_id, bool ahead,
                      const std::vector<std::vector<int>> *dist_mat);

// 判断remove_list中的元素是否有重复
bool remove_list_is_unique(std:: vector<int> &rList);

# endif //LOGISTICS_H
