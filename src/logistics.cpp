#include "logistics.hpp"
#include "util.hpp"
#include <cassert>
using namespace std;

vector<vector<int>> Route::dist_mat;
vector<vector<int>> Route::time_mat;
vector<Node> Route::node_list;

Route::Route(int v_id, int v_type, int start_time) : vehicle(v_id, v_type), total_weight(0), total_volume(0), total_dist(0), start_time(start_time), penaltyParam() {
    route_seq.push_back(0);
    route_seq.push_back(0);
    for (int i = 0; i < 2; ++i) {
        forward.push_back(make_property_from_node(node_list[0]));
        backward.push_back(make_property_from_node(node_list[0]));
    }
    penalizedCost = 0;
    objCost = 0;
}

/**
 * Insert the node to the route at given position
 * O(n)复杂度
 * @param x the index of node in node_list
 * @param position
 */
void Route::insert(const int x, int position) {
    lazy_insert(x, position);
    update_whole_route();
}

void
Route::set_graph_info(std::vector<std::vector<int>> &d_mat, std::vector<std::vector<int>> &t_mat, std::vector<Node> &nl) {
    time_mat = t_mat;
    node_list = nl;
    dist_mat = d_mat;
}

/**
 * 对seq1和seq2的链接操作子
 * @param a 记录seq1的操作子
 * @param b 记录seq2的操作子
 * @param id_last_node_in_a seq1的最后一个节点的id
 * @param id_first_node_in_b seq2的最后一个节点的id
 * @param max_distance 执行seq和seq2的车辆的最大里程
 * @return seq1和seq2链接后的路径的属性信息
 */
SeqProperty
Route::property_concatenate(const SeqProperty &a, const SeqProperty &b, int id_last_node_in_a, int id_first_node_in_b, int max_distance) {
    SeqProperty ret;
    // time property
    int delta = a.Dur - a.TW + time_mat[id_last_node_in_a][id_first_node_in_b]; // 松弛后从seq1的第一个点到seq2的第一个点所需要的时间
    int delta_wt = max(0, b.E - delta - a.L); // 两个seq连接位置的最小等待时间
    int delta_tw = max(0, a.E + delta - b.L); // 两个seq链接位置的最小松弛时间
    ret.Dur = a.Dur + b.Dur + time_mat[id_last_node_in_a][id_first_node_in_b] + delta_wt;
    ret.TW = a.TW + b.TW + delta_tw;
    ret.E = max(b.E - delta, a.E) - delta_wt;
    ret.L = min(b.L - delta, a.L) + delta_tw;
    ret.WT = a.WT + b.WT + delta_wt;
    // energe property
    int delta_y = max(0, a.Y_rear + dist_mat[id_last_node_in_a][id_first_node_in_b] + b.Y_prev - max_distance); // 链接的两个seq中, 前一个seq的最后一个充电站到后一个seq的第一个充电站之间的所需里程和车辆最大里程之间的差值, 当车辆最大里程满足要求时, 为0
    ret.f = a.f || b.f;
    ret.Y_prev = a.f? a.Y_prev : a.Y_prev + dist_mat[id_last_node_in_a][id_first_node_in_b] + b.Y_prev - delta_y;
    ret.Y_rear = b.f? b.Y_rear : a.Y_rear + dist_mat[id_last_node_in_a][id_first_node_in_b] + b.Y_rear - delta_y;
    ret.EY = a.EY + b.EY + delta_y;
    return ret;
}

inline int Route::get_node_by_position(int position) const{
    if (position < 0 || position >= size()) throw invalid_argument("positon is out of range.");
    return route_seq[position];
}

// 对整条路径的backward和forward属性进行更新
void Route::update_whole_route() {
    total_dist = 0;
    total_volume = 0;
    total_weight = 0;
    cnt_SR = 0;
    for (size_t i = 1; i < route_seq.size(); ++i) {
        int node_id = get_node_by_position(i);
        total_volume += node_list[node_id].demand_volume;
        total_weight += node_list[node_id].demand_weight;
        total_dist += dist_mat[get_node_by_position(i-1)][node_id];
        if (node_id > 1000) ++cnt_SR;
    }
    forward.resize(size());
    backward.resize(size());
    forward[0] = make_property_from_node(node_list[0]);
    backward.back() = make_property_from_node(node_list[0]);
    for (int i = 1; i < size(); ++i) {
        forward[i] = property_concatenate(forward[i-1], make_property_from_node(node_list[get_node_by_position(i)]), get_node_by_position(i-1), get_node_by_position(i), vehicle.max_distance());
    }
    for (int i = size() - 2; i >= 0; --i) {
        backward[i] = property_concatenate(make_property_from_node(node_list[get_node_by_position(i)]), backward[i+1], get_node_by_position(i), get_node_by_position(i+1), vehicle.max_distance());
    }
    penalizedCost = calculate_penalized_cost(vehicle, total_dist, cnt_SR, total_weight, total_volume, forward.back(), penaltyParam); // todo 存在问题 这里计算的是最小waiting time的状态 和 start time 不符
    objCost = calculate_penalized_cost(vehicle, total_dist, cnt_SR, total_weight, total_volume, forward.back(), PenaltyParam(0, 0, 0, 0));
}

/**
 * 只负责将node对应的id插入到route序列中, 不对路径属性进行更新
 * 当一次插入多个节点时 可以使用, 但必须要与update_all_route成对使用
 * @param id 插入的node的id
 * @param position 插入位置, 在当前position位置的节点之前插入
 */
void Route::lazy_insert(int id, int position) {
    if (position < 1 || position > size() - 1) throw invalid_argument("Position out of range.");
    if (id < 0 || id > node_list.size()) throw invalid_argument("Node id out of range.");
    route_seq.insert(route_seq.begin() + position, id);
}

// 不用全部更新 只需要从失效点开始更新
void Route::update_partial(int failure_position) {
}

int Route::get_back_time() const {
    return forward.back().E + forward.back().Dur;
}

void Route::lazy_remove(int position) {
    if (position < 1 || position >= size() - 1) throw invalid_argument("position out of range.");
    route_seq.erase(route_seq.begin() + position);
}

void Route::remove(int position) {
    lazy_remove(position);
    update_whole_route();
}

void Route::execTwoOpt(int begin, int end) {
    if (begin == end) return ;
    if (begin > end) throw invalid_argument("begin position must be ahead of end position");
    if (begin <= 0 || begin >= size() - 1 || end >= size() - 1) throw invalid_argument("Depot can't be reverse.");
    vector<int> temp;
    for (auto it = route_seq.begin() + begin; it != route_seq.begin() + end + 1; ++it) {
        temp.push_back(*it);
    }
    auto it = route_seq.begin() + begin;
    for (auto it_temp = temp.rbegin(); it_temp != temp.rend(); ++it, ++it_temp) {
        *it = *it_temp;
    }
    update_whole_route();
}

double Route::ACUT() {
    return static_cast<double>(vehicle.fix_cost() + total_dist * vehicle.cost_pkm()) / total_weight;
}

double Route::evaluateInsert(int position, int node_id) {
    // 当插入点时
    int node_before = get_node_by_position(position-1), node_after = get_node_by_position(position);
    double temp_weight = total_weight + node_list[node_id].demand_weight;
    double temp_volume = total_volume + node_list[node_id].demand_volume;
    int temp_dist = total_dist - dist_mat[node_before][node_after] + dist_mat[node_before][node_id] + dist_mat[node_id][node_after];
    int temp_cnt_SR = node_id > 1000 ? cnt_SR + 1 : cnt_SR;
    SeqProperty temp_route_property = property_concatenate(
            property_concatenate(forward[position - 1], make_property_from_node(node_list[node_id]), node_before, node_id, vehicle.max_distance()),
            backward[position],
            node_id,
            node_after,
            vehicle.max_distance());
    return calculate_penalized_cost(vehicle, temp_dist, temp_cnt_SR, temp_weight, temp_volume, temp_route_property, penaltyParam) - penalizedCost;
}

double Route::evaluateInsert(int position, std::vector<int> node_seq) {
    int node_before = get_node_by_position(position-1), node_after = get_node_by_position(position);
    double sWeight, sVol;
    int sDist, sCntSR;
    SeqProperty seq_to_insert = calPropertyForSeq(node_seq, sWeight, sVol, sDist, sCntSR);
    
    double temp_weight = total_weight + sWeight;
    double temp_volume = total_volume + sVol;
    int temp_dist = total_dist - dist_mat[node_before][node_after] + dist_mat[node_before][node_seq.front()] + dist_mat[node_seq.back()][node_after] + sDist;
    int temp_cnt_SR = cnt_SR + sCntSR;
    
    SeqProperty temp_route_property = property_concatenate(
            property_concatenate(forward[position - 1], seq_to_insert, node_before, node_seq.front(), vehicle.max_distance()),
            backward[position],
            node_seq.back(),
            node_after,
            vehicle.max_distance());
    return calculate_penalized_cost(vehicle, temp_dist, temp_cnt_SR, temp_weight, temp_volume, temp_route_property, penaltyParam) - penalizedCost;
}

double Route::evaluateRemove(int position) {
    int node_before = get_node_by_position(position-1), node_after = get_node_by_position(position+1), node_to_remove = get_node_by_position(position);
    double temp_weight = total_weight - node_list[node_to_remove].demand_weight;
    double temp_volume = total_volume - node_list[node_to_remove].demand_volume;
    int temp_dist = total_dist + dist_mat[node_before][node_after] - dist_mat[node_before][node_to_remove] - dist_mat[node_to_remove][node_after];
    int temp_cnt_SR = node_to_remove > 1000 ? cnt_SR - 1 : cnt_SR;
    SeqProperty temp_route_property = property_concatenate(
            forward[position - 1],
            backward[position + 1],
            node_before,
            node_after,
            vehicle.max_distance());
    return calculate_penalized_cost(vehicle, temp_dist, temp_cnt_SR, temp_weight, temp_volume, temp_route_property, penaltyParam) - penalizedCost;
}

double Route::evaluateTwoOpt(int begin, int end) {
    int node_before = get_node_by_position(begin-1), node_after = get_node_by_position(end+1);
    double sWeight, sVol;
    int sDist, sCntSR;
    vector<int> invSeq; // 获得被反转的子列
    for (auto it = route_seq.begin() + end; it != route_seq.begin() + begin - 1; --it) {
        invSeq.push_back(*it);
    }
    SeqProperty invSeqProperty = calPropertyForSeq(invSeq, sWeight, sVol, sDist, sCntSR); // 计算得到子列的信息
    int temp_dist = total_dist - dist_mat[node_before][get_node_by_position(begin)] - dist_mat[get_node_by_position(end)][node_after] + dist_mat[node_before][get_node_by_position(end)] + dist_mat[get_node_by_position(begin)][node_after];
    // todo 这里可能存在bug 默认子列的内部方向反转但是里程不变
    
    SeqProperty temp_Property = property_concatenate(property_concatenate(forward[begin - 1], invSeqProperty, node_before, invSeq.front(), vehicle.max_distance()),backward[end+1], invSeq.back(), node_after, vehicle.max_distance());

    return calculate_penalized_cost(vehicle, temp_dist, cnt_SR, total_weight, total_volume, temp_Property, penaltyParam) - penalizedCost;
    return 0;
}

/**
 * 对seq序列计算用于评价的值
 * @param seq vector& 用于计算的序列
 * @param weight 该序列的总重量
 * @param volume 该序列的总体积
 * @param distance 该序列的内部总里程
 * @param SR 该序列中的SR数量
 * @return 序列的SeqProperty对象
 */
SeqProperty
Route::calPropertyForSeq(const std::vector<int> &seq, double &weight, double &volume, int &distance, int &SR) {
    int first_node = seq.front();
    weight = node_list[first_node].demand_weight;
    volume = node_list[first_node].demand_volume;
    distance = 0;
    SR = (first_node > 1000) ? 1: 0;
    // 遍历node_seq 计算该seq的SeqProperty
    SeqProperty sProperty = make_property_from_node(node_list[first_node]);
    for (size_t i = 1; i < seq.size(); ++i) {
        const auto &cur = node_list[seq[i]];
        sProperty = property_concatenate(sProperty, make_property_from_node(cur), seq[i-1], seq[i], vehicle.max_distance());
        weight += cur.demand_weight;
        volume += cur.demand_volume;
        distance += dist_mat[seq[i-1]][seq[i]];
        SR += seq[i] > 1000 ? 1 : 0;
    }
    return sProperty;
}

bool Route::isFeasible() {
    SeqProperty &routeProperty = forward.back();
    return routeProperty.EY <= 0 && routeProperty.TW <= 0 && total_volume <= vehicle.capacity() && total_weight <= vehicle.max_weight();
}

int Route::get_start_time() const {
    return forward.back().E;
}

int Route::get_waiting_time() const {
    return forward.back().WT;
}

// 设置penalty的倍数, 负责设置后新值的更新
void Route::setPenalty(int times) {
    assert(times >= 1);
    penaltyParam.Raise(times);
    penalizedCost = calculate_penalized_cost(vehicle, total_dist, cnt_SR, total_weight, total_volume, forward.back(), penaltyParam);
}


// 针对每个节点在每个位置的插入, 进行四种尝试 {v}, {f,v}, {v, g}, {f, v, g}
// 返回带惩罚的cost最小的插入方式的info
InsertInfo evaluate_insert_with_rs(Route &route, int cur_route, int cur_position, int node_id) {
    InsertInfo info{cur_route, cur_position, -1, -1, 0};
    int sr_id_ahead = find_best_charger(route, cur_position, node_id, true, Route::get_dist_mat()); //todo implement find_best_charger
    int sr_id_post = find_best_charger(route, cur_position, node_id, false, Route::get_dist_mat());
    info.cost = route.evaluateInsert(cur_position, node_id);
    double cost_p = route.evaluateInsert(cur_position, vector<int>{node_id, sr_id_post});
    double cost_a = route.evaluateInsert(cur_position, vector<int>{sr_id_ahead, node_id});
    double cost_both = route.evaluateInsert(cur_position, vector<int>{sr_id_ahead, node_id, sr_id_post});
    if (cost_a < info.cost) {
        info.RS_ahead = sr_id_ahead;
        info.cost = cost_a;
    }
    if (cost_p < info.cost) {
        info.RS_ahead = -1;
        info.RS_post = sr_id_post;
        info.cost = cost_p;
    }
    if (cost_both < info.cost) {
        info.RS_post = sr_id_post;
        info.RS_ahead = sr_id_ahead;
        info.cost = cost_both;
    }
    return info;
}

// 与evaluate_insert_with_rs一起使用, 在插入时按照info的信息, 确定实际插入时如何处理RS
void do_insert_from_info(Route &route, InsertInfo &info, int node_id) {
    int it = 0;
    if (info.RS_post > 0) {
        route.lazy_insert(info.RS_post, info.cur_position);
        ++it;
    }
    route.lazy_insert(node_id, info.cur_position);
    ++it;
    if (info.RS_ahead > 0) {
        route.lazy_insert(info.RS_ahead, info.cur_position);
        ++it;
    }
    route.update(info.cur_position, info.cur_position + it);
}

const SeqProperty make_property_from_node(const Node &n) {
    return SeqProperty{0, n.early_time, n.last_time, 0, 0, n.id > 1000, 0, 0, 0};
}

/**
 * 根据信息和penalty计算penalized Object function 当penalty设置为0时, 计算object function
 * @param vehicle 负责序列的车辆类型
 * @param total_dist 序列的总里程
 * @param cnt_SR 序列中的总充电站数
 * @param total_weight 序列的总重
 * @param total_volume 序列的总容量
 * @param seqInfo 序列的属性信息
 * @param penaltyParam 惩罚参数
 * @return 带惩罚的/原目标函数值
 */
double
calculate_penalized_cost(const Vehicle &vehicle,
                         int total_dist,
                         int cnt_SR,
                         double total_weight,
                         double total_volume,
                         const SeqProperty &seqInfo,
                         const PenaltyParam &penaltyParam) {
    if (total_dist == 0) return 0;
    return vehicle.fix_cost()
            + total_dist * vehicle.cost_pkm() / 1000.0
            + cnt_SR * CHARGE_COST
            + WAITING_COST * seqInfo.WT / 60.0
            + seqInfo.EY * penaltyParam.getEnergyW()
            + seqInfo.TW * penaltyParam.getTimeWinW()
            + max(0., total_weight - vehicle.max_weight()) * penaltyParam.getWeightW()
            + max(0., total_volume - vehicle.capacity()) * penaltyParam.getVolumeW();
}


class DistLess {
public:
    typedef int first_argument_type;
    typedef int second_argument_type;
    typedef bool result_type;
    DistLess(int base_node, const vector<vector<int>> *dist_mat) : base(base_node), mat(dist_mat) {}
    bool operator()(int node1, int node2) {
        return (*mat)[base][node1] < (*mat)[base][node2];
    }
private:
    int base;
    const vector<vector<int>> *mat;
};

/**
 * ahead为true时, 返回node_id与cur_position-1节点之间插入的最优充电站
 * ahead为false时, 返回node_id与cur_position节点之间插入的最优充电站
 */
int find_best_charger(Route &cur_route, int cur_position, int node_id, bool ahead, const vector<vector<int>> *dist_mat) {
    static vector<vector<int>> near_chargers;
    if (near_chargers.empty()) {
        for (int i = 0; i <= 1000; ++i) {
            near_chargers.emplace_back();
            for(int j = 1001; j < 1101; ++j) {
                near_chargers[i].push_back(j);
            }
            sort(near_chargers[i].begin(), near_chargers[i].end(), DistLess(i, (dist_mat)));
            near_chargers[i].resize(5);
        }
    }
    if (node_id > 1000) return node_id;
    int node_before, node_after;
    if (ahead) {
        node_before = cur_route.get_node_by_position(cur_position - 1);
        node_after = node_id;
    }
    else {
        node_before = node_id;
        node_after = cur_route.get_node_by_position(cur_position);
    }
    int best_charger = -1, least_dist = INF;
    for (auto charger: near_chargers[node_id]) {
        int cur_dist = (*dist_mat)[node_before][charger] + (*dist_mat)[charger][node_after];
        if (cur_dist < least_dist) {
            least_dist = cur_dist;
            best_charger = charger;
        }
    }
    return best_charger;
}

bool remove_list_is_unique(std::vector<int> &rList) {
    vector<int> cnt(1001, 0);
    for (auto item : rList) {
        ++cnt[item];
    }
    for (int i = 1; i < cnt.size(); ++i) {
        if (cnt[i] > 1) return false;
    }
    return true;
}

double PenaltyParam::vw = 10000;
double PenaltyParam::ww = 10000;
double PenaltyParam::tw = 100;
double PenaltyParam::ew = 10;

// 设置penaltyParam的默认参数
void PenaltyParam::initialPenaltyParam(ALNS_Parameters &param) {
    vw = param.getVolumePenalty();
    ww = param.getWeightPenalty();
    tw = param.getTimeWinPenalty();
    ew = param.getEnergyPenalty();
}


