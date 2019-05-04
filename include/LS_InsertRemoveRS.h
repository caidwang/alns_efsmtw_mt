//
// Created by wsc on 19-5-4.
//

#ifndef ALNS_EFSMTW_MT_LS_INSERTREMOVERS_H
#define ALNS_EFSMTW_MT_LS_INSERTREMOVERS_H

#include "ALNS_inc.h"
#include "logistics.hpp"


/**
 * LocalSearch部分唯一可以增减额外的RS的操作子,
 * 其中当一个元素是RS的时候 尝试删除它 当一个元素前面不是RS的时候尝试加入一个RS
 * 加入RS的选择 使用find_best_charger方法
 */
class LS_InsertRemoveRS : public ILocalSearch{
public:
    explicit LS_InsertRemoveRS(std::string name) : name(std::move(name)) { }
    //! Perform a local search on the solution.
    //! \return true if the solution is improved.
    bool performLocalSearch(ISolution& sol) override ;

    //! \return the name of the local search operator.
    std::string getName() override {return name; }
private:
    std::string name;
};


#endif //ALNS_EFSMTW_MT_LS_INSERTREMOVERS_H
