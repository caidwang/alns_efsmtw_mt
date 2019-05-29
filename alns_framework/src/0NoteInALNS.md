ALNS主框架
```
init:S
while(没有达到停止条件) {
    选择Repair Operator
    选择Destroy Operator
    S'<-S
    destroy(S')
    update(S')
    repair(S')
    newCost = S'目标函数值
    BestSolManager->isNewbestSolution(S') 并根据结果对status进行设置
    检查是否是已知解
    判断S' < S 并更新IterationWithOutImprovement
    if(useLocalSearch) {
        LocalSearch(S')
        BestSolManager->isNewbestSolution(S')
    }
    transitionCurrentSolution(S') 根据判断标准判断是否接受S' 如果接受,则S = S'
    根据解是否能被接受
        更新AcceptAsCurrentSolution和nbIterationWithOutTransition状态
    <<<<<<<<
    if(useLocalSearch) {
        LocalSearch(S')
        BestSolManager->isNewbestSolution(S')
        if(status.AcceptedAsCurrentSolution) {
            根据判断标准判断是否接受S' 如果接受,则S = S'
        }
    }
    ========= 应该被替换为求最优可行解
    if (S' < S)
        it <- 0
        while (!isFeasible(S') && it < 2) {
            ++it;
            if (useLocalSearch) {
                S'.updatePanelty()
                LocalSearch(S')
            }
        }
        if (isFeasible(S')) BestSoverManager->isNewBestSolution(S');
    >>>>>>>>
    updateScores
    if(IterationWC到达一定次数)
        更新权值
    update(S')
    S = bestSolManager->reloadBestSolution(S,status)
}

```