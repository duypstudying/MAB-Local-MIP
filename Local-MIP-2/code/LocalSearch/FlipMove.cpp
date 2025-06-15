/*=====================================================================================

    Filename:     FlipMove.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#include "LocalMIP.h"

// 执行翻转操作（二进制变量的0/1切换）
bool LocalMIP::FlipMove(
    vector<bool> &_scoreTable,    // 记录已评估过的变量
    vector<size_t> &_scoreIdx)    // 记录评估顺序
{
    if (localVarUtil.binaryIdx.size() == 0)  // 无二进制变量时直接返回
        return false;

    // 初始化最优解记录
    long bestScore = 0;
    long bestSubscore = -std::numeric_limits<long>::max();
    size_t bestVarIdx = -1;
    Value bestDelta = 0;

    // 随机尝试bmsFlip次翻转
    for (size_t idx = 0; idx < bmsFlip; ++idx) {
        // 随机选择一个二进制变量
        size_t varIdx = localVarUtil.binaryIdx[mt() % localVarUtil.binaryIdx.size()];

        // 跳过已评估的变量
        if (_scoreTable[varIdx]) continue;

        // 标记并记录当前评估的变量
        _scoreTable[varIdx] = true;
        _scoreIdx.push_back(varIdx);

        // 获取变量信息
        auto &localVar = localVarUtil.GetVar(varIdx);
        auto &modelVar = modelVarUtil->GetVar(varIdx);
        assert(modelVar.type == VarType::Binary);  // 确保是二进制变量

        // 计算翻转后的值变化（1->0或0->1）
        Value delta = (localVar.nowValue > 0.5) ? -1 : 1;

        // 检查是否允许在当前步骤进行此变化
        if ((delta < 0 && curStep < localVar.allowDecStep) ||
            (delta > 0 && curStep < localVar.allowIncStep))
            continue;

        // 计算分数变化（主分数和子分数）
        long score = TightScore(modelVar, delta);

        // 更新最优解
        if (bestScore < score ||
           (bestScore == score && bestSubscore < subscore)) {
            bestScore = score;
            bestVarIdx = varIdx;
            bestDelta = delta;
            bestSubscore = subscore;
           }
    }

    // 执行最优翻转
    if (bestScore > 0) {
        if (DEBUG) printf("Flip: %-11ld; ", bestScore);  // 调试输出
        ++flipStep;
        ApplyMove(bestVarIdx, bestDelta);  // 应用变化
        PickVar[bestVarIdx]++;
        return true;
    }
    return false;  // 无改进时返回false
}