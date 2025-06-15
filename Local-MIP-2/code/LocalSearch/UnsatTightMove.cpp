/*=====================================================================================

    Filename:     UnsatTightMove.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#include "LocalMIP.h"

bool LocalMIP::UnsatTightMove()
{
  vector<bool> &scoreTable = localVarUtil.scoreTable; // 记录变量是否已被评分的表
  vector<size_t> scoreIdxs; // 记录已评分变量的索引
  long bestScore = 0; // 最佳得分
  long bestSubscore = -std::numeric_limits<long>::max(); // 次佳得分
  size_t bestVarIdx = -1; // 最佳变量的索引
  Value bestDelta = 0; // 最佳变量的变化值

  // 使用临时向量存储候选变量及其变化值
  vector<size_t> &neighborVarIdxs = localVarUtil.tempVarIdxs;
  vector<Value> &neighborDeltas = localVarUtil.tempDeltas;
  neighborVarIdxs.clear();
  neighborDeltas.clear();

  // 第一部分：处理不满足的约束
  if (localConUtil.unsatConIdxs.size() > 0)
  {
    size_t neighborSize = localConUtil.unsatConIdxs.size();
    vector<size_t> *neighborConIdxs = &localConUtil.unsatConIdxs;

    // 如果候选约束过多，随机采样部分约束
    if (sampleUnsat < neighborSize)
    {
      neighborSize = sampleUnsat;
      neighborConIdxs = &localConUtil.tempUnsatConIdxs;
      neighborConIdxs->clear();
      neighborConIdxs->assign(
          localConUtil.unsatConIdxs.begin(), localConUtil.unsatConIdxs.end());
      for (size_t sampleIdx = 0; sampleIdx < sampleUnsat; ++sampleIdx)
      {
        size_t randomIdx = mt() % (neighborConIdxs->size() - sampleIdx);
        size_t temp = neighborConIdxs->at(sampleIdx);
        neighborConIdxs->at(sampleIdx) = neighborConIdxs->at(randomIdx + sampleIdx);
        neighborConIdxs->at(randomIdx + sampleIdx) = temp;
      }
    }

    // 遍历候选约束，收集变量及其变化值
    for (size_t neighborIdx = 0; neighborIdx < neighborSize; ++neighborIdx)
    {
      auto &localCon = localConUtil.conSet[neighborConIdxs->at(neighborIdx)];
      auto &modelCon = modelConUtil->conSet[neighborConIdxs->at(neighborIdx)];

      for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
      {
        size_t varIdx = modelCon.varIdxSet[termIdx];
        auto &localVar = localVarUtil.GetVar(varIdx);
        auto &modelVar = modelVarUtil->GetVar(varIdx);

        // 计算使约束更紧的变化值 delta
        Value delta;
        if (!TightDelta(localCon, modelCon, termIdx, delta))
          if (modelCon.coeffSet[termIdx] > 0)
            delta = modelVar.lowerBound - localVar.nowValue;
          else
            delta = modelVar.upperBound - localVar.nowValue;

        // 跳过以下情况：
        // 1. 移动违反禁忌条件（允许变化的步数）
        // 2. delta 太小（小于可行性容忍度）
        if (delta < 0 && curStep < localVar.allowDecStep ||
            delta > 0 && curStep < localVar.allowIncStep)
          continue;
        if (fabs(delta) < FeasibilityTol)
          continue;

        // 将有效的移动添加到候选列表
        neighborVarIdxs.push_back(varIdx);
        neighborDeltas.push_back(delta);
      }
    }
  }

  // 第二部分：处理目标函数（如果已找到可行解且目标函数不满足）
  auto &localObj = localConUtil.conSet[0];
  auto &modelObj = modelConUtil->conSet[0];
  if (isFoundFeasible && localObj.UNSAT())
    for (size_t termIdx = 0; termIdx < modelObj.termNum; ++termIdx)
    {
      size_t varIdx = modelObj.varIdxSet[termIdx];
      auto &localVar = localVarUtil.GetVar(varIdx);
      auto &modelVar = modelVarUtil->GetVar(varIdx);

      // 计算使目标函数更紧的变化值 delta
      Value delta;
      if (!TightDelta(localObj, modelObj, termIdx, delta))
        if (modelObj.coeffSet[termIdx] > 0)
          delta = modelVar.lowerBound - localVar.nowValue;
        else
          delta = modelVar.upperBound - localVar.nowValue;

      // 跳过无效移动
      if (delta < 0 && curStep < localVar.allowDecStep ||
          delta > 0 && curStep < localVar.allowIncStep)
        continue;
      if (fabs(delta) < FeasibilityTol)
        continue;

      neighborVarIdxs.push_back(varIdx);
      neighborDeltas.push_back(delta);
    }

  // 第三部分：如果候选变量过多，使用 BMS（束搜索）随机选择部分候选
  size_t scoreSize = neighborVarIdxs.size();
  if (!isFoundFeasible && scoreSize > bmsUnsatInfeas ||
      isFoundFeasible && scoreSize > bmsUnsatFeas)
  {
    if (!isFoundFeasible)
      scoreSize = bmsUnsatInfeas;
    else
      scoreSize = bmsUnsatFeas;
    for (size_t bmsIdx = 0; bmsIdx < scoreSize; ++bmsIdx)
    {
      size_t randomIdx = (mt() % (neighborVarIdxs.size() - bmsIdx)) + bmsIdx;
      size_t varIdx = neighborVarIdxs[randomIdx];
      Value delta = neighborDeltas[randomIdx];
      neighborVarIdxs[randomIdx] = neighborVarIdxs[bmsIdx];
      neighborDeltas[randomIdx] = neighborDeltas[bmsIdx];
      neighborVarIdxs[bmsIdx] = varIdx;
      neighborDeltas[bmsIdx] = delta;
    }
  }

  // 第四部分：评估候选变量并选择最佳移动
  for (size_t idx = 0; idx < scoreSize; ++idx)
  {
    size_t varIdx = neighborVarIdxs[idx];
    Value delta = neighborDeltas[idx];
    auto &localVar = localVarUtil.GetVar(varIdx);
    auto &modelVar = modelVarUtil->GetVar(varIdx);

    // 处理二进制变量：避免重复评分
    if (modelVar.type == VarType::Binary)
    {
      if (scoreTable[varIdx])
        continue;
      else
      {
        scoreTable[varIdx] = true;
        scoreIdxs.push_back(varIdx);
      }
    }

    // 计算移动的得分
    long score = TightScore(modelVar, delta);
    if (bestScore < score ||
        bestScore == score && bestSubscore < subscore)
    {
      bestScore = score;
      bestVarIdx = varIdx;
      bestDelta = delta;
      bestSubscore = subscore;
    }
  }

  // 第五部分：应用最佳移动（如果找到）
  if (bestScore > 0)
  {
    if (DEBUG)
      printf("UNSAT: %-10ld; ", bestScore);
    ++tightStepUnsat; // 更新统计信息
    ApplyMove(bestVarIdx, bestDelta); // 应用移动
    PickVar[bestVarIdx]++;
    for (auto idx : scoreIdxs)
      scoreTable[idx] = false; // 重置评分表
    return true;
  }
  else
  {
    // 如果没有找到有效移动，尝试其他移动策略
    bool resFurtherMove = false;
    if (isFoundFeasible)
      resFurtherMove = SatTightMove(scoreTable, scoreIdxs); // 尝试满足约束的移动
    if (!resFurtherMove)
      resFurtherMove = FlipMove(scoreTable, scoreIdxs); // 尝试翻转移动
    for (auto idx : scoreIdxs)
      scoreTable[idx] = false; // 重置评分表
    return resFurtherMove;
  }
}