/*=====================================================================================

    Filename:     SatTightMove.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#include "LocalMIP.h"

bool LocalMIP::SatTightMove(
    vector<bool> &score_table, // 记录变量是否已被评分的表
    vector<size_t> &score_idx)  // 记录已评分变量的索引
{
  // 如果约束数量不足，直接返回 false
  if (modelConUtil->conNum <= 1)
    return false;

  // 初始化变量以跟踪最佳移动
  long bestScore = 0; // 最佳得分
  long bestSubscore = -std::numeric_limits<long>::max(); // 次佳得分
  size_t bestVarIdx = -1; // 最佳变量的索引
  Value bestDelta = 0;    // 最佳变量的变化值

  // 使用临时向量存储候选变量及其变化值
  vector<size_t> &neighborVarIdxs = localVarUtil.tempVarIdxs;
  vector<Value> &neighborDeltas = localVarUtil.tempDeltas;
  neighborVarIdxs.clear();
  neighborDeltas.clear();

  // 使用临时向量存储满足约束的索引
  auto &neighborConIdxs = localConUtil.tempSatConIdxs;
  neighborConIdxs.clear();
  localConUtil.sampleSet.clear(); // 清空采样集合

  // 第一部分：随机采样满足的约束
  for (size_t time = 0; time < sampleSat; time++)
  {
    size_t conIdx = mt() % (modelConUtil->conNum - 1) + 1; // 随机选择一个约束（跳过目标函数）
    if (localConUtil.sampleSet.find(conIdx) == localConUtil.sampleSet.end() && // 避免重复采样
        localConUtil.conSet[conIdx].SAT() && // 约束必须满足
        !modelConUtil->conSet[conIdx].inferSAT) // 约束不能是推断满足的
    {
      localConUtil.sampleSet.insert(conIdx); // 记录已采样的约束
      neighborConIdxs.push_back(conIdx); // 添加到候选约束列表
    }
  }

  // 第二部分：遍历候选约束，收集变量及其变化值
  size_t neighborSize = neighborConIdxs.size();
  for (size_t neighborIdx = 0; neighborIdx < neighborSize; ++neighborIdx)
  {
    auto &localCon = localConUtil.conSet[neighborConIdxs[neighborIdx]];
    auto &modelCon = modelConUtil->conSet[neighborConIdxs[neighborIdx]];

    // 遍历约束中的每一项
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
    {
      size_t varIdx = modelCon.varIdxSet[termIdx];
      auto &localVar = localVarUtil.GetVar(varIdx);
      auto &modelVar = modelVarUtil->GetVar(varIdx);

      // 计算使约束更紧的变化值 delta
      Value delta;
      if (!TightDelta(localCon, modelCon, termIdx, delta))
        if (modelCon.coeffSet[termIdx] > 0)
          delta = modelVar.upperBound - localVar.nowValue;
        else
          delta = modelVar.lowerBound - localVar.nowValue;

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

  // 第三部分：如果候选变量过多，使用 BMS（束搜索）随机选择部分候选
  size_t scoreSize = neighborVarIdxs.size();
  if (neighborVarIdxs.size() > bmsSat)
  {
    scoreSize = bmsSat;
    for (size_t bmsIdx = 0; bmsIdx < bmsSat; ++bmsIdx)
    {
      size_t randomIdx = (mt() % (neighborVarIdxs.size() - bmsIdx)) + bmsIdx;
      // 将随机选择的候选交换到前面
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
      if (score_table[varIdx])
        continue; // 已评分，跳过
      else
      {
        score_table[varIdx] = true; // 标记为已评分
        score_idx.push_back(varIdx); // 记录评分变量的索引
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
      printf("SAT: %-12ld; ", bestScore);
    ++tightStepSat; // 更新统计信息
    ApplyMove(bestVarIdx, bestDelta); // 应用移动
    PickVar[bestVarIdx]++;
    return true;
  }

  return false; // 未找到有效移动
}