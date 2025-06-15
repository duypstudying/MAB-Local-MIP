#include <memory>

#include "LocalMIP.h"

void LocalMIP::RandomTightMove()
{
  // 初始化变量以跟踪最佳移动
  long bestScore = -100000000000; // 最佳得分（初始化为极小值）
  long bestSubscore = -std::numeric_limits<long>::max(); // 次佳得分（用于同分比较）
  size_t bestVarIdx = -1; // 最佳变量的索引（-1表示未找到）
  Value bestDelta = 0;    // 最佳变量的变化值（0表示无有效移动）

  // 使用临时向量存储候选变量及其变化值（复用工具类的临时存储避免重复内存分配）
  vector<size_t> &neighborVarIdxs = localVarUtil.tempVarIdxs;
  vector<Value> &neighborDeltas = localVarUtil.tempDeltas;
  neighborVarIdxs.clear(); // 清空旧候选
  neighborDeltas.clear();  // 清空旧变化值

  // 第一部分：处理不满足约束的变量（优先修复违反的约束）
  if (localConUtil.unsatConIdxs.size() > 0) // 存在未满足的约束时
  {
    // 随机选择一个不满足的约束（避免固定顺序导致的局部最优）
    size_t conIdx = localConUtil.unsatConIdxs[mt() % localConUtil.unsatConIdxs.size()];
    auto &localCon = localConUtil.conSet[conIdx];   // 本地约束状态
    auto &modelCon = modelConUtil->conSet[conIdx]; // 模型约束定义

    // 遍历约束中的每一项（每个变量项）
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
    {
      size_t varIdx = modelCon.varIdxSet[termIdx]; // 当前项对应的变量索引
      auto &localVar = localVarUtil.GetVar(varIdx); // 变量当前状态
      auto &modelVar = modelVarUtil->GetVar(varIdx); // 变量模型定义

      // 计算使约束更紧的变化值 delta（尝试收紧约束边界）
      Value delta;
      if (!TightDelta(localCon, modelCon, termIdx, delta)) // 若TightDelta计算失败（可能边界已紧）
      {
        // 备用方案：将delta设置为变量边界值（根据系数符号选择上下界）
        if (modelCon.coeffSet[termIdx] > 0)
          delta = modelVar.lowerBound - localVar.nowValue; // 正系数时向下收紧
        else
          delta = modelVar.upperBound - localVar.nowValue; // 负系数时向上收紧
      }

      // 过滤无效移动：
      // 1. 违反禁忌条件（避免最近刚调整过的变量重复移动）
      // 2. 变化量过小（小于可行性容忍度视为无意义移动）
      if (delta < 0 && curStep == localVar.lastIncStep + 1 || // 向下调整但处于增量禁忌期
          delta > 0 && curStep == localVar.lastDecStep + 1)   // 向上调整但处于减量禁忌期
        continue;
      if (fabs(delta) < FeasibilityTol) // 变化量过小
        continue;

      // 将有效的移动添加到候选列表（记录变量索引和变化值）
      neighborVarIdxs.push_back(varIdx);
      neighborDeltas.push_back(delta);
    }
  }

  // 第二部分：如果找到可行解，处理目标函数的移动（优化目标值）
  auto &localObj = localConUtil.conSet[0]; // 假设第一个约束是目标函数
  auto &modelObj = modelConUtil->conSet[0];
  if (isFoundFeasible && localObj.UNSAT()) // 已找到可行解但目标未满足时
  {
    for (size_t termIdx = 0; termIdx < modelObj.termNum; ++termIdx) // 遍历目标函数中的变量项
    {
      size_t varIdx = modelObj.varIdxSet[termIdx];
      auto &localVar = localVarUtil.GetVar(varIdx);
      auto &modelVar = modelVarUtil->GetVar(varIdx);

      Value delta;
      if (!TightDelta(localObj, modelObj, termIdx, delta)) // 计算使目标更优的变化值
        // 备用方案：根据目标系数方向设置边界（最大化目标则向有利方向调整）
        if (modelObj.coeffSet[termIdx] > 0)
          delta = modelVar.lowerBound - localVar.nowValue;
        else
          delta = modelVar.upperBound - localVar.nowValue;

      // 过滤无效移动（目标优化场景的禁忌条件：当前步骤小于允许调整的步骤）
      if (delta < 0 && curStep < localVar.allowDecStep ||
          delta > 0 && curStep < localVar.allowIncStep)
        continue;
      if (fabs(delta) < FeasibilityTol) // 变化量过小
        continue;

      neighborVarIdxs.push_back(varIdx);
      neighborDeltas.push_back(delta);
    }
  }

  // 第三部分：如果候选变量过多，随机选择部分候选（控制计算量）
  size_t scoreSize = neighborVarIdxs.size();
  if (neighborVarIdxs.size() > bmsRandom) // 候选数超过设定阈值时
  {
    scoreSize = bmsRandom; // 仅保留前bmsRandom个候选
    if (!isFoundFeasible) {
      for (size_t bmsIdx = 0; bmsIdx < bmsRandom; ++bmsIdx) {
        // 随机选择候选
        size_t randomIdx = (mt() % (neighborVarIdxs.size() - bmsIdx)) + bmsIdx;
        // 交换当前位置与随机位置的候选
        size_t varIdx = neighborVarIdxs[randomIdx];
        Value delta = neighborDeltas[randomIdx];
        neighborVarIdxs[randomIdx] = neighborVarIdxs[bmsIdx];
        neighborDeltas[randomIdx] = neighborDeltas[bmsIdx];
        neighborVarIdxs[bmsIdx] = varIdx;
        neighborDeltas[bmsIdx] = delta;
      }
    }
    else {
      for (size_t bmsIdx = 0; bmsIdx < bmsRandom; ++bmsIdx) {
        double MaxValue=-1e10;
        size_t MaxIdx=0;
        for (size_t idx = bmsIdx; idx < neighborVarIdxs.size(); ++idx) {
          if (VarValue[neighborVarIdxs[idx]]>MaxValue) {
            MaxValue=VarValue[neighborVarIdxs[idx]];
            MaxIdx=idx;
          }
        }
        size_t varIdx = neighborVarIdxs[MaxIdx];
        Value delta = neighborDeltas[MaxIdx];
        neighborVarIdxs[MaxIdx] = neighborVarIdxs[bmsIdx];
        neighborDeltas[MaxIdx] = neighborDeltas[bmsIdx];
        neighborVarIdxs[bmsIdx] = varIdx;
        neighborDeltas[bmsIdx] = delta;
      }
    }
  }

  // 第四部分：评估候选变量并选择最佳移动（基于得分函数）
  for (size_t idx = 0; idx < scoreSize; ++idx)
  {
    size_t varIdx = neighborVarIdxs[idx];
    Value delta = neighborDeltas[idx];
    auto &localVar = localVarUtil.GetVar(varIdx);
    auto &modelVar = modelVarUtil->GetVar(varIdx);

    // 计算移动的得分
    long score = TightScore(modelVar, delta);
    // 更新最佳移动
    if (bestScore < score ||
        bestScore == score && bestSubscore < subscore)
    {
      bestScore = score;
      bestVarIdx = varIdx;
      bestDelta = delta;
      bestSubscore = subscore;
    }
  }
  // bestVarIdx%=reward.size();
  // 第五部分：应用最佳移动（如果找到有效移动）
  if (bestVarIdx != -1 && bestDelta != 0)
  {
    float lastValue=localConUtil.conSet[0].LHS;
    if (DEBUG)
      printf("Radom: %-10ld; ", bestScore);
    ApplyMove(bestVarIdx, bestDelta); // 执行变量调整
    PickVar[bestVarIdx]++;
    ++randomStep; // 随机移动步骤计数
    if (isFoundFeasible) {
      reward[bestVarIdx][Varindex[bestVarIdx]]=((lastValue-localConUtil.conSet[0].LHS)/(lastValue-bestOBJ+1));
      double gamma=pow(0.89,4);
      // VarValue[bestVarIdx]*=(1-1/PickVar[bestVarIdx]);
      // printf("%ld\n",PickVar[bestVarIdx]);
      for (size_t idx = 1; idx <= 5; ++idx) {
        VarValue[bestVarIdx]+=(gamma*reward[bestVarIdx][(idx+Varindex[bestVarIdx])%5]);
        gamma/=0.89;
      }
      Varindex[bestVarIdx]=(++Varindex[bestVarIdx])%5;
    }
    return;
  }
}
