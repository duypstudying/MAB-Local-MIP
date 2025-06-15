/*=====================================================================================

    Filename:     TightOperator.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#include "LocalMIP.h"

// 计算当前变量的调整对目标函数和约束的评分
long LocalMIP::TightScore(
    const ModelVar &_modelVar, // 模型变量
    Value _delta)              // 调整量
{
  long score = 0;              // 总评分
  size_t conIdx;               // 约束索引
  size_t posInCon;             // 约束中的位置
  Value newLHS;                // 调整后的约束左侧值
  Value newOBJ;                // 调整后的目标函数值
  bool isPreSat;               // 调整前是否满足约束
  bool isNowSat;               // 调整后是否满足约束
  bool isPreStable;            // 调整前是否稳定（严格满足约束）
  bool isNowStable;            // 调整后是否稳定
  bool isPreBetter;            // 调整前目标函数是否更优
  bool isNowBetter;            // 调整后目标函数是否更优
  subscore = 0;                // 子评分（用于额外评分）

  // 遍历模型变量的所有项
  for (size_t termIdx = 0; termIdx < _modelVar.termNum; ++termIdx)
  {
    conIdx = _modelVar.conIdxSet[termIdx];
    posInCon = _modelVar.posInCon[termIdx];
    auto &localCon = localConUtil.conSet[conIdx];
    auto &modelCon = modelConUtil->conSet[conIdx];

    if (conIdx == 0) // 如果是目标函数
    {
      if (isFoundFeasible) // 如果已经找到可行解
      {
        newOBJ = localCon.LHS + modelCon.coeffSet[posInCon] * _delta;
        // 判断目标函数是否更优
        if (newOBJ < localCon.LHS)
          score += localCon.weight; // 更优则加分
        else
          score -= localCon.weight; // 否则减分

        isPreBetter = localCon.LHS < localCon.RHS;
        isNowBetter = newOBJ < localCon.RHS;
        // 更新子评分
        if (!isPreBetter && isNowBetter)
          subscore += localCon.weight;
        else if (isPreBetter && !isNowBetter)
          subscore -= localCon.weight;
      }
    }
    else // 如果是普通约束
    {
      newLHS = localCon.LHS + modelCon.coeffSet[posInCon] * _delta;
      isPreSat = localCon.SAT(); // 调整前是否满足约束
      isNowSat = newLHS < localCon.RHS + FeasibilityTol; // 调整后是否满足约束
      // 更新评分
      if (!isPreSat && isNowSat)
        score += localCon.weight; // 从不满足到满足，加分
      else if (isPreSat && !isNowSat)
        score -= localCon.weight; // 从满足到不满足，减分
      else if (!isPreSat && !isNowSat)
        if (localCon.LHS > newLHS)
          score += localCon.weight >> 1; // 更接近满足约束，加分（权重减半）
        else
          score -= localCon.weight >> 1; // 更远离满足约束，减分（权重减半）

      isPreStable = localCon.LHS < localCon.RHS - FeasibilityTol;
      isNowStable = newLHS < localCon.RHS - FeasibilityTol;
      // 更新子评分
      if (!isPreStable && isNowStable)
        subscore += localCon.weight;
      else if (isPreStable && !isNowStable)
        subscore -= localCon.weight;
    }
  }
  return score; // 返回总评分
}

// 计算调整量 delta_x，使得 a * delta_x + gap <= 0
bool LocalMIP::TightDelta(
    LocalCon &_localCon,    // 局部约束
    const ModelCon &_modelCon, // 模型约束
    size_t _termIdx,        // 项索引
    Value &_res)            // 返回的调整量
{
  Value gap = _localCon.LHS - _localCon.RHS; // 当前约束的间隙
  auto varIdx = _modelCon.varIdxSet[_termIdx];
  auto &localVar = localVarUtil.GetVar(varIdx);
  auto &modelVar = modelVarUtil->GetVar(varIdx);
  Value delta = -(gap / _modelCon.coeffSet[_termIdx]); // 计算理论调整量

  // 根据系数符号和变量类型调整 delta
  if (_modelCon.coeffSet[_termIdx] > 0)
  {
    if (modelVar.type == VarType::Real)
      _res = delta; // 实数变量直接使用 delta
    else
      _res = floor(delta); // 整数变量向下取整
  }
  else
  {
    if (modelVar.type == VarType::Real)
      _res = delta; // 实数变量直接使用 delta
    else
      _res = ceil(delta); // 整数变量向上取整
  }

  // 检查调整后的值是否在变量边界内
  return modelVar.InBound(localVar.nowValue + _res);
}

// 更新权重：对不满足的约束增加权重，如果所有约束满足且找到可行解，则增加目标函数权重
void LocalMIP::UpdateWeight()
{
  for (size_t conIdx : localConUtil.unsatConIdxs)
  {
    auto &localCon = localConUtil.conSet[conIdx];
    ++localCon.weight; // 不满足的约束权重加 1
  }
  auto &localObj = localConUtil.conSet[0]; // 目标函数
  if (isFoundFeasible && localConUtil.unsatConIdxs.empty())
    ++localObj.weight; // 如果找到可行解且所有约束满足，目标函数权重加 1
}

// 平滑权重：对满足的约束且权重大于 0 的，权重减 1
void LocalMIP::SmoothWeight()
{
  for (auto &localCon : localConUtil.conSet)
    if (localCon.SAT() && localCon.weight > 0)
      --localCon.weight; // 满足的约束权重减 1
}