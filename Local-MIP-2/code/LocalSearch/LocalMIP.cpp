/*=====================================================================================

    Filename:     LocalMIP.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/
#include "LocalMIP.h"

#include <memory>

int LocalMIP::LocalSearch(
    Value _optimalObj,
    chrono::_V2::system_clock::time_point _clkStart)
{
  Allocate();          // 分配内存和初始化数据结构
  InitSolution();      // 初始化解
  InitState();         // 初始化约束状态
  auto &localObj = localConUtil.conSet[0];
  curStep = 0;
  time_t start,stop;
  start=time(NULL);
  while (true)
  {
    if (DEBUG)
      printf("\nc UNSAT Size: %-10ld; ", localConUtil.unsatConIdxs.size());

    // 如果所有约束都满足
    if (localConUtil.unsatConIdxs.empty())
    {
      if (!isFoundFeasible || localObj.LHS < localObj.RHS)
      {
        UpdateBestSolution(); // 更新最优解
        LogObj(_clkStart);    // 记录目标函数值
        if (!isFoundFeasible) {
          stop=time(NULL);
          printf("Time: %ld\n",stop-start);
        }
        isFoundFeasible = true;
      }

      bool res = LiftMoveWithoutBreak(); // 尝试提升移动
      if (GetObjValue() <= _optimalObj) // 如果达到最优解
        return 1;

      ++curStep;
      if (Timeout(_clkStart)) // 检查超时
        break;
      if (res)
        continue;
    }

    if (Timeout(_clkStart))
      break;

    // 尝试不满足约束的紧致移动
    if (!UnsatTightMove())
    {
      // 根据概率更新权重或平滑权重
      if (mt() % 10000 > smoothProbability)
        UpdateWeight();
      else
        SmoothWeight();

      RandomTightMove(); // 随机紧致移动
    }
    ++curStep;
  }
  return 0; // 返回未找到最优解
}

bool LocalMIP::Timeout(
    chrono::_V2::system_clock::time_point &_clkStart)
{
  auto clk_now = chrono::high_resolution_clock::now();
  auto solve_time =
      chrono::duration_cast<chrono::seconds>(clk_now - _clkStart).count();
  if (solve_time >= OPT(cutoff)) // 检查是否超过截止时间
    return true;
  return false;
}

void LocalMIP::LogObj(
    chrono::_V2::system_clock::time_point &_clkStart)
{
  auto clk = TimeNow();
  printf(
      "n %-20f %lf\n",
      (GetObjValue()),
      ElapsedTime(clk, _clkStart)); // 打印目标函数值和运行时间
  RunTime=ElapsedTime(clk, _clkStart);
}

void LocalMIP::InitSolution()
{
  for (size_t varIdx = 0; varIdx < modelVarUtil->varNum; varIdx++)
  {
    auto &localVar = localVarUtil.GetVar(varIdx);
    const auto &modelVar = modelVarUtil->GetVar(varIdx);
    if (modelVar.lowerBound > 0)
      localVar.nowValue = modelVar.lowerBound; // 设置为下界
    else if (modelVar.upperBound < 0)
      localVar.nowValue = modelVar.upperBound; // 设置为上界
    else
      localVar.nowValue = 0; // 默认设为0
    assert(modelVar.InBound(localVar.nowValue)); // 检查边界
  }
}
void save_result(const char *filename, int win ,double time ,double bestobj) {
  FILE *fp;
  fp=fopen(filename,"a");
  if (win)
    fprintf(fp,"%d,%.5lf,%.5lf\n",win,time,bestobj);
  else
    fprintf(fp,"%d,%.5lf,%s\n",win,time,"N/A");
  fclose(fp);
  return;
}
void LocalMIP::PrintResult()
{
  int win=0;
  if (!isFoundFeasible)
    printf("o no feasible solution found.\n");
  else if (VerifySolution())
  {
    win=1;
    printf("o Best objective: %lf\n", GetObjValue());
    // printf("B 1 %lf\n", GetObjValue());
    if (OPT(PrintSol))
      PrintSol();
  }
  else
    cout << "solution verify failed." << endl;
  save_result((char *)OPT(log).c_str(),win,RunTime,bestOBJ);
}

void LocalMIP::InitState()
{
  for (size_t conIdx = 1; conIdx < modelConUtil->conNum; ++conIdx)
  {
    auto &localCon = localConUtil.conSet[conIdx];
    auto &modelCon = modelConUtil->conSet[conIdx];
    localCon.LHS = 0;
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
      localCon.LHS +=
          modelCon.coeffSet[termIdx] *
          localVarUtil.GetVar(modelCon.varIdxSet[termIdx]).nowValue; // 计算约束的左侧值
    if (localCon.UNSAT())
      localConUtil.insertUnsat(conIdx); // 标记不满足的约束
  }

  // 初始化目标函数
  auto &localObj = localConUtil.conSet[0];
  auto &modelObj = modelConUtil->conSet[0];
  localObj.RHS = Infinity;
  localObj.LHS = 0;
  for (size_t termIdx = 0; termIdx < modelObj.termNum; ++termIdx)
    localObj.LHS +=
        modelObj.coeffSet[termIdx] *
        localVarUtil.GetVar(modelObj.varIdxSet[termIdx]).nowValue;
}

void LocalMIP::UpdateBestSolution()
{
  lastImproveStep = curStep;
  for (auto &localVar : localVarUtil.varSet)
    localVar.bestValue = localVar.nowValue; // 保存当前解为最优解
  auto &localObj = localConUtil.conSet[0];
  auto &modelObj = modelConUtil->conSet[0];
  bestOBJ = localObj.LHS;
  localObj.RHS = bestOBJ - OptimalTol; // 更新目标函数的右侧值
}

void LocalMIP::ApplyMove(
    size_t _varIdx,
    Value _delta)
{
  auto &localVar = localVarUtil.GetVar(_varIdx);
  auto &modelVar = modelVarUtil->GetVar(_varIdx);
  localVar.nowValue += _delta; // 更新变量值

  // 更新相关约束的状态
  for (size_t termIdx = 0; termIdx < modelVar.termNum; ++termIdx)
  {
    size_t conIdx = modelVar.conIdxSet[termIdx];
    size_t posInCon = modelVar.posInCon[termIdx];
    auto &localCon = localConUtil.conSet[conIdx];
    auto &modelCon = modelConUtil->conSet[conIdx];
    Value newLHS = 0;
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
      newLHS +=
          modelCon.coeffSet[termIdx] *
          localVarUtil.GetVar(modelCon.varIdxSet[termIdx]).nowValue;

    if (conIdx == 0)
      localCon.LHS = newLHS; // 更新目标函数
    else
    {
      bool isPreSat = localCon.SAT();
      bool isNowSat = newLHS < localCon.RHS + FeasibilityTol;
      if (isPreSat && !isNowSat)
        localConUtil.insertUnsat(conIdx); // 标记不满足的约束
      else if (!isPreSat && isNowSat)
        localConUtil.RemoveUnsat(conIdx); // 移除满足的约束
      localCon.LHS = newLHS;

    }
  }

  // 更新禁忌表
  if (_delta > 0)
  {
    localVar.lastIncStep = curStep;
    localVar.allowDecStep =
        curStep + tabuBase + mt() % tabuVariation;
  }
  else
  {
    localVar.lastDecStep = curStep;
    localVar.allowIncStep =
        curStep + tabuBase + mt() % tabuVariation;
  }
}

void LocalMIP::Restart()
{
  lastImproveStep = curStep;
  ++restartTimes;
  localConUtil.unsatConIdxs.clear(); // 清空不满足的约束

  // 随机重置变量值
  for (size_t varIdx = 0; varIdx < modelVarUtil->varNum; varIdx++)
  {
    auto &localVar = localVarUtil.GetVar(varIdx);
    auto &modelVar = modelVarUtil->GetVar(varIdx);
    if (modelVar.type == VarType::Binary)
      localVar.nowValue = mt() % 2; // 二进制变量随机赋值
    else if (modelVar.type == VarType::Integer &&
             modelVar.lowerBound > -1e15 &&
             modelVar.upperBound < 1e15)
    {
      long long lowerBound = (long long)modelVar.lowerBound;
      long long upperBound = (long long)modelVar.upperBound;
      localVar.nowValue = modelVar.lowerBound + (mt() % (upperBound + 1 - lowerBound)); // 整数变量随机赋值
    }
    else
    {
      if (modelVar.lowerBound > 0)
        localVar.nowValue = modelVar.lowerBound;
      else if (modelVar.upperBound < 0)
        localVar.nowValue = modelVar.upperBound;
      else
        localVar.nowValue = 0;
    }
    assert(modelVar.InBound(localVar.nowValue));

    // 50%概率恢复为最优解
    if (isFoundFeasible && mt() % 100 > 50)
      localVar.nowValue = localVar.bestValue;

    // 重置禁忌表
    localVar.lastDecStep = curStep;
    localVar.allowIncStep = 0;
    localVar.lastIncStep = curStep;
    localVar.allowDecStep = 0;
  }

  // 重新初始化约束状态
  for (size_t conIdx = 1; conIdx < modelConUtil->conNum; ++conIdx)
  {
    auto &localCon = localConUtil.conSet[conIdx];
    auto &modelCon = modelConUtil->conSet[conIdx];
    localCon.LHS = 0;
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
      localCon.LHS +=
          modelCon.coeffSet[termIdx] *
          localVarUtil.GetVar(modelCon.varIdxSet[termIdx]).nowValue;
    if (localCon.UNSAT())
      localConUtil.insertUnsat(conIdx);
    localCon.weight = 1; // 重置权重
  }

  // 重新初始化目标函数
  auto &localObj = localConUtil.conSet[0];
  auto &modelObj = modelConUtil->conSet[0];
  localObj.LHS = 0;
  localObj.weight = 1;
  for (size_t termIdx = 0; termIdx < modelObj.termNum; ++termIdx)
    localObj.LHS +=
        modelObj.coeffSet[termIdx] *
        localVarUtil.GetVar(modelObj.varIdxSet[termIdx]).nowValue;
}

bool LocalMIP::VerifySolution()
{
  // 检查变量边界
  for (size_t var_idx = 0; var_idx < modelVarUtil->varNum; var_idx++)
  {
    auto &var = localVarUtil.GetVar(var_idx);
    auto &modelVar = modelVarUtil->GetVar(var_idx);
    if (!modelVar.InBound(var.bestValue))
      return false;
  }

  // 检查约束是否满足
  for (size_t conIdx = 1; conIdx < modelConUtil->conNum; ++conIdx)
  {
    auto &con = localConUtil.conSet[conIdx];
    auto &modelCon = modelConUtil->conSet[conIdx];
    Value lhs = 0;
    for (size_t termIdx = 0; termIdx < modelCon.termNum; ++termIdx)
      lhs +=
          modelCon.coeffSet[termIdx] *
          localVarUtil.GetVar(modelCon.varIdxSet[termIdx]).bestValue;
    if (lhs > modelCon.RHS + FeasibilityTol)
    {
      printf("c lhs: %lf; rhs: %lf\n", lhs, modelCon.RHS);
      return false;
    }
  }

  // 检查目标函数值
  auto &localObj = localConUtil.conSet[0];
  auto &modelObj = modelConUtil->conSet[0];
  Value objValue = 0;
  for (size_t termIdx = 0; termIdx < modelObj.termNum; ++termIdx)
    objValue +=
        modelObj.coeffSet[termIdx] *
        localVarUtil.GetVar(modelObj.varIdxSet[termIdx]).bestValue;
  return fabs(objValue - bestOBJ) < 1e-3; // 允许微小误差
}

void LocalMIP::PrintSol()
{
  printf("c best-found solution:\n");
  printf("%-50s        %s\n", "Variable name", "Variable value");
  for (size_t varIdx = 0; varIdx < modelVarUtil->varNum; varIdx++)
  {
    const auto &var = localVarUtil.GetVar(varIdx);
    const auto &modelVar = modelVarUtil->GetVar(varIdx);
    if (var.bestValue)
      printf("%-50s        %lf\n", modelVar.name.c_str(), var.bestValue); // 打印变量名和值
  }
}

void LocalMIP::Allocate()
{
  liftStep = 0;
  breakStep = 0;
  tightStepUnsat = 0;
  tightStepSat = 0;
  flipStep = 0;
  randomStep = 0;
  restartTimes = 0;
  smoothProbability = 3;
  tabuBase = 3;
  tabuVariation = 10;
  isBin = modelVarUtil->isBin;
  isKeepFeas = false;
  isFoundFeasible = false;
  weightUpperBound = 10000000;
  objWeightUpperBound = 10000000;
  lastImproveStep = 0;
  sampleUnsat = 12;
  bmsUnsatInfeas = 2000;
  bmsUnsatFeas = 3000;
  sampleSat = 20;
  bmsSat = 190;
  RunTime=-1;
  bmsFlip = 20;
  printf("%ld\n",modelVarUtil->varNum);
  for (size_t VarIdx = 0; VarIdx < modelVarUtil->varNum; VarIdx++) {
    PickVar.push_back(0);
    VarValue.push_back(1);
    vector<double> a;
    Varindex.push_back(0);
    for (size_t i = 0; i < 5; i++) {
      a.push_back(0);
    }
    reward.push_back(a);
  }
  bmsRandom = 150;
  bestOBJ = Infinity;
  localVarUtil.Allocate(
      modelVarUtil->varNum,
      modelConUtil->conSet[0].varIdxSet.size());
  localConUtil.Allocate(modelConUtil->conNum);
  for (size_t conIdx = 1; conIdx < modelConUtil->conNum; conIdx++)
    localConUtil.conSet[conIdx].RHS = modelConUtil->conSet[conIdx].RHS;
  for (size_t varIdx = 0; varIdx < modelVarUtil->varNum; varIdx++)
  {
    auto &modelVar = modelVarUtil->GetVar(varIdx);
    if (modelVar.type == VarType::Binary)
      localVarUtil.binaryIdx.push_back(varIdx);
  }
  mt.seed(2832);
}

Value LocalMIP::GetObjValue()
{
  return modelConUtil->MIN * (bestOBJ + modelVarUtil->objBias);
}

LocalMIP::LocalMIP(
    const ModelConUtil *_modelConUtil,
    const ModelVarUtil *_modelVarUtil)
    : modelConUtil(_modelConUtil),
      modelVarUtil(_modelVarUtil)
{
  // set running parameter
  DEBUG = OPT(DEBUG);
}

LocalMIP::~LocalMIP()
{
}