/*=====================================================================================

    Filename:     header.h

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#pragma once
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <unordered_set>
#include <random>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <fstream>
#include <sys/time.h>
#include <stdlib.h>
#include <chrono>
using namespace std;

// 数值类型别名（默认为 double）
using Value = double;

// 数值极限常量
const Value Infinity = 1e20;                  // 正无穷
const Value NegativeInfinity = -Infinity;     // 负无穷
const Value DefaultIntegerUpperBound = 1.0;    // 整数变量默认上界
const Value DefaultRealUpperBound = Infinity;  // 实数变量默认上界
const Value DefaultLowerBound = 0.0;           // 默认下界
const Value InfiniteUpperBound = Infinity;     // 无限上界（别名）
const Value InfiniteLowerBound = NegativeInfinity;  // 无限下界（别名）

// 优化问题容差
const Value FeasibilityTol = 1e-6;  // 可行性检查容差
const Value OptimalTol = 1e-4;      // 最优解判定容差

// 变量类型枚举（用于数学优化）
enum class VarType {
    Binary,   // 0或1
    Integer,  // 整数
    Real,     // 实数
    Fixed     // 固定值
};

// 时间工具函数
std::chrono::_V2::system_clock::time_point TimeNow();  // 获取当前时间
double ElapsedTime(const std::chrono::_V2::system_clock::time_point &a,
                   const std::chrono::_V2::system_clock::time_point &b);  // 计算时间差

// 字符串工具函数
bool IsBlank(const string &a);       // 检查字符串是否为空/空白
void PrintfError(const string &a);   // 打印错误信息
