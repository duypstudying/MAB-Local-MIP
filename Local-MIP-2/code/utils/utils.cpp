/*=====================================================================================

Filename:     utils.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/


#include "header.h"  // 包含自定义头文件（可能定义了相关类型）

// 获取当前高精度时间点
std::chrono::_V2::system_clock::time_point TimeNow() {
  return chrono::high_resolution_clock::now();  // 返回纳秒级时间戳
}

// 计算两个时间点之间的秒数差
double ElapsedTime(
    const std::chrono::_V2::system_clock::time_point &a,
    const std::chrono::_V2::system_clock::time_point &b) {
  // 将时间差转换为毫秒后转秒（保留小数）
  return chrono::duration_cast<chrono::milliseconds>(a - b).count() / 1000.0;
}

// 检查字符串是否仅包含空白字符（空格/换行符）
bool IsBlank(const string &a) {
  for (auto x : a)
    if (x != ' ' && x != '\n' && x != '\r')  // 发现非空白字符立即返回false
      return false;
  return true;  // 全部为空白字符
}

// 打印错误信息并终止程序
void PrintfError(const string &a) {
  printf("c error line: %s\n", a.c_str());  // 'c'前缀可能表示调试信息
  exit(-1);  // 非正常退出（错误码-1）#include "header.h"  // 包含自定义头文件（可能定义了相关类型）
}
