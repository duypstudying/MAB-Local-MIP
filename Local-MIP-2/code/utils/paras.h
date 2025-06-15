#ifndef _paras_hpp_INCLUDED
#define _paras_hpp_INCLUDED

#include <cstring>
#include <string>
#include <unordered_map>
#include "header.h"

// 数值参数宏定义
// 格式: PARA(参数名, 类型, 短选项, 是否必填, 默认值, 最小值, 最大值, 描述)
#define PARAS \
    PARA( cutoff        , double, '\0' , false , 7200       , 0  , 1e8      , "Cutoff time") \
    PARA( PrintSol      , int   , '\0' , false , 1          , 0  , 1        , "Print best found solution or not")\
    PARA( DEBUG         , int   , '\0' , false , 0          , 0  , 1        , "")

// 字符串参数宏定义
// 格式: STR_PARA(参数名, 短选项, 是否必填, 默认值, 描述)
#define STR_PARAS \
    STR_PARA( instance   , 'i'   ,  true    , "" , ".mps format instance")\
    STR_PARA( log       , 'l'  ,  false  , "./result.csv", "log file")

struct paras {
    // 展开 PARAS 宏，生成数值类型成员变量
    #define PARA(N, T, S, M, D, L, H, C) \
        T N = D;
        PARAS
    #undef PARA

    // 展开 STR_PARAS 宏，生成字符串类型成员变量
    #define STR_PARA(N, S, M, D, C) \
        std::string N = D;
        STR_PARAS
    #undef STR_PARA

    void parse_args(int argc, char *argv[]);  // 解析命令行参数
    void print_change();                     // 打印参数信息
    Value identify_opt(const char *file);    // 识别参数
};

#define INIT_ARGS __global_paras.parse_args(argc, argv);

extern paras __global_paras;

// 快捷访问全局参数的宏（如 OPT(cutoff) 等价于 __global_paras.cutoff）
#define OPT(N) (__global_paras.N)

#endif