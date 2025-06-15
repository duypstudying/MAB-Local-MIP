/*=====================================================================================

    Filename:     parse.cpp

    Description:
        Version:  1.0

    Author:       Peng Lin, penglincs@outlook.com

    Organization: Shaowei Cai Group,
                  State Key Laboratory of Computer Science,
                  Institute of Software, Chinese Academy of Sciences,
                  Beijing, China

=====================================================================================*/

#include <cstring>
#include <fstream>
#include "header.h"
#include "paras.h"

// 从文件路径中提取文件名（不含路径）
Value paras::identify_opt(const char *file) {
    // 分配缓冲区：文件名长度 + 终止符
    char name[strlen(file) + 1];
    int p = -1;  // 最后一个'/'的位置
    int l = strlen(file);

    // 逆向查找最后一个'/'的位置
    for (int i = l - 1; i >= 0; i--) {
        if (file[i] == '/') {
            p = i;
            break;
        }
    }

    // 复制文件名部分（跳过路径）
    strncpy(name, file + p + 1, l - p - 1);
    name[l - p - 1] = '\0';  // 确保终止符

    // 调试输出（原路径和提取的文件名）
    printf("c File name (with path): %s\n", file);
    printf("c File name: %s\n", name);
    FILE *fp;
    fp=fopen((char *)OPT(log).c_str(),"a");
    fprintf(fp,"%s,",name);
    fclose(fp);
    return NegativeInfinity;  // 固定返回值（可能用于特殊逻辑）
}
