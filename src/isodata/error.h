// #pragma clang diagnostic push
// #pragma ide diagnostic ignored "cert-err58-cpp"
//
// Created by Jeff on 2019/1/4 0004.
//

#ifndef ISODATA_ERROR_H
#define ISODATA_ERROR_H

#include <string>

// to gain time, only warnings are given and the execution is not interrupted
const std::string WARN_DATA_SIZE("Data size error");
const std::string WARN_VECTOR_SIZE("Vector size error");
const std::string WARN_FILE_OPEN_FAIL("Fail to open file");
const std::string WARN_POINT_REPEAT("Index repeat");
const std::string WARN_CLUSTER_SIZE_SMALL("Cluster size too small");
#endif //ISODATA_ERROR_H

// #pragma clang diagnostic pop
