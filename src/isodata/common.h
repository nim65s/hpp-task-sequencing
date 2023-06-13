//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_COMMON_H
#define ISODATA_COMMON_H

#include <vector>
#include <iostream>
#include <cmath>
#include "error.h"

// // euclidian distance between points p1 and p2
// inline double get_distance(const Eigen::ArrayXd& p1, const Eigen::ArrayXd& p2)
// {
//   if (p1.size() != p2.size()){
//     std::cout << WARN_VECTOR_SIZE << std::endl;
//     return -1;
//   }
//   else{
//     Eigen::VectorXd diff = p2-p1;
//     return diff.norm();
//   }
// }

// hole distance
inline double get_distance(const Eigen::ArrayXd& p1, const Eigen::ArrayXd& p2) // add 'double k' for hole distance
{
  if (p1.size() != 7 || p2.size() != 7) {
    std::cout << WARN_VECTOR_SIZE << std::endl;
    return -1;
  }
  else
    {
      // l2 distance between the origins
      Eigen::Vector3d p1h = p1.head(3);
      Eigen::Vector3d p2h = p2.head(3);
      double res = (p2h-p1h).norm();
      // squared inner prod of the quaternions
      Eigen::Vector4d p1t = p1.tail(4);
      Eigen::Vector4d p2t = p2.tail(4);
      res = res * (1 + pow(p1t.dot(p2t),2)); // k*pow(p1t.dot(p2t),2) for hole distance
      return res;
    }
}

#endif //ISODATA_COMMON_H
