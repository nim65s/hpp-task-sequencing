//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_COMMON_H
#define ISODATA_COMMON_H

#include <vector>
#include <iostream>
#include <cmath>
#include "error.h"

// task distance
inline double get_distance(const Eigen::ArrayXd& p1, const Eigen::ArrayXd& p2, const double k_task)
{
  const double pi = 3.141592653589793;
    // size check of the configurations
  if (p1.size() != 7 || p2.size() != 7) {
    std::cout << WARN_VECTOR_SIZE << std::endl;
    return -1;
  }
  else
    {
      // squared L2 distance between the origins
      Eigen::Vector3d p1h = p1.head(3);
      Eigen::Vector3d p2h = p2.head(3);
      double originDist = (p2h-p1h).squaredNorm();
      // quaternion product
      Eigen::Vector4d prod;
      prod << p1[3]*p2[3] - p1[4]*p2[4] - p1[5]*p2[5] - p1[6]*p2[6],
	p1[3]*p2[4] + p1[4]*p2[3] + p1[5]*p2[6] - p1[6]*p2[5],
	p1[3]*p2[5] - p1[4]*p2[6] + p1[5]*p2[3] + p1[6]*p2[4],
	p1[3]*p2[6] + p1[4]*p2[5] - p1[5]*p2[4] + p1[6]*p2[3];
      // quaternion distance
      double scalarPart = prod[0];
      double vectorPartNorm = sqrt(prod[1]*prod[1] + prod[2]*prod[2] + prod[3]*prod[3]);
      double quatDist = 2*std::atan2(vectorPartNorm, scalarPart);
      if (quatDist>pi) quatDist = 2*pi-quatDist;
      quatDist = quatDist/pi;
      // task distance
      double res = sqrt(originDist*originDist + k_task*quatDist*quatDist);
      return res;
    }
}

#endif //ISODATA_COMMON_H
