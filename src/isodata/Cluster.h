//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_CLUSTER_H
#define ISODATA_CLUSTER_H

#include <Eigen/Core>
#include <unordered_set>
#include "common.h"

struct Cluster{
  double innerMeanDis; // mean distance
  Eigen::ArrayXd sigma; // variance
  static double allMeanDis; // global mean distance
  Eigen::ArrayXd center; // barycenter
  std::unordered_set<unsigned> ids; // ids of the pointsin the cluster (ids from isodata)
  Cluster(): center{}, innerMeanDis(0), sigma(Eigen::ArrayXd{}){}
  explicit Cluster(Eigen::ArrayXd c):
  center(c), innerMeanDis(0), sigma(Eigen::ArrayXd::Zero(c.size())) {}
  void add_point(int p_index);
  void clear_ids();
};

struct ResultCluster{
  Eigen::VectorXd centroid;
  Eigen::MatrixXd points;
};


#endif //ISODATA_CLUSTER_H
