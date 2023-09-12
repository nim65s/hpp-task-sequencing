//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_CLUSTER_H
#define ISODATA_CLUSTER_H

#include <Eigen/Core>
#include <unordered_set>
#include "common.h"

struct Cluster{
  double innerMeanDis; // mean distance (in the cluster)
  Eigen::ArrayXd sigma; // variance (in the cluster)
  static double allMeanDis; // global mean distance
  Eigen::ArrayXd center; // centroid
  std::unordered_set<unsigned> ids; // ids of the points in the cluster (ids from isodata)
  // constructors
  explicit Cluster(): innerMeanDis(0), sigma(Eigen::ArrayXd{}), center{} {}
  explicit Cluster(Eigen::ArrayXd c): innerMeanDis(0), sigma(Eigen::ArrayXd::Zero(c.size())), center(c) {}
  // methods
  void add_point(int p_index);
  void clear_ids();
};

// structure for the result of ISODATA
struct ResultCluster{
  Eigen::VectorXd centroid;
  Eigen::MatrixXd points;
};


#endif //ISODATA_CLUSTER_H
