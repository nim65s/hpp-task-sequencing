//
// Created by Jeff on 2019/1/6 0006.
//

#include "Cluster.h"

double Cluster::allMeanDis = 0;

// adds point p_index to the cluster
void Cluster::add_point(int p_index) {
  if (ids.find(static_cast<const unsigned int &>(p_index)) != ids.end())
    {
      std::cout << WARN_POINT_REPEAT << std::endl;
      return;
    }
  ids.emplace(p_index);
}

// to empty the the cluster
void Cluster::clear_ids() {
  ids.clear();
}
