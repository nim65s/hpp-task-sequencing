//
// Created by Jeff on 2019/1/6 0006.
//

#ifndef ISODATA_CLUSTER_H
#define ISODATA_CLUSTER_H

#include <vector>
#include <unordered_set>

using namespace std;

struct Cluster{
    double innerMeanDis; // mean distance
    vector<double> sigma; // variance
    static double allMeanDis; // global mean distance
    vector<double> center; // barycenter
  unordered_set<unsigned> ids; // ids of the pointsin the cluster (ids from isodata)
    Cluster():
      innerMeanDis(0), sigma(vector<double>{}), center{}{}
    explicit Cluster(vector<double> &c):
      innerMeanDis(0), sigma(vector<double>(c.size(), 0)), center(c) {}
    void add_point(int p_index);
    void clear_ids();
};


#endif //ISODATA_CLUSTER_H
