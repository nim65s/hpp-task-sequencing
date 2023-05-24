//
// Created by Jeff on 2019/1/2 0002.
//

#ifndef ISODATA_ISODATA_H
#define ISODATA_ISODATA_H

#include <algorithm>
#include <time.h>
#include <random>
#include <deque>
#include <functional>
#include <fstream>
#include <queue>
#include "Cluster.h"

// implementation of the ISODATA clustering algorithm

typedef Eigen::Matrix<ResultCluster, Eigen::Dynamic, 1> ResultingClusters;

class isodata {
private:
  // data for the initialization
  // for the sake of simplicity, parameters can only be set at initialization
  unsigned _c; // expected nb of clusters
  unsigned _nc; // initial nb of clusters
  unsigned _tn; // min nb of points in a cluster
  double _te; // max value of the variance of a cluster
  double _tc; // min distance between two centroids
  unsigned _nt; // max nb of merges at each iteration
  unsigned _ns; // max nb of iterations
  // not initialized in the constructor
  unsigned row; // nb of samples
  unsigned col; // nb of features
  Eigen::MatrixXd data; // data to be classified
  std::deque<Cluster> clusters;
  double alpha; // splitting factor
public:
  /**
   * Constructor
   * @param f function to read the data. returns a 2D vector
   * @param dataPath file to read the data from
   * @param paramPath file to read the parameters from
   * @param resultPath file to write the results in
   */
  explicit isodata(Eigen::MatrixXd points, int NbRows, int NbCols, unsigned c, unsigned nc, unsigned tn, double te, double tc, unsigned nt, unsigned ns) :
    _c(c), _nc(nc), _tn(tn), _te(te), _tc(tc), _nt(nt), _ns(ns), row(NbRows), col(NbCols), data(points), alpha(0.3) {}


  std::vector<ResultCluster> run()
  {
    init_clusters();
    for (int iter = 0; iter < _ns; ++iter) {
      re_assign();
      check_tn();
      update_centers();
      update_meandis();
      switch_method(iter);
    }
    return resultFormatting();
  }

private:
  void init_clusters();
  std::pair<int, double> get_nearest_cluster(int p_index, int ignore);
  std::pair<int, double> get_nearest_cluster(int p_index, std::unordered_set<unsigned>&);
  void re_assign();
  void check_tn();
  void update_centers();
  void update_center(Cluster& cluster);
  void update_sigmas();
  void update_sigma(Cluster& cluster);
  void update_meandis();
  void check_split();
  void split(const int& c_index);
  void check_merge();
  void merge(const int& id1, const int& id2);
  void switch_method(const int& index);
  std::vector<ResultCluster> resultFormatting() const;
};


#endif //ISODATA_ISODATA_H

