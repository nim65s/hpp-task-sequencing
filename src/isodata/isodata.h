//
// Created by Jeff on 2019/1/2 0002.
//

#ifndef ISODATA_ISODATA_H
#define ISODATA_ISODATA_H

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <time.h>
#include <random>
#include "error.h"
#include "Cluster.h"
#include <deque>
#include <functional>
#include "common.h"
#include <fstream>

using namespace std;
// implementation of the ISODATA clustering algorithm

typedef std::vector<std::vector<double>> matrix_t;
class isodata {
private:
  typedef matrix_t (*READFUNC)(const char*);
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
  matrix_t data; // data to be classified
  deque<Cluster> clusters;
  READFUNC read_func; // custom function reading the data
  const char* dataFile; // file containing the points to cluster
  const char* paramFile; // file containing the wished values for the parameters
  const char* resultFile; // file where the obtained clusters will be written
  double alpha; // splitting factor
public:
  /**
   * Constructor
   * @param f function to read the data. returns a 2D vector
   * @param dataPath file to read the data from
   * @param paramPath file to read the parameters from
   * @param resultPath file to write the results in
   */
  explicit isodata(matrix_t(f)(const char*), const char* dataPath, const char* paramPath, const char* resultPath) :
    row(0), col(0), clusters(), dataFile(dataPath), paramFile(paramPath), resultFile(resultPath), alpha(0.3)
  {
    read_func = f;
  }

  explicit isodata(matrix_t points, unsigned c, unsigned nc, unsigned tn, double te, double tc, unsigned nt, unsigned ns, const char* resultPath) :
    _c(c), _nc(nc), _tn(tn), _te(te), _tc(tc), _nt(nt), _ns(ns), data(points), resultFile(resultPath)
  {
    row = data.size();
    col = data[0].size();
  }



  void runWFiles()
  {
    setParam();
    setData();
    init_clusters();
    for (int iter = 0; iter < _ns; ++iter) {
      re_assign();
      check_tn();
      update_centers();
      update_meandis();
      switch_method(iter);
    }
    output();
  }

  void runWPoints()
  {
    init_clusters();
    for (int iter = 0; iter < _ns; ++iter) {
      re_assign();
      check_tn();
      update_centers();
      update_meandis();
      switch_method(iter);
    }
    output();
  }

private:
  void setParam();
  void setData();
  void init_clusters();
  pair<int, double> get_nearest_cluster(int p_index, int ignore);
  pair<int, double> get_nearest_cluster(int p_index, unordered_set<unsigned>&);
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
  void output() const;
};


#endif //ISODATA_ISODATA_H

