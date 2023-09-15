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

// structure to store the result of ISODATA
typedef Eigen::Matrix<ResultCluster, Eigen::Dynamic, 1> ResultingClusters;

class isodata {
private:
  // data for the initialization
  // parameters can only be set at initialization
  unsigned _c; // expected nb of clusters
  unsigned _nc; // initial nb of clusters
  unsigned _tn; // min nb of points in a cluster
  double _te; // max value of the variance of a cluster
  double _tc; // min distance between two centroids
  unsigned _nt; // max nb of merges at each iteration
  unsigned _ns; // max nb of iterations
  // not initialized in the constructor
  unsigned rows; // nb of samples
  unsigned cols; // nb of features
  Eigen::MatrixXd data; // data to be classified
  std::deque<Cluster> clusters;
  double k_task; // weight of the angle in the distance measure
  double alpha; // splitting factor
public:
  /**
   * Constructor
   * @param f function to read the data. returns a 2D vector
   * @param dataPath file to read the data from
   * @param paramPath file to read the parameters from
   * @param resultPath file to write the results in
   */
  explicit isodata(Eigen::MatrixXd points, int NbRows, int NbCols, unsigned c, unsigned nc, unsigned tn, double te, double tc, unsigned nt, unsigned ns, double k=0.) :
    _c(c), _nc(nc), _tn(tn), _te(te), _tc(tc), _nt(nt), _ns(ns), rows(NbRows), cols(NbCols), data(points), k_task(k), alpha(0.3) {}


  std::vector<ResultCluster> run()
  {
    // initialisation
    unsigned iter = 0;
    bool end = false;
    init_clusters();
    while (iter<_ns && !end)
      {
	bool deletion = false;
	bool split_flag = false;
	// allocation
	++iter;
	re_assign();
	check_tn(deletion);
	update_centers();
	if (!deletion)
	  {
	    update_meandis();
	    if (iter==_ns){
	      _tc = 0;
	      end = true;
	    }
	    else if (2*clusters.size()>_nc && (iter%2==0 || clusters.size()>=2*_nc))
	      end = true;
	    if (!end)
	      {
		// splitting
		update_sigmas();
		for (unsigned i=0; i<clusters.size(); ++i)
		  {
		    // retrieving the index of the biggest variance
		    EIGEN_DEFAULT_DENSE_INDEX_TYPE iter;
		    clusters[i].sigma.maxCoeff(&iter);
		    long int sigmaMaxIdx(iter);
		    if (clusters[i].sigma[sigmaMaxIdx]>_te && ((clusters[i].innerMeanDis>Cluster::allMeanDis && clusters[i].ids.size()>2*(_tn+1)) || 2*clusters.size()<=_nc)){
		      split(i);
		      split_flag = true;
		    }
		  }
	      }
	    if (!split_flag)
	      {
		// combining
		update_meandis();
		check_merge();
	      }
	  }
	if (iter<_ns)
	  end = false;
      }
    return resultFormatting();
  }

private:
  void init_clusters();
  std::pair<int, double> get_nearest_cluster(int p_index, int ignore);
  void re_assign();
  void check_tn(bool& flag);
  void update_centers();
  void update_center(Cluster& cluster);
  void update_sigmas();
  void update_sigma(Cluster& cluster);
  void update_meandis();
  void split(const int& c_index);
  void check_merge();
  void merge(const int& id1, const int& id2);
  std::vector<ResultCluster> resultFormatting() const;
};


#endif //ISODATA_ISODATA_H

