// #include <stdlib>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <iostream>

class distanceMatrix
{
  Eigen::MatrixXf _configurations;
  Eigen::MatrixXi _clusters;
  Eigen::ArrayXf _jointSpeeds;
  Eigen::VectorXf _q0;
  int nbVertices;
  int nbClusters;
  Eigen::MatrixXf distances;
 public:
  explicit distanceMatrix(Eigen::MatrixXf configurations, Eigen::MatrixXi clusters,
			  Eigen::ArrayXf jointSpeeds, Eigen::VectorXf q0) :
  _configurations(configurations), _clusters(clusters), _jointSpeeds(jointSpeeds), _q0(q0)
    {nbVertices = _configurations.rows();
      nbClusters = _clusters.rows();
      distances = Eigen::MatrixXf::Zero(nbVertices, nbVertices);}
  void computeDistances();
  float getDist(int i, int j) const;
 private:
  // base configuration distances
  bool baseMoves(int config1, int config2) const;
  float baseL1dist(int config1, int config2) const;
  // arm configuration distances
  float maxJointDiff(int config1, int config2) const;
  float maxJointDiff(int config) const;
  float jointL2dist(int config1, int config2) const;
  float jointL2dist(int config1, int config2, Eigen::VectorXf weights) const;
  // to compute the total distance between two configurations
  float configDist(int config1, int config2) const;
};
