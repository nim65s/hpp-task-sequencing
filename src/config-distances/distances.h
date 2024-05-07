#include <set>
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <iostream>

#include <fstream>
#include <string>

namespace hpp{
namespace task_sequencing{

class distanceMatrix
{
  Eigen::MatrixXd _configurations; // a configuration per row
  Eigen::MatrixXi _clusters; // a cluster per row (ended by "-1"s)
  Eigen::VectorXd _jointSpeeds;
  Eigen::VectorXd _q0; // rest configuration of the robot
  int nbConfigs;
  int nbClusters;
  Eigen::MatrixXd distances;
  Eigen::MatrixXd symmetricDistances;
  float k_config; //weight for the base distance in the global distance
  float k_base; // weight for the angular change in the base distance
 public:
  explicit distanceMatrix(Eigen::MatrixXd configurations, Eigen::MatrixXi clusters,
			  Eigen::VectorXd jointSpeeds, Eigen::VectorXd q0) :
    _configurations(configurations), _clusters(clusters), _jointSpeeds(jointSpeeds), _q0(q0)
  {nbConfigs = int(_configurations.rows());
    nbClusters = int(_clusters.rows());
    distances = Eigen::MatrixXd::Zero(nbConfigs+1, nbConfigs+1);
    symmetricDistances = Eigen::MatrixXd::Zero(2*nbConfigs+2, 2*nbConfigs+2);
    k_config = 2.;
    k_base = 2.;}
  void computeDistances();
  void generateGTSPtxtIntanceFile(const std::string& filename);
  double getDist(int i, int j) const;
  Eigen::MatrixXd getMatrix() const;
  void writeMatrix(std::string filePath) const;
 private:
  // Base configuration distances
  bool baseMoves(int config1, int config2) const;
  double baseL1dist(int config1, int config2) const;
  // Arm configuration distances
  double maxJointDiff(int config1, int config2) const;
  double maxJointDiff(int config) const;
  double jointL2dist(int config1, int config2) const;
  double jointL2dist(int config1, int config2, Eigen::VectorXd weights) const;
  // Total distance between two configurations
  double configDist(int config1, int config2) const;
};

} // task_sequecing
} // hpp
