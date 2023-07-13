#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <iostream>

namespace hpp{
namespace task_sequencing{

  
Eigen::MatrixXd parseConfigurations(std::string filePath)
{
  std::ifstream infile(filePath);
  std::vector<std::vector<float>> configs;
  configs.clear();
  std::vector<float> config;
  int nbConfigs = 0;
  for (std::string line; std::getline(infile, line); ){
    nbConfigs+=1;
    std::stringstream lineToSplit(line);
    config.clear();
    for (std::string value; std::getline(lineToSplit, value, ' '); ){
      config.push_back(stof(value));
    }
    configs.push_back(config);
  }
  // put into matrix
  int configSize = int(config.size());
  Eigen::MatrixXd configurations(nbConfigs, configSize);
  for (int i=0; i<nbConfigs; i++)
    {
      for (int j=0; j<configSize; j++)
        configurations(i,j) = configs[i][j];
    }
  return configurations;
}

class distanceMatrix
{
  Eigen::MatrixXd _configurations;
  Eigen::MatrixXd _clusters;
  Eigen::VectorXd _jointSpeeds; // Eigen::ArrayXd _jointSpeeds;
  Eigen::VectorXd _q0;
  int nbVertices;
  int nbClusters;
  Eigen::MatrixXd distances;
 public:
  // explicit distanceMatrix(Eigen::MatrixXd configurations, Eigen::MatrixXi clusters,
  // 			  Eigen::VectorXd jointSpeeds, Eigen::VectorXd q0) :
  //   _configurations(configurations), _clusters(clusters), _jointSpeeds(Eigen::ArrayXd(jointSpeeds)), _q0(q0)
  // {nbVertices = int(_configurations.rows());
  //   nbClusters = int(_clusters.rows());
  //   distances = Eigen::MatrixXd::Zero(nbVertices, nbVertices);}
  explicit distanceMatrix(std::string configsPath, Eigen::MatrixXd clusters,
			  Eigen::VectorXd jointSpeeds, Eigen::VectorXd q0) :
    _clusters(clusters), _jointSpeeds(jointSpeeds), //_jointSpeeds(Eigen::ArrayXd(jointSpeeds)),
    _q0(q0)
  { _configurations = parseConfigurations(configsPath);
    nbVertices = int(_configurations.rows());
    nbClusters = int(_clusters.rows());
    distances = Eigen::MatrixXd::Zero(nbVertices, nbVertices);}
  void computeDistances();
  double getDist(int i, int j) const;
  Eigen::MatrixXd getMatrix() const;
  std::string writeDistanceMatrix() const;
 private:
  // base configuration distances
  bool baseMoves(int config1, int config2) const;
  double baseL1dist(int config1, int config2) const;
  // arm configuration distances
  double maxJointDiff(int config1, int config2) const;
  double maxJointDiff(int config) const;
  double jointL2dist(int config1, int config2) const;
  double jointL2dist(int config1, int config2, Eigen::VectorXd weights) const;
  // to compute the total distance between two configurations
  double configDist(int config1, int config2) const;
};

} // task_sequecing
} // hpp
