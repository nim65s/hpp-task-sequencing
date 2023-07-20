#include "distances.h"

/*
COMPOSITION OF A CONFIGURATION
0 - 1 : x,y of the base
2 - 3 : angle of the base in the workshop plane
4     : torso height
5 -11 : angle of the arm articulations (from shoulder to wrist)
12-14 : finger angles (index, middle-ring-little, thumb)
15-16 : neck z and x/y
17-23 : driller x,y,z,quaternion
24-30 : table+P72 x,y,z,quaternion
 */

namespace hpp{
namespace task_sequencing{

bool distanceMatrix::baseMoves(int config1, int config2) const
{
  return (abs(_configurations.row(config2)[0]-_configurations.row(config1)[0])>1.0e-5 || abs(_configurations.row(config2)[1]-_configurations.row(config1)[1])>1.0e-5);
}

double distanceMatrix::baseL1dist(int config1, int config2) const
{
  return abs(_configurations.row(config2)[0]-_configurations.row(config1)[0]) + abs(_configurations.row(config2)[1]-_configurations.row(config1)[1]);
}

double distanceMatrix::maxJointDiff(int config1, int config2) const // most promising
{
  int nbJoints = int(_jointSpeeds.size());
  Eigen::VectorXd c1 = _configurations.row(config1);
  Eigen::VectorXd c2 = _configurations.row(config2);
  Eigen::VectorXd joints1 = c1.segment(4,nbJoints);
  Eigen::VectorXd joints2 = c2.segment(4,nbJoints);
  Eigen::ArrayXd diff = joints2-joints1;
  // Eigen::VectorXd coeffs = Eigen::inverse(_jointSpeeds);
  Eigen::VectorXd coeffs(nbJoints);
  for (int i=0; i<nbJoints; i++)
    coeffs[i] = 1/_jointSpeeds[i];
  Eigen::VectorXd movementTimes = Eigen::abs(diff);
  movementTimes = movementTimes.cwiseProduct(coeffs);
  return double(movementTimes.lpNorm<Eigen::Infinity>());
}

double distanceMatrix::maxJointDiff(int config) const
{
  int nbJoints = int(_jointSpeeds.size());
  Eigen::VectorXd c1 = _configurations.row(config);
  Eigen::VectorXd joints0 = _q0.segment(4,nbJoints);
  Eigen::VectorXd joints1 = c1.segment(4,nbJoints);
  Eigen::ArrayXd diff = joints0-joints1;
  // Eigen::VectorXd coeffs = Eigen::inverse(_jointSpeeds);
  Eigen::VectorXd coeffs(nbJoints);
  for (int i=0; i<nbJoints; i++)
    coeffs[i] = 1/_jointSpeeds[i];
  Eigen::VectorXd movementTimes = Eigen::abs(diff);
  movementTimes = movementTimes.cwiseProduct(coeffs);
  return double(movementTimes.lpNorm<Eigen::Infinity>());
}

double distanceMatrix::jointL2dist(int config1, int config2) const // another possible distance
{
  Eigen::VectorXd c1 = _configurations.row(config1);
  Eigen::VectorXd c2 = _configurations.row(config2);
  Eigen::VectorXd joints1 = c1.segment(4,12);
  Eigen::VectorXd joints2 = c2.segment(4,12);
  return (joints2-joints1).norm();
}

double distanceMatrix::jointL2dist(int config1, int config2, Eigen::VectorXd weights) const // with weights
{
  Eigen::VectorXd c1 = _configurations.row(config1);
  Eigen::VectorXd c2 = _configurations.row(config2);
  Eigen::VectorXd joints1 = c1.segment(4,12);
  Eigen::VectorXd joints2 = c2.segment(4,12);
  Eigen::ArrayXd diff = joints2-joints1;
  Eigen::VectorXd squared = Eigen::abs2(diff);
  return sqrt(squared.dot(weights));
}

// to compute the total distance between two configurations
double distanceMatrix::configDist(int config1, int config2) const
{
  double res = 0;
  if (baseMoves(config1, config2)==true) {
    res += baseL1dist(config1, config2);
    res += maxJointDiff(config1);
    res += maxJointDiff(config2);
  }
  else {
    res += maxJointDiff(config1, config2);
  }
  return res;
}

void distanceMatrix::computeDistances()
{
  std::set<int> allVertices;
  for (int i=0; i<nbVertices; i++)
    allVertices.emplace(i);
  // all costs to infinity (non existant arcs)
  for (int i=0; i<distances.rows(); i++)
    {
      for (int j=0; j<distances.cols(); j++)
	distances(i,j) = 1e12;
    }
  // change costs for existant arcs
  for (int k=0; k<nbClusters; k++)
    {
      std::cout << "cluster " << k << std::endl;
      Eigen::VectorXi clus = _clusters.row(k);
      int clusterSize = int(clus.size());
      while (clus[clusterSize-1]<0)
	clusterSize-=1;
      std::vector<int> cluster;
      for (int i=0; i<clusterSize; i++)
	cluster.emplace_back(clus[i]);
      std::vector<int> verticesToConsider(nbVertices);
      std::vector<int>::iterator it;
      it = std::set_difference(allVertices.begin(), allVertices.end(), cluster.begin(), cluster.end(), verticesToConsider.begin());
      verticesToConsider.resize(it-verticesToConsider.begin());
      for (int i=1; i<clusterSize-1; i++)
	{
	  distances(cluster[i], cluster[i+1]) = 0;
	  for (std::vector<int>::iterator j=verticesToConsider.begin(); j<verticesToConsider.end(); j++)
	    distances(cluster[i-1], *j) = int(100000*configDist(cluster[i], *j));
	}
      distances(cluster[0], cluster[1]) = 0;
      distances(cluster[cluster.size()-1], cluster[0]) = 0;
    }
  std::cout << "Matrix computed" << std::endl;
}

double distanceMatrix::getDist(int i, int j) const
{
  return distances(i,j);
}

Eigen::MatrixXd distanceMatrix::getMatrix() const {
  std::cout << "trying to return matrix" << std::endl;
  return distances;
}

std::string distanceMatrix::writeDistanceMatrix() const {
  std::string fileLocation  = "/tmp/distanceMatrix.txt";
  std::ofstream matrixFile(fileLocation);
  for (int i=0; i<nbVertices; i++)
    {
      for (int j=0; j<nbVertices-1; j++)
	matrixFile << distances(i,j) << ",";
      matrixFile << distances(i,nbVertices-1) << "\n";
    }
  matrixFile.close();
  return fileLocation;
}

} // task_sequencing
} // hpp
