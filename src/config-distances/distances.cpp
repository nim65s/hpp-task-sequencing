#include "distances.h"

/*
COMPOSITION OF A CONFIGURATION (TIAGO)
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

// Checks if two configurations have different base positions
bool distanceMatrix::baseMoves(int config1, int config2) const
{
  Eigen::VectorXd c1(_configurations.row(config1-1));
  Eigen::VectorXd c2(_configurations.row(config2-1));
  return abs(c2[0]-c1[0])>1.0e-5 || abs(c2[1]-c1[1])>1.0e-5;
}

// Computes the Manhattan distance between two base configurations
double distanceMatrix::baseL1dist(int config1, int config2) const
{
  double pi = 3.14159265;
  Eigen::VectorXd c1(_configurations.row(config1-1));
  Eigen::VectorXd c2(_configurations.row(config2-1));
  double angleDiff = abs(std::atan2(c1[3],c1[2]) - std::atan2(c2[3],c2[2]));
  if (angleDiff>pi)
    angleDiff = 2*pi - angleDiff;
  return (abs(c2[0]-c1[0]) + abs(c2[1]-c1[1])) * (1+k_base*angleDiff);
}

// Computes the maximum time needed for a joint to go from its position in config1
// to its position in config2 (supposing no acceleration time and no collision)
double distanceMatrix::maxJointDiff(int config1, int config2) const
{
  int nbJoints = int(_jointSpeeds.size());
  Eigen::VectorXd c1 = _configurations.row(config1-1);
  Eigen::VectorXd c2 = _configurations.row(config2-1);
  Eigen::ArrayXd joints1 = c1.segment(4,nbJoints);
  Eigen::ArrayXd joints2 = c2.segment(4,nbJoints);
  Eigen::VectorXd coeffs(nbJoints);
  for (int i=0; i<nbJoints; i++)
    coeffs[i] = 1/_jointSpeeds[i];
  Eigen::VectorXd movementTimes = Eigen::abs(joints2-joints1);
  movementTimes = movementTimes.cwiseProduct(coeffs);
  return double(movementTimes.lpNorm<Eigen::Infinity>());
}

// Computes the maximum time needed for a joint to go from its rest position
// to its position in config (supposing no acceleration time and no collision)
double distanceMatrix::maxJointDiff(int config) const
{
  int nbJoints = int(_jointSpeeds.size());
  Eigen::VectorXd c1 = _configurations.row(config-1);
  Eigen::ArrayXd joints0 = _q0.segment(4,nbJoints);
  Eigen::ArrayXd joints1 = c1.segment(4,nbJoints);
  Eigen::VectorXd coeffs(nbJoints);
  for (int i=0; i<nbJoints; i++)
    coeffs[i] = 1/_jointSpeeds[i];
  Eigen::VectorXd movementTimes = Eigen::abs(joints0-joints1);
  movementTimes = movementTimes.cwiseProduct(coeffs);
  return double(movementTimes.lpNorm<Eigen::Infinity>());
}

// Computes the L2 distance between the joints in config1 and config2
double distanceMatrix::jointL2dist(int config1, int config2) const // another possible distance
{
  Eigen::VectorXd c1 = _configurations.row(config1-1);
  Eigen::VectorXd c2 = _configurations.row(config2-1);
  return (c2.segment(4,12) - c1.segment(4,12)).norm();
}

// Computes the weighted L2 distance between the joints in config1 and config2
double distanceMatrix::jointL2dist(int config1, int config2, Eigen::VectorXd weights) const // with weights
{
  Eigen::VectorXd c1 = _configurations.row(config1-1);
  Eigen::VectorXd c2 = _configurations.row(config2-1);
  Eigen::ArrayXd joints1 = c1.segment(4,12);
  Eigen::ArrayXd joints2 = c2.segment(4,12);
  Eigen::VectorXd squared = Eigen::abs2(joints2 - joints1);
  return sqrt(squared.dot(weights));
}

// Computes the total distance between two configurations config1 and config2
// If the base moves we first fold the arm, then move the base and eventually unfold the arm
double distanceMatrix::configDist(int config1, int config2) const
{
  double res = 0;
  if (baseMoves(config1, config2)==true) {
    res += k_config*baseL1dist(config1, config2);
    res += maxJointDiff(config1);
    res += maxJointDiff(config2);
  }
  else {
    res += maxJointDiff(config1, config2);
  }
  return res;
}

// Computes the distance matrix of the TSP model of our GTSP (a node per configuration)
void distanceMatrix::computeDistances()
{
  const double infty = 9999999;
  const int bigNegNb = 0;
  const int zero = 500;
  const int factor = 10000;
  std::set<int> allVertices;
  for (int i=1; i<nbConfigs+1; i++)
    allVertices.emplace(i);
  // costs to and from depot are already 0
  // all other costs to infinity (non existant arcs)
  for (int i=1; i<distances.rows(); i++)
    {
      for (int j=1; j<distances.cols(); j++)
	distances(i,j) = infty;
    }
  // change costs for existant arcs
  for (int k=0; k<nbClusters; k++)
    {
      // retrieve the data
      Eigen::VectorXi clus = _clusters.row(k); // retrieve the cluster from the matrix
      std::set<int> clusterSet; // put it into a set
      clusterSet.clear();
      for (int i=0; i<clus.size(); i++)
	clusterSet.insert(clus[i]);
      clusterSet.erase(-1);
      int clusterSize = int(clusterSet.size()); // get its size
      std::vector<int> cluster(clusterSet.begin(), clusterSet.end()); // vector version of the set
      std::vector<int> verticesToConsider(nbConfigs); // vertices out of the cluster
      std::vector<int>::iterator it;
      it = std::set_difference(allVertices.begin(), allVertices.end(), clusterSet.begin(), clusterSet.end(), verticesToConsider.begin());
      verticesToConsider.resize(it-verticesToConsider.begin());
      // compute the costs
      if (clusterSize>2)
	{
	  // first and last nodes
	  distances(cluster[0], cluster[1]) = zero;
	  distances(cluster[clusterSize-1], cluster[0]) = zero;
	  for (std::vector<int>::iterator j=verticesToConsider.begin(); j!=verticesToConsider.end(); j++)
	    { distances(cluster[clusterSize-1], *j) = int(factor*configDist(cluster[0], *j));
	      distances(cluster[clusterSize-2], *j) = int(factor*configDist(cluster[clusterSize-1], *j)); }
	  // other nodes
	  for (int i=1; i<clusterSize-1; i++)
	    {
	      distances(cluster[i], cluster[i+1]) = zero;
	      for (std::vector<int>::iterator j=verticesToConsider.begin(); j!=verticesToConsider.end(); j++)
		distances(cluster[i-1], *j) = int(factor*configDist(cluster[i], *j));
	    }
	}
      else if (clusterSize==2)
	{
	  distances(cluster[0], cluster[1]) = zero;
	  distances(cluster[1], cluster[0]) = zero;
	  for (std::vector<int>::iterator j=verticesToConsider.begin(); j!=verticesToConsider.end(); j++)
	    { distances(cluster[0], *j) = int(factor*configDist(cluster[1], *j));
	      distances(cluster[1], *j) = int(factor*configDist(cluster[0], *j)); }
	}
      else if (clusterSize==1)
	{
	  for (std::vector<int>::iterator j=verticesToConsider.begin(); j!=verticesToConsider.end(); j++)
	    distances(cluster[0], *j) = int(factor*configDist(cluster[0], *j));
	}
      else
	std::cout << "Problem with cluster " << k << " (of size " <<  clusterSize << ")" << std::endl;
    }
  // Transformation to a symmetric instance (as described by Jonker and Volgenant, 1983)
  for (int i=0; i<nbConfigs+1; i++) {
    for (int j=0; j<nbConfigs+1; j++) {
      symmetricDistances(i, j) = infty; // top-left and bottom-right corners to infitnity
      symmetricDistances(nbConfigs+1+i, nbConfigs+1+j) = infty;
      if (i!=j) // copy the original distances such that the matrix is symmetric
	{ symmetricDistances(nbConfigs+1+i, j) = distances(i,j);
	  symmetricDistances(i, nbConfigs+1+j) = distances(j,i); }
      else // negative costs for the diagonal of the original matrix
	{ symmetricDistances(nbConfigs+1+i, j) = bigNegNb;
	  symmetricDistances(i, nbConfigs+1+j) = bigNegNb; }
    }
  }
  // verify symmetry
  bool sym = true;
  for (int i=0; i<symmetricDistances.rows(); i++){
    for (int j=i; j<symmetricDistances.cols(); j++){
      if (symmetricDistances(i,j)!=symmetricDistances(j,i))
	{ if (sym==true)
	    std::cout << i << " " << j << std::endl;
	  sym=false; }
    }
  }
}

// Returns coeffcient (i,j) of the distance matrix i.e. the distance between configurations i and j
double distanceMatrix::getDist(int i, int j) const { return distances(i,j); }

// Returns the distance matrix
Eigen::MatrixXd distanceMatrix::getMatrix() const { return distances; }

// Write the distance matrix to a file
void distanceMatrix::writeMatrix(std::string filePath) const {
  int nbCols = int( symmetricDistances.cols() );
  std::ofstream file(filePath);
  file << "NAME: matrix\nTYPE: TSP\nDIMENSION: " << 2*nbConfigs+2
       << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  for (int i=0; i<symmetricDistances.rows(); i++) {
    for (int j=0; j<nbCols-1; j++)
      file << symmetricDistances(i,j) << " ";
    file << symmetricDistances(i,nbCols-1) << "\n";
  }
  file << "EOF";
  file.close();
}

} // task_sequencing
} // hpp
