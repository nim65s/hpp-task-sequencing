#include "quatBarycenter.h"

void QuatBarycenter::computeBarycenter()
{
  // compute M
  Eigen::MatrixXd M(4, 4);
  M.setZero();
  for (int i=0; i<nbQuats; i++){
    M += weights[i]*((quaternions.row(i).transpose())*quaternions.row(i));
  }
  // maximize q.transpose()*M*q over q i.e. find eigen vector of maximum eigen value
  // eigen values and vectors
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  Eigen::Vector4d eigenValues = es.eigenvalues().real();
  Eigen::Matrix4d eigenVectors = es.eigenvectors().real();
  // max eigen value
  int maxEigenIdx = 0;
  double maxEigenVal = eigenValues[0];
  for (int i=1; i<4; i++){
    if (eigenValues[i] > maxEigenVal){
      maxEigenIdx = i;
      maxEigenVal = eigenValues[i];
    }
  }
  // associated vector put in S^3
  barycenter = eigenVectors.col(maxEigenIdx).normalized();
}

Eigen::Vector4d QuatBarycenter::getBarycenter() const { return barycenter; }
