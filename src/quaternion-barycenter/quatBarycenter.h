#include <Eigen/Core>
#include <Eigen/Eigenvalues>

class QuatBarycenter
{
  Eigen::Matrix<double, Eigen::Dynamic, 4> quaternions; // a quaternion per row
  int nbQuats;
  Eigen::VectorXd weights; // weights to associate to the quaternions
  Eigen::Vector4d barycenter; // the average quaternion
 public:
  explicit QuatBarycenter(Eigen::Matrix<double, Eigen::Dynamic, 4> data)
    { nbQuats = int(data.rows());
      quaternions.resize(nbQuats,4);
      weights.resize(nbQuats);
      for (int i=0; i<nbQuats; i++){
	quaternions.row(i) = (data.row(i)).normalized();
	weights[i] = 1.;
      }
    }

  void computeBarycenter();
  Eigen::Vector4d getBarycenter() const;
 private:
  
};
