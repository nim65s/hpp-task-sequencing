// Copyright (c) 2023, LAAS-CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <../corba/tools.impl.hh>
#include <../corba/task-sequencing.hh>

#include <../src/isodata/isodata.h>
#include <../src/config-distances/distances.h>
#include <../src/quaternion-barycenter/quatBarycenter.h>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/liegroup-space.hh>

#include <hpp/core/problem-solver.hh>

#include <hpp/corbaserver/conversions.hh>

namespace hpp {
namespace task_sequencing {
namespace impl {

using corbaServer::floatSeqToConfig;
using corbaServer::floatSeqToVector;
using corbaServer::vectorToFloatSeq;
using corbaServer::matrixToFloatSeqSeq;

DevicePtr_t Tools::getRobotOrThrow()
{
  DevicePtr_t robot(problemSolver()->robot());
  if (!robot){
    throw std::runtime_error("No robot has been loaded.");
  }
  return robot;
}

Tools::Tools() : server_(0x0) {}

hpp::core::ProblemSolverPtr_t Tools::problemSolver()
{
  return server_->problemSolver();
}

CORBA::Double Tools::distanceToMesh(const hpp::floatSeq &q,
   const CORBA::Double *p,CORBA::Double *closest)
{

  typedef ::pinocchio::JointIndex JointIndex;
  typedef ::pinocchio::GeometryObject GeometryObject;
  typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;
  typedef ::pinocchio::GeomIndex GeomIndex;

  // Get pointer to robot
  DevicePtr_t robot(getRobotOrThrow());
  if (!robot->model().existJointName("part/root_joint")){
    throw std::runtime_error("No joint named \"part/root_joint\"");
  }
  // Look for mesh
  GeomIndex geomId(-1);
  JointIndex jid(robot->model().getJointId("part/root_joint"));
  bool found(false);
  CollisionGeometryPtr mesh;
  for (std::size_t i=0; i<robot->geomModel().geometryObjects.size(); ++i){
    const GeometryObject& obj(robot->geomModel().geometryObjects[i]);
    if (obj.parentJoint == jid){
      mesh = obj.geometry;
      if (std::dynamic_pointer_cast<hpp::fcl::BVHModelBase>(mesh)){
        geomId = i;
        found = true;
        break;
      }
    }
  }
  if (!found)
    throw std::runtime_error("Joint \"part/root_joint\" has no geometry of"
                             " type hpp::fcl::BVHModelBase.");
  // Compute forward kinematics
  pinocchio::Data data(robot->model());
  ::pinocchio::GeometryData geomData(robot->geomModel());
  ::pinocchio::forwardKinematics(robot->model(), data,
                                 hpp::corbaServer::floatSeqToVector(q));
  ::pinocchio::updateGeometryPlacements(robot->model(), data,
                                        robot->geomModel(), geomData);
  const ::pinocchio::SE3& meshPose(geomData.oMg[geomId]);
  ::pinocchio::SE3 pointPose; pointPose.setIdentity();
  pointPose.translation()[0] = p[0];
  pointPose.translation()[1] = p[1];
  pointPose.translation()[2] = p[2];

  // Build distance computation data structures
  double eps(1e-4);
  hpp::fcl::Sphere *sphere(new hpp::fcl::Sphere(eps));
  hpp::fcl::ComputeDistance cd(sphere, mesh.get());
  hpp::fcl::DistanceRequest request(true);
  hpp::fcl::DistanceResult result;
  cd(hpp::fcl::Transform3f(pointPose.rotation(), pointPose.translation()),
     hpp::fcl::Transform3f(meshPose.rotation(), meshPose.translation()),
     request, result);
  closest[0] = result.nearest_points[1][0];
  closest[1] = result.nearest_points[1][1];
  closest[2] = result.nearest_points[1][2];
  return result.min_distance + eps;
}

void Tools::isoData(const ::hpp::floatSeqSeq& points, CORBA::Long nbRows,
			 CORBA::Long nbCols, CORBA::ULong c, CORBA::ULong nc, CORBA::ULong tn,
			 CORBA::Double te, CORBA::Double tc, CORBA::ULong nt, CORBA::ULong ns,
			 CORBA::Double k,
			 hpp::corbaserver::task_sequencing::Clusters_out result)
{
  try{
    using hpp::corbaserver::task_sequencing::Clusters;
    
    // Clustering the points
    isodata isodataTest(hpp::corbaServer::floatSeqSeqToMatrix(points), nbRows, nbCols, c, nc, tn, te, tc, nt, ns, k);
    std::vector<ResultCluster> res = isodataTest.run();

    // Storing the result in a sequence
    std::size_t size = res.size();
    hpp::corbaserver::task_sequencing::Cluster *clusters = Clusters::allocbuf((CORBA::ULong)size);
    Clusters* tmp = new Clusters((CORBA::ULong)size, (CORBA::ULong)size, clusters, true);
    result = tmp;
    for (std::size_t i=0; i<size;++i){
      Eigen::VectorXd c = res[i].centroid;
      Eigen::MatrixXd p = res[i].points;
      hpp::corbaserver::task_sequencing::Cluster element;
      element.centroid = *corbaServer::vectorToFloatSeq(c);
      element.points = *corbaServer::matrixToFloatSeqSeq(p);
      clusters[i] = element;
    }
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Tools::computeDistances(const hpp::floatSeqSeq& configs, const hpp::intSeqSeq& clusters,
				const hpp::floatSeq& jointSpeeds, const hpp::floatSeq& q0,
				hpp::floatSeqSeq_out distances)
{
  try{
    // Compute the distance matrix
    distanceMatrix matrix(hpp::corbaServer::floatSeqSeqToMatrix(configs),
			  hpp::corbaServer::intSeqSeqToMatrix(clusters),
			  hpp::corbaServer::floatSeqToVector(jointSpeeds),
			  hpp::corbaServer::floatSeqToVector(q0));
    matrix.computeDistances();
    matrix.generateGTSPtxtIntanceFile("./gtsp.txt");
    
    // Store the matrix in a sequence
    Eigen::MatrixXd distMat = matrix.getMatrix();
    distances = corbaServer::matrixToFloatSeqSeq(distMat);
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Tools::setRobotArmIndices(const CORBA::ULong start, const CORBA::ULong size)
{
  try{
    int armFirstIdx = start;
    int armSize = size;
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Tools::quaternionBarycenter(const hpp::floatSeqSeq& quaternions, hpp::floatSeq_out barycenter)
{
  try{
    QuatBarycenter barycenterInstance(hpp::corbaServer::floatSeqSeqToMatrix(quaternions));
    barycenterInstance.computeBarycenter();
    barycenter = corbaServer::vectorToFloatSeq(barycenterInstance.getBarycenter());
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

} // namespace impl
} // namespace task_sequencing
} // namespace hpp
