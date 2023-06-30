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

} // namespace impl
} // namespace task_sequencing
} // namespace hpp
