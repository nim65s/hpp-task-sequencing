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

#include <../corba/solver.impl.hh>
#include <../corba/task-sequencing.hh>

#include <../src/isodata/isodata.h>
#include <../src/config-distances/distances.h>
#include <hpp/pinocchio/device.hh>
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

DevicePtr_t Solver::getRobotOrThrow()
{
  DevicePtr_t robot(problemSolver()->robot());
  if (!robot){
    throw std::runtime_error("No robot has been loaded.");
  }
  return robot;
}

Solver::Solver() : solver_(LiegroupSpace::empty(), 0), server_(0x0) {}

hpp::core::ProblemSolverPtr_t Solver::problemSolver()
{
  return server_->problemSolver();
}

void Solver::create(const CORBA::ULong nbInstances)
{
  try{
    DevicePtr_t robot(getRobotOrThrow());
    solver_ = MultiRobotSolver(robot->configSpace(), nbInstances);
  } catch(const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Solver::setErrorThreshold(const double threshold)
{
  try{
    solver_.errorThreshold(threshold);
  } catch(const std::exception& exc) {
    throw Error(exc.what());
  }
}

  void Solver::setMaxIterations(const CORBA::Long iterations)
{
  try{
    solver_.maxIterations(iterations);
  } catch(const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Solver::addConstraint(const char* name, const CORBA::ULong index)
{
  try{
    // Get constraint from problem solver
    ImplicitPtr_t constraint(problemSolver()->numericalConstraint
                             (std::string(name)));
    if (!constraint){
      std::ostringstream os;
      os << "No constraint with name " << name;
      throw std::runtime_error(os.str().c_str());
    }
    solver_.addConstraint(constraint, index);
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Solver::addEqualityConstraint(const char* jointName,
    const CORBA::ULong index1, const CORBA::ULong index2)
{
  try{
    DevicePtr_t robot(getRobotOrThrow());
    JointPtr_t joint(robot->getJointByName(std::string(jointName)));
    size_type iq(joint->rankInConfiguration());
    size_type iv(joint->rankInVelocity());
    solver_.addEqualityConstraint(std::string(jointName),
        joint->configurationSpace(), iq, iv, (size_type) index1,
        (size_type) index2);
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Solver::setRightHandSideFromVector(const hpp::floatSeq& input)
{
  try{
    DevicePtr_t robot(getRobotOrThrow());
    Configuration_t q(floatSeqToVector(input, robot->configSize() *
                                       solver_.numberRobotInstances()));
    solver_.rightHandSideFromConfig(q);
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

bool Solver::solve(const hpp::floatSeq& input, hpp::floatSeq_out output,
                   hpp::floatSeq_out error)
{
  try{
    DevicePtr_t robot(getRobotOrThrow());
    Configuration_t q(floatSeqToVector(input, robot->configSize() *
                                       solver_.numberRobotInstances()));
    vector_t residualError; residualError.resize(solver_.dimension());
    bool res = solver_.solve(q, residualError);
    output = vectorToFloatSeq(q);
    error = vectorToFloatSeq(residualError);
    return res;
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Solver::display(CORBA::String_out solver)
{
  try {
    std::ostringstream oss;
    oss << (solver_);
    solver = oss.str().c_str();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Solver::testIsoData(const ::hpp::floatSeqSeq& points, CORBA::Long nbRows,
			 CORBA::Long nbCols, CORBA::ULong c, CORBA::ULong nc, CORBA::ULong tn,
			 CORBA::Double te, CORBA::Double tc, CORBA::ULong nt, CORBA::ULong ns,
			 CORBA::Double k,
			 hpp::corbaserver::task_sequencing::Clusters_out result)
{
  try{
    using hpp::corbaserver::task_sequencing::Clusters;
    // using the clustering algorithm
    isodata isodataTest(hpp::corbaServer::floatSeqSeqToMatrix(points), nbRows, nbCols, c, nc, tn, te, tc, nt, ns, k);
    std::vector<ResultCluster> res = isodataTest.run();
    // storing the returned result in a sequence
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

// void Solver::computeDistances(const hpp::floatSeqSeq& configs, const hpp::intSeqSeq& clusters,
				// const hpp::floatSeq& jointSpeeds, const hpp::floatSeq& q0,
				// hpp::floatSeqSeq_out distances)
void Solver::computeDistances(const char* configsPath, const hpp::floatSeqSeq& clusters,
			      const hpp::floatSeq& jointSpeeds, const hpp::floatSeq& q0,
			      CORBA::String_out location) //hpp::floatSeqSeq_out distances)
{
  try{
    using hpp::corbaserver::task_sequencing::Clusters;
    // distance matrix computation class
    // distanceMatrix matrix(hpp::corbaServer::floatSeqSeqToMatrix(configs),
    // 			  hpp::corbaServer::intSeqSeqToMatrix(clusters),
    // 			  hpp::corbaServer::floatSeqToVector(jointSpeeds),
    // 			  hpp::corbaServer::floatSeqToVector(q0));
    distanceMatrix matrix(std::string(configsPath),
			  hpp::corbaServer::floatSeqSeqToMatrix(clusters),
			  hpp::corbaServer::floatSeqToVector(jointSpeeds),
			  hpp::corbaServer::floatSeqToVector(q0));
    matrix.computeDistances();
    // storing the distances in a sequence
    // Eigen::MatrixXd distMat = matrix.getMatrix();
    // distances = corbaServer::matrixToFloatSeqSeq(distMat);
    std::ostringstream oss;
    oss << (matrix.writeDistanceMatrix());
    location = oss.str().c_str();
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

void Solver::setRobotArmIndices(const CORBA::ULong start, const CORBA::ULong size)
{
  try{
    armFirstIdx = start;
    armSize = size;
  } catch(const std::exception& exc){
    throw Error(exc.what());
  }
}

} // namespace impl
} // namespace task_sequencing
} // namespace hpp
