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

} // namespace impl
} // namespace task_sequencing
} // namespace hpp
