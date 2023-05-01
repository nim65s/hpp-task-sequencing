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

#include <hpp/util/debug.hh>
#include <hpp/task-sequencing/multi-robot-solver.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/constraints/function/of-parameter-subset.hh>
#include <hpp/constraints/affine-function.hh>
#include <hpp/pinocchio/configuration.hh>

namespace hpp{
namespace task_sequencing{

static segments_t applyOffset(const segments_t input, size_type offset)
{
  segments_t res;
  for(segment_t seg : input)
  {
    res.push_back(std::make_pair(seg.first + offset, seg.second));
  }
  return res;
}

ExplicitPtr_t applyOnOneRobotInstance(const ExplicitPtr_t& constraint,
                                      const LiegroupSpacePtr_t configSpace,
                                      size_type offsetQ, size_type offsetV)
{
  ExplicitPtr_t res (Explicit::create(configSpace,
      constraint->explicitFunction(),
      applyOffset(constraint->inputConf(), offsetQ),
      applyOffset(constraint->outputConf(), offsetQ),
      applyOffset(constraint->inputVelocity(), offsetV),
      applyOffset(constraint->outputVelocity(), offsetV),
      constraint->comparisonType(), std::vector<bool>()));
  return res;
}

static LiegroupSpacePtr_t buildConfigSpace(
   const LiegroupSpaceConstPtr_t configSpace, size_type nRobots)
{
  LiegroupSpacePtr_t res(LiegroupSpace::empty());
  size_type n(nRobots);
  while (n > 0){
    *res *=(configSpace); --n;
  }
  return res;
}
MultiRobotSolver::MultiRobotSolver(const DevicePtr_t robot,
                                   size_type nRobots) :
  singleRobotConfigSpace_(robot->configSpace()), nInstances_(nRobots),
  solver_(buildConfigSpace(robot->configSpace(), nRobots))
{
  // Set saturation instance for the solver
  size_type iq=0, iv=0,
    nq = robot->configSpace()->nq(),
    nv = robot->configSpace()->nv();
  vector_t lb, ub;
  lb.resize(nRobots * nq); ub.resize(nRobots * nq);
  Eigen::VectorXi iq2iv; iq2iv.resize(nRobots * nq);

  const pinocchio::Model& m = robot->model();
  for (size_type i=0; i < nRobots; ++i){
    lb.segment(iq, nq) = m.lowerPositionLimit;
    ub.segment(iq, nq) = m.upperPositionLimit;
    for (std::size_t j = 1; j < m.joints.size(); ++j) {
      const size_type nq_j = m.joints[j].nq();
      const size_type nv_j = m.joints[j].nv();
      const size_type idx_q = m.joints[j].idx_q();
      const size_type idx_v = m.joints[j].idx_v();
      for (size_type k = 0; k < nq_j; ++k) {
        const size_type iq_j = idx_q + k;
        const size_type iv_j = idx_v + std::min(k, nv_j - 1);
        iq2iv[iq + iq_j] = (int)(iv + iv_j);
      }
    }
    iq += nq;
    iv += nv;
  }
  solver_.saturation(std::make_shared<constraints::solver::saturation::Bounds>
                     (lb, ub, iq2iv));
}

MultiRobotSolver::MultiRobotSolver() :
  singleRobotConfigSpace_(LiegroupSpace::empty()), nInstances_(0),
  solver_(LiegroupSpace::empty())
{
}

void MultiRobotSolver::errorThreshold(const double& threshold)
{
  solver_.errorThreshold(threshold);
}

void MultiRobotSolver::maxIterations(size_type iterations)
{
  solver_.maxIterations(iterations);
}

void MultiRobotSolver::addConstraint(const ImplicitPtr_t& constraint,
                                     size_type index)
{
  ExplicitPtr_t xplicit(HPP_DYNAMIC_PTR_CAST(Explicit, constraint));
  if (xplicit){
    solver_.add(applyOnOneRobotInstance(xplicit, solver_.configSpace(),
        singleRobotConfigSpace_->nq()*index,
        singleRobotConfigSpace_->nv()*index)
    );
  } else {
    solver_.add(
        Implicit::create(OfParameterSubset::create(
            constraint->functionPtr(),
            solver_.configSpace()->nq(), solver_.configSpace()->nv(),
            std::make_pair(singleRobotConfigSpace_->nq()*index,
                           singleRobotConfigSpace_->nq()),
            std::make_pair(singleRobotConfigSpace_->nv()*index,
                           singleRobotConfigSpace_->nv())),
                            constraint->comparisonType())
    );
  }
}

void MultiRobotSolver::addEqualityConstraint(const std::string& jointName,
    const LiegroupSpacePtr_t jointConfigSpace, size_type iq, size_type iv,
    size_type index1, size_type index2)
{
  DifferentiableFunctionPtr_t jointIdentity(
      constraints::Identity::create(jointConfigSpace,
          std::string("value of ") + jointName));
  segments_t inputConf, outputConf, inputVel, outputVel;
  outputConf.push_back(std::make_pair(index1 *
      singleRobotConfigSpace_->nq() + iq, jointConfigSpace->nq()));
  outputVel.push_back(std::make_pair(index1 *
      singleRobotConfigSpace_->nv() + iv, jointConfigSpace->nv()));
  inputConf.push_back(std::make_pair(index2 *
      singleRobotConfigSpace_->nq() + iq, jointConfigSpace->nq()));
  inputVel.push_back(std::make_pair(index2 *
      singleRobotConfigSpace_->nv() + iv, jointConfigSpace->nv()));
  ExplicitPtr_t constraint(Explicit::create(solver_.configSpace(),
      jointIdentity, inputConf, outputConf, inputVel, outputVel));
  solver_.add(constraint);
}

LiegroupSpaceConstPtr_t MultiRobotSolver::singleRobotConfigSpace() const
{
  return singleRobotConfigSpace_;
}

size_type MultiRobotSolver::numberRobotInstances() const
{
  return nInstances_;
}

void MultiRobotSolver::rightHandSideFromConfig(const Configuration_t q)
{
  solver_.rightHandSideFromConfig(q);
}

bool MultiRobotSolver::solve(vectorOut_t q, vectorOut_t error)
{
  hpp::constraints::solver::HierarchicalIterative::Status status
    (solver_.solve<hpp::constraints::solver::lineSearch::Backtracking>
     (q));
  solver_.residualError(error);
  return (status == hpp::constraints::solver::HierarchicalIterative::SUCCESS);
}

std::ostream& MultiRobotSolver::print(std::ostream& os) const
{
  os << solver_;
  return os;
}

} // namespace task_sequencing
} // namespace hpp
