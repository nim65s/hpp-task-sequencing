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

#ifndef HPP_TASK_SEQUENCING_MULTI_ROBOT_SOLVER_IMPL_HH
#define HPP_TASK_SEQUENCING_MULTI_ROBOT_SOLVER_IMPL_HH

#include <hpp/task-sequencing/fwd.hh>
#include <hpp/constraints/solver/by-substitution.hh>

namespace hpp{
namespace task_sequencing{

/// Create an explicit constraint that applies on one robot instance
///
/// \param constraint the explicit constraint that applies to the single
///        robot,
/// \param configSpace the configuration space of the multi-robot system,
/// \param offsetQ, offsetV the offsets offsets in the configuration vector
///        from the single robot to the multi robot system.
/// \return shared pointer to the new explicit constraint
///
/// Let us consider a single robot with configuration space of dimension nq
/// and tangent space of dimension nv. Let us consider a multi-robot system
/// composed of N instances of the single robot. An explicit constraint
/// defined for the single robot can be applied to robot number k (between
/// 0 and N-1) by using
///  \li offsetQ = k * nq,
///  \li offsetV = k * nv.
ExplicitPtr_t applyOnOneRobotInstance(const ExplicitPtr_t& constraint,
                                      const LiegroupSpacePtr_t configSpace,
                                      size_type offsetQ, size_type offsetV);

/// Solve problems implying several instances of the same robot.
///
/// This can be useful to find several mobile manipulator configurations that
/// solve a task each and that share the same base pose.
class MultiRobotSolver{
public:
  /// Constructor
  /// \param configSpace configuration space of a single robot
  /// \param nRobots, number of robot instances
  MultiRobotSolver(const LiegroupSpaceConstPtr_t configSpace,
                   size_type nRobots);
  /// Set error threshold
  void errorThreshold(const double& threshold);
  /// Set maximal number of iterations
  void maxIterations(size_type iterations);
  /// Add a constraint to a robot instance
  /// \param constraint constraint that applies to the single robot,
  /// \param index index of robot instance in the solver.
  ///
  /// If the input constraint is explicit, the constraint created will
  /// also be explicit.
  void addConstraint(const ImplicitPtr_t& constraint, size_type index);
  /// Add an equality constraint between values of a joint in two instances
  ///
  /// \param jointName name of the joint,
  /// \param jointConfigSpace configuration space of the joint,
  /// \param iq rank of the joint in configuration vector of single robot,
  /// \param iv rank of the joint in velocity vector of single robot.
  /// \param index1, index2 indices of the robot instances.
  void addEqualityConstraint(const std::string& jointName,
                             const LiegroupSpacePtr_t jointConfigSpace,
                             size_type iq, size_type iv, size_type index1,
                             size_type index2);

  /// Return configuration space of a single robot
  LiegroupSpaceConstPtr_t singleRobotConfigSpace() const;
  /// Return the number of instances of the single robot
  size_type numberRobotInstances() const;
  /// Return the size of the constraints
  size_type dimension() const
  {
    return solver_.dimension();
  }
  /// Return the residual error as a vector
  void residualError(vectorOut_t error)
  {
    return solver_.residualError(error);
  }
  /// Set right hand side of parameterizable functions from input configuration
  void rightHandSideFromConfig(const Configuration_t q);

  /// Solve the problem
  ///
  /// \param q initial guess and solution,
  /// \retval error residual error,
  /// \return whether the solver succeeded.
  bool solve(vectorOut_t q, vectorOut_t error);
  /// Print the solve in a stream
  virtual std::ostream& print(std::ostream& os) const;
private:
  LiegroupSpaceConstPtr_t singleRobotConfigSpace_;
  size_type nInstances_;
  Solver_t solver_;
}; // class MultiRobotSolver

inline std::ostream& operator<<(std::ostream& os,
                                const MultiRobotSolver& solver) {
  return solver.print(os);
}

} // namespace task_sequencing
} // namespace hpp
#endif //HPP_TASK_SEQUENCING_MULTI_ROBOT_SOLVER_IMPL_HH
