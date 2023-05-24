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

#ifndef HPP_TASK_SEQUENCING_FWD_IMPL_HH
#define HPP_TASK_SEQUENCING_FWD_IMPL_HH

#include <hpp/constraints/fwd.hh>

namespace hpp{
namespace task_sequencing{

typedef pinocchio::Configuration_t Configuration_t;
typedef pinocchio::vector3_t vector3_t;
typedef pinocchio::matrix3_t matrix3_t;
typedef pinocchio::vector_t vector_t;
typedef pinocchio::vectorIn_t vectorIn_t;
typedef pinocchio::vectorOut_t vectorOut_t;
typedef pinocchio::Device Device;
typedef pinocchio::DevicePtr_t DevicePtr_t;
typedef pinocchio::JointPtr_t JointPtr_t;
typedef pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;
typedef pinocchio::LiegroupSpaceConstPtr_t LiegroupSpaceConstPtr_t;
typedef pinocchio::size_type size_type;
typedef pinocchio::LiegroupSpace LiegroupSpace;

typedef constraints::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
typedef constraints::Explicit Explicit;
typedef constraints::ExplicitPtr_t ExplicitPtr_t;
typedef constraints::Implicit Implicit;
typedef constraints::ImplicitPtr_t ImplicitPtr_t;
typedef hpp::constraints::function::OfParameterSubset OfParameterSubset;
typedef constraints::segment_t segment_t;
typedef constraints::segments_t segments_t;
typedef constraints::solver::BySubstitution Solver_t;
typedef constraints::size_type size_type;
  
HPP_PREDEF_CLASS(ExplicitConstraint);
typedef hpp::shared_ptr<ExplicitConstraint> ExplicitConstraintPtr_t;

} // namespace task_sequencing
} // namespace hpp

#endif // HPP_TASK_SEQUENCING_FWD_IMPL_HH
