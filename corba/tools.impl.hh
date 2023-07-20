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

#ifndef HPP_TASK_SEQUENCING_CORBA_TOOLS_IMPL_HH
#define HPP_TASK_SEQUENCING_CORBA_TOOLS_IMPL_HH

#include <corba/tools-idl.hh>
#include <hpp/core/fwd.hh>
#include <hpp/task-sequencing/multi-robot-solver.hh>

namespace hpp {
namespace task_sequencing {
class Server;
namespace impl {

class Tools : public virtual POA_hpp::corbaserver::task_sequencing::Tools
{
public:
  Tools();
  void setServer(Server* server) { server_ = server; }

  virtual CORBA::Double distanceToMesh(const hpp::floatSeq &q,
      const CORBA::Double *p, CORBA::Double *closest);
  virtual void isoData(const ::hpp::floatSeqSeq& points, CORBA::Long nbRows,
			   CORBA::Long nbCols, CORBA::ULong c, CORBA::ULong nc, CORBA::ULong tn,
			   CORBA::Double te, CORBA::Double tc, CORBA::ULong nt, CORBA::ULong ns,
			   CORBA::Double k,
			   hpp::corbaserver::task_sequencing::Clusters_out result);
  virtual void computeDistances(const hpp::floatSeqSeq& configs, const hpp::intSeqSeq& clusters,
				const hpp::floatSeq& jointSpeeds, const hpp::floatSeq& q0,
				hpp::floatSeqSeq_out distances);
  virtual void setRobotArmIndices(const CORBA::ULong start, const CORBA::ULong size);
private:
  core::ProblemSolverPtr_t problemSolver();
  DevicePtr_t getRobotOrThrow();
  MultiRobotSolver solver_;
  Server* server_;
}; // class Tools
} // namespace impl
} // namespace task_sequencing
} // namespace hpp

#endif // HPP_TASK_SEQUENCING_CORBA_TOOLS_IMPL_HH
