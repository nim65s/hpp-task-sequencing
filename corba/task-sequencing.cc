// Copyright (c) 2023 CNRS
// Author: Florent Lamiraux
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

#include <../corba/task-sequencing.hh>
#include <../corba/solver.impl.hh>
#include <hpp/corbaserver/server.hh>

namespace hpp{
namespace task_sequencing{

Server::Server(corbaServer::Server* parent) : corbaServer::ServerPlugin(parent),
                                              solverImpl_(0x0)
{
}

Server::~Server()
{
  if (solverImpl_) delete solverImpl_;
}

std::string Server::name() const { return "task-sequencing"; }

/// Start corba server
void Server::startCorbaServer(const std::string& contextId,
                              const std::string& contextKind) {
  initializeTplServer(solverImpl_, contextId, contextKind, name(), "solver");

  solverImpl_->implementation().setServer(this);
}

hpp::core::ProblemSolverPtr_t Server::problemSolver() {
  return problemSolverMap_->selected();
}

::CORBA::Object_ptr Server::servant(const std::string& name) const {
  if (name == "solver") return solverImpl_->implementation()._this();
  throw std::invalid_argument("No servant " + name);
}

} // namespace task_sequencing
} // name hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::task_sequencing::Server)
