// Copyright 2018 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SNP_GODEL_BRIDGE_H
#define SNP_GODEL_BRIDGE_H

#include <memory>
#include "command.h"

namespace snp_demo_gui
{

class GodelBridgeImpl;

class GodelBridge
{
public:
  GodelBridge();

  CommandResult home(CommandLogFunc);
  CommandResult scan(CommandLogFunc);
  CommandResult plan(CommandLogFunc);
  CommandResult simulate(CommandLogFunc);
  CommandResult execute(CommandLogFunc);

  virtual ~GodelBridge();
private:
  std::unique_ptr<GodelBridgeImpl> impl_;
};
}

#endif // !SNP_GODEL_BRIDGE_H
