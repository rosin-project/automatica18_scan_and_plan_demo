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

#ifndef SNP_DEMO_WIDGET_H
#define SNP_DEMO_WIDGET_H

#include <QWidget>
#include <memory>
#include <vector>

#include "godel_bridge.h"
#include "command_widget.h"

namespace Ui
{
 class DemoWidget;
}

namespace snp_demo_gui
{

class GodelCommand;

class DemoWidget : public QWidget
{
  Q_OBJECT
public:
  DemoWidget(QWidget* parent = 0);
  virtual ~DemoWidget();
private:
  using CommandArray = std::vector<CommandWidget*>;
  CommandArray commands_;
  CommandWidget* next_;
  CommandWidget* setupCmd(QString name, CommandFunc func, bool clear=false);
  bool selectNext(CommandWidget* cmd);
  void setNext(CommandWidget* cmd);
  std::unique_ptr<Ui::DemoWidget> ui_;
  GodelBridge bridge_;
};
}

#endif // !SNP_DEMO_WIDGET_H
