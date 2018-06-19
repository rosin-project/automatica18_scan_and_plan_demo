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

#ifndef SNP_COMMAND_WIDGET_H
#define SNP_COMMAND_WIDGET_H

#include <QWidget>
#include <memory>

#include "command.h"

namespace Ui
{
 class CommandWidget;
}

namespace snp_demo_gui
{

class CommandWidget : public QWidget
{
  Q_OBJECT
public:
  CommandWidget(QString name, CommandFunc func, QWidget* parent = 0);
  virtual ~CommandWidget();
  QString getName() const { return name_; }

public Q_SLOTS:
  void disable();
  void enable();
  void start();
  void reset();

Q_SIGNALS:
  void started();
  void succeeded();
  void failed(QString);
  void log(QString);

private:
  std::unique_ptr<Ui::CommandWidget> ui_;
  QString name_;
  CommandFunc func_;
};
}

#endif // !SNP_COMMAND_WIDGET_H
