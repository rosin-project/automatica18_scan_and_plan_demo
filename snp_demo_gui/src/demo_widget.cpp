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

#include "demo_widget.h"
#include "ui_demo_widget.h"

#include "command_widget.h"

#include <algorithm>

namespace snp_demo_gui
{

enum class LoopMode : int {
  StepMode,
  UntilEnd,
  LoopAll,
  LoopExecute
};

DemoWidget::DemoWidget(QWidget *parent)
: QWidget(parent), ui_(new Ui::DemoWidget)
{
    ui_->setupUi(this);

    CommandWidget *home  = setupCmd("Home", [this](CommandLogFunc log)-> CommandResult {return bridge_.home(log);}, true);
    CommandWidget *scan  = setupCmd("Scan", [this](CommandLogFunc log)-> CommandResult {return bridge_.scan(log);});
    CommandWidget *plan  = setupCmd("Plan", std::bind(&GodelBridge::plan, &bridge_, std::placeholders::_1));
    CommandWidget *simulate  = setupCmd("Simulate", std::bind(&GodelBridge::simulate, &bridge_, std::placeholders::_1));
    CommandWidget *execute  = setupCmd("Execute", std::bind(&GodelBridge::execute, &bridge_, std::placeholders::_1));

    connect(plan, &CommandWidget::failed, this, [this,scan]() {
      setNext(scan);
      if(ui_->modeComboBox->currentIndex()) {
        next_->start();
      }
    });

    connect(ui_->simulateCheckBox, &QCheckBox::clicked, simulate, &CommandWidget::setVisible);

    setNext(commands_.front());
    connect(ui_->stepButton, &QPushButton::clicked, this, [this]() { next_->start(); });
}

DemoWidget::~DemoWidget() = default;

void DemoWidget::setNext(CommandWidget* cmd){
  next_ =  cmd;
  ui_->stepButton->setText(next_->getName());
  next_->enable();
}

bool DemoWidget::selectNext(CommandWidget* cmd){
  bool overflow = cmd == commands_.back();
  CommandWidget* next = overflow ? commands_.front() : *(++(std::find(commands_.begin(), commands_.end(), cmd)));
  if( !next->isVisible()) {
    return selectNext(next) || overflow;
  }
  setNext(next);
  return overflow;
}

CommandWidget* DemoWidget::setupCmd(QString name, CommandFunc func, bool clear){
  CommandWidget* cmd = new CommandWidget(name, func);

  clear |= commands_.empty();

  for(CommandWidget* c: commands_){
    connect(c, &CommandWidget::started, cmd,  &CommandWidget::reset);
    connect(cmd, &CommandWidget::started, c,  &CommandWidget::disable);
    connect(cmd, &CommandWidget::failed, c,  &CommandWidget::enable);
    connect(cmd, &CommandWidget::succeeded, c,  &CommandWidget::enable);
  }
  commands_.push_back(cmd);

  ui_->commandsLayout->addWidget(cmd);

  if(clear) connect(cmd, &CommandWidget::started, ui_->log, &QTextEdit::clear);
  connect(cmd, &CommandWidget::started, this, [this,name]() {
    ui_->stepButton->setEnabled(false);
    ui_->log->append(QString("%1: started").arg(name));
  });

  connect(cmd, &CommandWidget::succeeded, this, [this,name, cmd]() {
    ui_->log->append(QString("%1: finished").arg(name));
    bool overflow = selectNext(cmd);
    bool run = false;
    switch(static_cast<LoopMode>(ui_->modeComboBox->currentIndex())) {
      case LoopMode::UntilEnd:
        run = !overflow;
        break;
      case LoopMode::LoopExecute:
        if(overflow) {
          setNext(commands_.back());
        }
        // no break
      case LoopMode::LoopAll:
        run = true;
        break;
      default:
        break;
    };
    if(run) {
      next_->start();
    } else {
      ui_->stepButton->setEnabled(true);
    }
  });

  connect(cmd, &CommandWidget::log, this, [this, name](QString s) {
    ui_->log->append(QString("%1: %2").arg(name).arg(s));
  });

  connect(cmd, &CommandWidget::failed, this, [this, name, cmd](QString s) {
    setNext(cmd);
    ui_->stepButton->setEnabled(true);
    if(!s.isEmpty()) ui_->log->append(QString("%1: failed with error '%2'").arg(name).arg(s));
    else ui_->log->append(QString("%1: failed with unkown error").arg(name));
  });

  return cmd;
};

}
