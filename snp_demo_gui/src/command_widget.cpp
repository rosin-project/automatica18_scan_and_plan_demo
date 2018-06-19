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

#include "command_widget.h"
#include "ui_command_widget.h"

#include <QtConcurrent/QtConcurrentRun>

namespace snp_demo_gui
{

CommandWidget::CommandWidget(QString name, CommandFunc func, QWidget *parent)
: QWidget(parent), ui_(new Ui::CommandWidget), name_(name), func_(func)
{
    ui_->setupUi(this);
    ui_->pushButton->setText(name);
    reset();
    connect(ui_->pushButton, &QPushButton::clicked, this, &CommandWidget::start);
    connect(this, &CommandWidget::started, this, &CommandWidget::disable);
    connect(this, &CommandWidget::failed, this, &CommandWidget::enable);
    connect(this, &CommandWidget::succeeded, this, &CommandWidget::enable);
}

void CommandWidget::start(){
  if(!ui_->pushButton->isEnabled()) return;
  Q_EMIT started();
  ui_->label->setText("RUN");
  ui_->label->setStyleSheet("QLabel { background-color: yellow; }");
  QtConcurrent::run([this](){
    CommandResult res = func_([this](const std::string &msg) { Q_EMIT log(QString::fromStdString(msg));});
    if (std::get<0>(res)){
      ui_->label->setText("OK");
      ui_->label->setStyleSheet("QLabel { background-color: green; }");
      Q_EMIT succeeded();
    }else{
      ui_->label->setText("ERR");
      ui_->label->setStyleSheet("QLabel { background-color: red; }");
      Q_EMIT failed(QString::fromStdString(std::get<1>(res)));
    }
  });
}

void CommandWidget::reset() {
  ui_->label->setText("");
  ui_->label->setStyleSheet("");
  disable();
}
void CommandWidget::disable() {
  ui_->pushButton->setEnabled(false);
}
void CommandWidget::enable() {
  ui_->pushButton->setEnabled(true);
}
CommandWidget::~CommandWidget() = default;

}
