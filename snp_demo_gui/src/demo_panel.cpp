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

#include "demo_panel.h"

#include "demo_widget.h"
#include <QVBoxLayout>

namespace snp_demo_gui {

DemoPanel::DemoPanel( QWidget* parent)
: rviz::Panel(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new DemoWidget();
  layout->addWidget(widget_);
  setLayout(layout);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(snp_demo_gui::DemoPanel, rviz::Panel )
