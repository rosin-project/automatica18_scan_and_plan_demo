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

#ifndef SNP_DEMO_PANEL_H
#define SNP_DEMO_PANEL_H

#include <rviz/panel.h>

namespace snp_demo_gui {

class DemoWidget;

class DemoPanel : public rviz::Panel {
  Q_OBJECT
public:
  DemoPanel( QWidget* parent = 0 );
private:
  DemoWidget *widget_;
};

}
#endif // SNP_DEMO_PANEL_H
