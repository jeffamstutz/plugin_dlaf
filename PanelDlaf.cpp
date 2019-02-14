// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "PanelDlaf.h"

#include "imgui.h"

namespace ospray {
  namespace dlaf_plugin {

    PanelDlaf::PanelDlaf() : Panel("Dlaf Panel") {}

    void PanelDlaf::buildUI()
    {
      ImGui::OpenPopup("Dlaf Panel");

      if (ImGui::BeginPopupModal(
              "Dlaf Panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("%s", "Hello from the dlaf plugin!");
        ImGui::Separator();

        if (ImGui::Button("Close")) {
          setShown(false);
          ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
      }
    }

  }  // namespace dlaf_plugin
}  // namespace ospray
