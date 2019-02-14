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
// imgui
#include "../../app/widgets/sg_ui/ospray_sg_ui.h"
#include "imgui.h"
// jobs
#include "../../app/jobs/JobScheduler.h"
// dlaf
#include "dlaf.h"
// ospray_sg
#include "sg/common/Data.h"

namespace ospray {
  namespace dlaf_plugin {
    using namespace sg;

    PanelDlaf::PanelDlaf() : Panel("dlaf Panel") {}

    void PanelDlaf::buildUI()
    {
      auto flags = g_defaultWindowFlags | ImGuiWindowFlags_AlwaysAutoResize;
      if (ImGui::Begin("dlaf Panel", nullptr, flags)) {
        ImGui::Text("Diffusion-Limited Aggregation Parameters:");

        ImGui::NewLine();
        ImGui::Text("TODO: ui configurable parameters");
        ImGui::NewLine();

        static bool jobRunning = false;
        static float done      = 0.f;

        if (jobRunning) {
          ImGui::Text("generating data: ");
          ImGui::SameLine();
          ImGui::ProgressBar(done);
        } else {
          if (ImGui::Button("Compute New")) {
            done = 0.f;
            job_scheduler::scheduleJob([&]() {
              jobRunning = true;

              job_scheduler::Nodes retval;
              auto spheres_node = createNode("dlaf_spheres", "Spheres");

              auto sphere_centers = std::make_shared<DataVector3f>();
              sphere_centers->setName("spheres");

              auto vertices     = dlaf::compute_points(done);
              sphere_centers->v = std::move(vertices);

              spheres_node->add(sphere_centers);

              spheres_node->createChild("radius", "float", 1.f);
              spheres_node->createChild(
                  "bytes_per_sphere", "int", int(sizeof(vec3f)));

              retval.push_back(spheres_node);

              jobRunning = false;

              return retval;
            });
          }
        }

        ImGui::Separator();

        if (ImGui::Button("Close"))
          setShown(false);

        ImGui::End();
      }
    }

  }  // namespace dlaf_plugin
}  // namespace ospray
