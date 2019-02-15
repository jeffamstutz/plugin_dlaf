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
        static dlaf::DLAF_Params params;

        if (ImGui::Button("reset inputs"))
          params = dlaf::DLAF_Params();

        static std::shared_ptr<sg::Node> radius_node =
            sg::createNode("radius", "float", 1.f);

        ImGui::NewLine();
        ImGui::Text("Data Generation Parameters:");
        ImGui::DragInt("# particles", &params.NumParticles, 10, 0, 1e9f);
        ImGui::DragFloat("spacing", &params.ParticleSpacing, 0.1f);
        ImGui::DragFloat(
            "attraction distance", &params.AttractionDistance, 0.1f);
        ImGui::DragFloat("min move dist", &params.MinMoveDistance, 0.1f);
        ImGui::DragInt("stubbornness", &params.Stubbornness, 1);
        ImGui::DragFloat("stickiness", &params.Stickiness, 0.1f);
        ImGui::ColorEdit3("center color", (float *)&params.LowerColor.x);
        ImGui::ColorEdit3("outer color", (float *)&params.UpperColor.x);
        ImGui::NewLine();

        ImGui::Text("Scene Parameters:");
        guiSGSingleNode("point radius", *radius_node);
        ImGui::NewLine();

        static bool jobRunning = false;
        static bool cancel     = false;
        static float done      = 0.f;

        if (jobRunning) {
          if (ImGui::Button("cancel"))
            cancel = true;
          ImGui::Text("generating data: ");
          ImGui::SameLine();
          ImGui::ProgressBar(done);
        } else {
          if (ImGui::Button("compute new")) {
            done = 0.f;
            job_scheduler::scheduleJob([&]() {
              jobRunning = true;
              cancel = false;

              job_scheduler::Nodes retval;
              auto spheres_node = createNode("dlaf_spheres", "Spheres");
              spheres_node->remove("material");

              auto sphere_centers = std::make_shared<DataVector3f>();
              sphere_centers->setName("spheres");

              auto results      = dlaf::compute_points(params, done, cancel);
              sphere_centers->v = std::move(results.points);

              auto sphere_colors = std::make_shared<DataVector4f>();
              sphere_colors->setName("color");

              sphere_colors->v = std::move(results.colors);

              spheres_node->add(sphere_centers);
              spheres_node->add(sphere_colors);
              spheres_node->add(radius_node);

              spheres_node->createChild(
                  "bytes_per_sphere", "int", int(sizeof(vec3f)));

              retval.push_back(spheres_node);

              jobRunning = false;

              radius_node->setValue(1.f);

              return retval;
            });
          }
          ImGui::NewLine();
        }

        ImGui::Separator();
        ImGui::Separator();

        if (ImGui::Button("Close"))
          setShown(false);

        ImGui::End();
      }
    }

  }  // namespace dlaf_plugin
}  // namespace ospray
