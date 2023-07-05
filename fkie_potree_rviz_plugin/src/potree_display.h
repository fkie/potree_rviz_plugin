/****************************************************************************
 *
 * fkie_potree_rviz_plugin
 * Copyright © 2018-2023 Fraunhofer FKIE
 * Author: Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#ifndef SRC_POTREE_DISPLAY_H_
#define SRC_POTREE_DISPLAY_H_

#ifndef Q_MOC_RUN
#    include "fs_path_property.h"

#    include <rviz/display.h>
#    include <rviz/properties/bool_property.h>
#    include <rviz/properties/float_property.h>
#    include <rviz/properties/int_property.h>
#    include <rviz/properties/quaternion_property.h>
#    include <rviz/properties/tf_frame_property.h>
#    include <rviz/properties/vector_property.h>

#    include <memory>
#endif

namespace fkie_potree_rviz_plugin
{

class PotreeVisual;

class PotreeDisplay : public rviz::Display
{
    Q_OBJECT
public:
    PotreeDisplay();

protected:
    virtual void onInitialize() override;
    virtual void onEnable() override;
    virtual void onDisable() override;
    virtual void fixedFrameChanged() override;

private Q_SLOTS:
    void updateOrigin();
    void updateRenderOptions();
    void updateCloud();

private:
    FsPathProperty* path_property_;
    rviz::TfFrameProperty* frame_property_;
    rviz::VectorProperty* origin_offset_property_;
    rviz::QuaternionProperty* origin_rotation_property_;
    rviz::IntProperty* point_budget_property_;
    rviz::FloatProperty* point_size_property_;
    rviz::BoolProperty* splat_render_property_;
    std::shared_ptr<PotreeVisual> visual_;
};

}  // namespace fkie_potree_rviz_plugin

#endif
