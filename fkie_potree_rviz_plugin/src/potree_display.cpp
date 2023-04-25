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

#include "potree_display.h"

#include "cloud_loader.h"
#include "potree_visual.h"

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/status_property.h>
#include <rviz/render_panel.h>

namespace fkie_potree_rviz_plugin
{

PotreeDisplay::PotreeDisplay() : rviz::Display()
{
    path_property_ = new rviz::StringProperty(
        "Path", "", "Filesystem path to the point cloud", this,
        SLOT(updateCloud()));
    frame_property_ = new rviz::TfFrameProperty(
        "Reference Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
        "The TF frame this point cloud will use for its origin.", this, 0, true,
        SLOT(updateOrigin()));
    origin_offset_property_ =
        new rviz::VectorProperty("Offset", Ogre::Vector3::ZERO,
                                 "Allows you to offset the point cloud from "
                                 "the origin of the reference frame.",
                                 this, SLOT(updateOrigin()));
    origin_rotation_property_ = new rviz::QuaternionProperty(
        "Rotation", Ogre::Quaternion::IDENTITY,
        "Allows you to rotate the point cloud w.r.t. the origin of the "
        "reference frame.",
        this, SLOT(updateOrigin()));
    point_budget_property_ =
        new rviz::IntProperty("Point Budget", 1000000,
                              "Set the rendering budget. The more points, the "
                              "more detailed the view.",
                              this, SLOT(updateRenderOptions()));
    point_budget_property_->setMin(100000);
    point_budget_property_->setMax(20000000);
    point_size_property_ = new rviz::FloatProperty(
        "Point Size", 5, "Set the rendering point size.", this,
        SLOT(updateRenderOptions()));
    point_size_property_->setMin(1);
    point_size_property_->setMax(50.0);
    splat_render_property_ = new rviz::BoolProperty(
        "Splat Rendering", false, "Use splats for better visual quality.", this,
        SLOT(updateRenderOptions()));
    splat_render_property_->setDisableChildrenIfFalse(true);
}

void PotreeDisplay::onInitialize()
{
    frame_property_->setFrameManager(context_->getFrameManager());
}

void PotreeDisplay::onEnable()
{
    updateCloud();
}

void PotreeDisplay::onDisable()
{
    visual_.reset();
}

void PotreeDisplay::fixedFrameChanged()
{
    updateOrigin();
}

void PotreeDisplay::updateOrigin()
{
    Ogre::Vector3 position, pos_offset;
    Ogre::Quaternion orientation, ori_offset;
    rviz::FrameManager* m = context_->getFrameManager();
    std::string error_msg;
    if (m->transformHasProblems(frame_property_->getFrameStd(), ros::Time(),
                                error_msg))
    {
        setStatus(rviz::StatusProperty::Error, "Transform",
                  QString::fromStdString(error_msg));
        if (visual_)
            visual_->setVisible(false);
        return;
    }
    if (!m->getTransform(frame_property_->getFrameStd(), ros::Time(), position,
                         orientation))
    {
        ROS_ERROR("Unexpected error transforming from frame '%s' to frame '%s'",
                  qPrintable(frame_property_->getFrame()),
                  qPrintable(fixed_frame_));
        if (visual_)
            visual_->setVisible(false);
        return;
    }
    pos_offset = origin_offset_property_->getVector();
    ori_offset = origin_rotation_property_->getQuaternion();
    position += orientation * pos_offset;
    if (std::abs(ori_offset.Norm() - 1.f) < 1e-2)
        orientation = orientation * ori_offset;
    if (visual_)
    {
        visual_->setVisible(true);
        visual_->setOrigin(position, orientation);
    }
    setStatus(rviz::StatusProperty::Ok, "Transform", "Transform OK");
}

void PotreeDisplay::updateRenderOptions()
{
    if (visual_)
    {
        visual_->setPointBudget(point_budget_property_->getInt());
        visual_->setPointSize(point_size_property_->getFloat());
        visual_->enableSplatRendering(splat_render_property_->getBool());
    }
}

void PotreeDisplay::updateCloud()
{
    visual_.reset();  // first delete the old point cloud from memory, if any
    if (!isEnabled())
        return;
    std::string error_msg;
    fs::path path = path_property_->getStdString();
    try
    {
        std::shared_ptr<CloudLoader> loader = CloudLoader::create(path);
        visual_ = std::make_shared<PotreeVisual>(
            loader, context_->getSceneManager(), scene_node_);
        setStatus(rviz::StatusProperty::Ok, "Cloud",
                  QString("%1 points").arg(loader->metaData()->pointCount()));
        updateOrigin();
        updateRenderOptions();
    }
    catch (std::exception& e)
    {
        setStatus(rviz::StatusProperty::Error, "Cloud", e.what());
    }
}

}  // namespace fkie_potree_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fkie_potree_rviz_plugin::PotreeDisplay, rviz::Display);
