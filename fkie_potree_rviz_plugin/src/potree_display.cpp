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

#include "potree_display.hpp"

#include "cloud_loader.hpp"
#include "cloud_loader_streaming.hpp"
#include "potree_visual.hpp"

#include <potree_display.moc>
#include <algorithm>
#include <functional>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#define LOG_STDERR std::cerr << "[fkie_potree_rviz_plugin::PotreeDisplay] "

namespace fkie_potree_rviz_plugin
{

PotreeDisplay::PotreeDisplay() : rviz_common::Display()
{
    source_property_ = new rviz_common::properties::EnumProperty("Source", "Potree Folder",
                                                                 "Choose the data source for the point cloud.", this,
                                                                 SLOT(updateSource()));
    source_property_->addOption("Potree Folder", SourceFile);
    source_property_->addOption("ROS Topic", SourceTopic);

    path_property_ = new FsPathProperty("Path", "", "Filesystem path to the point cloud", this, SLOT(updateCloud()));
    topic_property_ = new rviz_common::properties::RosTopicProperty(
        "PointCloud Topic", "", QString::fromLatin1("sensor_msgs/msg/PointCloud"),
        "PointCloud topic to subscribe for streaming clouds.", this, SLOT(updateTopic()));
    topic_property_->setHidden(true);
    lod_max_points_property_ =
        new rviz_common::properties::IntProperty("LOD Max Points", 2000,
                                                 "Maximum number of points stored per octree node for streamed clouds.",
                                                 this, SLOT(updateLodOptions()));
    lod_max_points_property_->setMin(64);
    lod_max_points_property_->setMax(200000);
    lod_max_points_property_->setHidden(true);
    lod_max_depth_property_ = new rviz_common::properties::IntProperty(
        "LOD Max Depth", 6, "Maximum octree depth generated for streamed clouds.", this, SLOT(updateLodOptions()));
    lod_max_depth_property_->setMin(1);
    lod_max_depth_property_->setMax(12);
    lod_max_depth_property_->setHidden(true);

    frame_property_ = new rviz_common::properties::TfFrameProperty(
        "Reference Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
        "The TF frame this point cloud will use for its origin.", this, 0, true, SLOT(updateOrigin()));
    origin_offset_property_ = new rviz_common::properties::VectorProperty("Offset", Ogre::Vector3::ZERO,
                                                                          "Allows you to offset the point cloud from "
                                                                          "the origin of the reference frame.",
                                                                          this, SLOT(updateOrigin()));
    origin_rotation_property_ =
        new rviz_common::properties::QuaternionProperty("Rotation", Ogre::Quaternion::IDENTITY,
                                                        "Allows you to rotate the point cloud w.r.t. the origin of the "
                                                        "reference frame.",
                                                        this, SLOT(updateOrigin()));
    point_budget_property_ = new rviz_common::properties::IntProperty("Point Budget", 1000000,
                                                                      "Set the rendering budget. The more points, the "
                                                                      "more detailed the view.",
                                                                      this, SLOT(updateRenderOptions()));
    point_budget_property_->setMin(100000);
    point_budget_property_->setMax(20000000);
    point_size_property_ = new rviz_common::properties::FloatProperty("Point Size", 5, "Set the rendering point size.",
                                                                      this, SLOT(updateRenderOptions()));
    point_size_property_->setMin(1);
    point_size_property_->setMax(50.0);
    splat_render_property_ = new rviz_common::properties::BoolProperty(
        "Splat Rendering", false, "Use splats for better visual quality.", this, SLOT(updateRenderOptions()));
    splat_render_property_->setDisableChildrenIfFalse(true);
}

void PotreeDisplay::onInitialize()
{
    frame_property_->setFrameManager(context_->getFrameManager());
    topic_property_->initialize(context_->getRosNodeAbstraction());
    updateSource();
}

void PotreeDisplay::onEnable()
{
    if (source_property_->getOptionInt() == SourceTopic)
    {
        setStatus(rviz_common::properties::StatusProperty::Warn, "Cloud", "Waiting for point cloud...");
        subscribe();
    }
    else
    {
        updateCloud();
    }
}

void PotreeDisplay::onDisable()
{
    visual_.reset();
    unsubscribe();
}

void PotreeDisplay::fixedFrameChanged()
{
    updateOrigin();
}

void PotreeDisplay::update(float wall_dt, float ros_dt)
{
    rviz_common::Display::update(wall_dt, ros_dt);
    if (source_property_->getOptionInt() == SourceTopic)
        processPendingPointCloud();
}

void PotreeDisplay::updateOrigin()
{
    Ogre::Vector3 position, pos_offset;
    Ogre::Quaternion orientation, ori_offset;
    rviz_common::FrameManagerIface* m = context_->getFrameManager();
    std::string error_msg;
    if (m->transformHasProblems(frame_property_->getFrameStd(), rclcpp::Time(0, 0, RCL_ROS_TIME), error_msg))
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(error_msg));
        if (visual_)
            visual_->setVisible(false);
        return;
    }
    if (!m->getTransform(frame_property_->getFrameStd(), rclcpp::Time(0, 0, RCL_ROS_TIME), position, orientation))
    {
        LOG_STDERR << "Unexpected error transforming from frame '" << qPrintable(frame_property_->getFrame())
                   << "' to frame '" << qPrintable(fixed_frame_) << "'" << std::endl;
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
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
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
    if (source_property_->getOptionInt() != SourceFile)
        return;
    visual_.reset();  // first delete the old point cloud from memory, if any
    if (!isEnabled())
        return;
    fs::path path = path_property_->getFsPath();
    try
    {
        std::shared_ptr<CloudLoader> loader = CloudLoader::create(path);
        setLoader(loader);
    }
    catch (std::exception& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Cloud", e.what());
    }
}

void PotreeDisplay::updateSource()
{
    bool topic_mode = source_property_->getOptionInt() == SourceTopic;
    path_property_->setHidden(topic_mode);
    topic_property_->setHidden(!topic_mode);
    lod_max_points_property_->setHidden(!topic_mode);
    lod_max_depth_property_->setHidden(!topic_mode);

    if (topic_mode)
    {
        unsubscribe();
        visual_.reset();
        {
            std::lock_guard<std::mutex> lock{pending_mutex_};
            pending_pointcloud_.reset();
            pending_update_ = false;
        }
        setStatus(rviz_common::properties::StatusProperty::Warn, "Cloud", "Waiting for point cloud...");
        subscribe();
    }
    else
    {
        unsubscribe();
        {
            std::lock_guard<std::mutex> lock{pending_mutex_};
            pending_pointcloud_.reset();
            pending_update_ = false;
            latest_pointcloud_.reset();
        }
        updateCloud();
    }
}

void PotreeDisplay::updateTopic()
{
    if (source_property_->getOptionInt() != SourceTopic)
        return;
    unsubscribe();
    subscribe();
}

void PotreeDisplay::updateLodOptions()
{
    if (source_property_->getOptionInt() != SourceTopic)
        return;
    {
        std::lock_guard<std::mutex> lock{pending_mutex_};
        if (latest_pointcloud_)
        {
            pending_pointcloud_ = latest_pointcloud_;
            pending_update_ = true;
        }
    }
    if (context_)
        context_->queueRender();
}

void PotreeDisplay::subscribe()
{
    if (!isEnabled() || source_property_->getOptionInt() != SourceTopic)
        return;
    std::string topic = topic_property_->getTopicStd();
    if (topic.empty())
    {
        setStatus(rviz_common::properties::StatusProperty::Warn, "Cloud", "No topic selected");
        subscription_.reset();
        return;
    }
    auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!ros_node_abstraction)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Cloud", "ROS node unavailable");
        return;
    }
    rclcpp::Node::SharedPtr raw_node = ros_node_abstraction->get_raw_node();
    if (!raw_node)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Cloud", "ROS node unavailable");
        return;
    }
    rclcpp::SensorDataQoS qos;
    subscription_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud>(
        topic, qos, std::bind(&PotreeDisplay::enqueuePointCloud, this, std::placeholders::_1));
    setStatus(rviz_common::properties::StatusProperty::Warn, "Cloud", "Waiting for point cloud...");
}

void PotreeDisplay::unsubscribe()
{
    subscription_.reset();
}

void PotreeDisplay::enqueuePointCloud(const sensor_msgs::msg::PointCloud::ConstSharedPtr& msg)
{
    if (source_property_->getOptionInt() != SourceTopic)
        return;
    {
        std::lock_guard<std::mutex> lock{pending_mutex_};
        pending_pointcloud_ = msg;
        pending_update_ = true;
    }
    if (context_)
        context_->queueRender();
}

void PotreeDisplay::processPendingPointCloud()
{
    sensor_msgs::msg::PointCloud::ConstSharedPtr msg;
    {
        std::lock_guard<std::mutex> lock{pending_mutex_};
        if (!pending_update_)
            return;
        msg = pending_pointcloud_;
        pending_pointcloud_.reset();
        pending_update_ = false;
    }
    if (!msg)
        return;
    try
    {
        std::size_t max_points = static_cast<std::size_t>(std::max(1, lod_max_points_property_->getInt()));
        std::size_t max_depth = static_cast<std::size_t>(std::max(1, lod_max_depth_property_->getInt()));
        std::shared_ptr<StreamingCloudLoader> loader =
            StreamingCloudLoader::fromPointCloud(*msg, max_points, max_depth);
        latest_pointcloud_ = msg;
        setLoader(loader);
    }
    catch (std::exception& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Cloud", e.what());
    }
}

void PotreeDisplay::setLoader(const std::shared_ptr<CloudLoader>& loader)
{
    visual_.reset();
    if (!loader)
        return;
    if (!isEnabled())
        return;
    try
    {
        visual_ = std::make_shared<PotreeVisual>(loader, context_->getSceneManager(), scene_node_);
        setStatus(rviz_common::properties::StatusProperty::Ok, "Cloud",
                  QString("%1 points").arg(loader->metaData()->pointCount()));
        updateOrigin();
        updateRenderOptions();
    }
    catch (std::exception& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Cloud", e.what());
        visual_.reset();
    }
}

}  // namespace fkie_potree_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fkie_potree_rviz_plugin::PotreeDisplay, rviz_common::Display);
