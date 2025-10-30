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
#    include "fs_path_property.hpp"

#    include <fkie_potree_rviz_plugin_export.h>
#    include <rviz_common/display.hpp>
#    include <rviz_common/properties/bool_property.hpp>
#    include <rviz_common/properties/enum_property.hpp>
#    include <rviz_common/properties/float_property.hpp>
#    include <rviz_common/properties/int_property.hpp>
#    include <rviz_common/properties/quaternion_property.hpp>
#    include <rviz_common/properties/ros_topic_property.hpp>
#    include <rviz_common/properties/tf_frame_property.hpp>
#    include <rviz_common/properties/vector_property.hpp>

#    include <rclcpp/subscription.hpp>
#    include <sensor_msgs/msg/point_cloud.hpp>

#    include <memory>
#endif

#include <mutex>

namespace fkie_potree_rviz_plugin
{

class PotreeVisual;
class CloudLoader;

class FKIE_POTREE_RVIZ_PLUGIN_EXPORT PotreeDisplay : public rviz_common::Display
{
    Q_OBJECT
public:
    PotreeDisplay();

protected:
    virtual void onInitialize() override;
    virtual void onEnable() override;
    virtual void onDisable() override;
    virtual void fixedFrameChanged() override;
    virtual void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
    void updateOrigin();
    void updateRenderOptions();
    void updateCloud();
    void updateSource();
    void updateTopic();
    void updateLodOptions();

private:
    enum SourceMode
    {
        SourceFile = 0,
        SourceTopic = 1
    };

    void subscribe();
    void unsubscribe();
    void processPendingPointCloud();
    void enqueuePointCloud(const sensor_msgs::msg::PointCloud::ConstSharedPtr& msg);
    void setLoader(const std::shared_ptr<CloudLoader>& loader);

    FsPathProperty* path_property_;
    rviz_common::properties::EnumProperty* source_property_;
    rviz_common::properties::RosTopicProperty* topic_property_;
    rviz_common::properties::IntProperty* lod_max_points_property_;
    rviz_common::properties::IntProperty* lod_max_depth_property_;
    rviz_common::properties::TfFrameProperty* frame_property_;
    rviz_common::properties::VectorProperty* origin_offset_property_;
    rviz_common::properties::QuaternionProperty* origin_rotation_property_;
    rviz_common::properties::IntProperty* point_budget_property_;
    rviz_common::properties::FloatProperty* point_size_property_;
    rviz_common::properties::BoolProperty* splat_render_property_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
    std::mutex pending_mutex_;
    sensor_msgs::msg::PointCloud::ConstSharedPtr pending_pointcloud_;
    sensor_msgs::msg::PointCloud::ConstSharedPtr latest_pointcloud_;
    bool pending_update_ = false;
    std::shared_ptr<PotreeVisual> visual_;
};

}  // namespace fkie_potree_rviz_plugin

#endif
