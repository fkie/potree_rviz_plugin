#pragma once

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
#ifndef SRC_CLOUD_LOADER_STREAMING_H_
#define SRC_CLOUD_LOADER_STREAMING_H_

#include "cloud_loader.hpp"

#include <sensor_msgs/msg/point_cloud.hpp>

#include <memory>
#include <string>
#include <vector>

namespace Ogre
{
class AxisAlignedBox;
}

namespace fkie_potree_rviz_plugin
{

class StreamingCloudLoader : public CloudLoader
{
public:
    static std::shared_ptr<StreamingCloudLoader> fromPointCloud(const sensor_msgs::msg::PointCloud& msg,
                                                                std::size_t max_points_per_node,
                                                                std::size_t max_depth);

    virtual std::shared_ptr<const CloudMetaData> metaData() const override;
    virtual std::shared_ptr<PotreeNode> loadHierarchy() const override;
    virtual std::size_t estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const override;
    virtual void loadPoints(const std::shared_ptr<PotreeNode>& node, bool recursive = false) const override;

private:
    struct DynamicPoint;
    struct OctreeNodeData;

    static std::vector<DynamicPoint> downsamplePoints(const std::vector<DynamicPoint>& input,
                                                      std::size_t max_points);
    static std::unique_ptr<OctreeNodeData> buildOctree(const std::vector<DynamicPoint>& points,
                                                       const Ogre::AxisAlignedBox& bounds,
                                                       std::size_t max_points_per_node, std::size_t max_depth,
                                                       std::size_t depth = 0);
    static std::shared_ptr<PotreeNode> createPotreeNode(const OctreeNodeData& octree_node,
                                                        const std::shared_ptr<CloudMetaData>& meta_data,
                                                        const std::string& name,
                                                        const std::weak_ptr<PotreeNode>& parent,
                                                        std::size_t& unique_index, bool has_color);

    StreamingCloudLoader(const std::shared_ptr<CloudMetaData>& meta_data,
                         const std::shared_ptr<PotreeNode>& root_node);

    std::shared_ptr<CloudMetaData> meta_data_;
    std::shared_ptr<PotreeNode> root_node_;
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_CLOUD_LOADER_STREAMING_H_ */
