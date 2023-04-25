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

#include "cloud_loader_1.h"

#include "potree_node.h"

#include <OgreManualObject.h>
#include <boost/filesystem.hpp>
#include <ros/console.h>

#include <queue>

namespace fkie_potree_rviz_plugin
{

CloudLoader1::CloudLoader1(const std::shared_ptr<CloudMetaData>& meta_data)
    : meta_data_(meta_data)
{
}

std::shared_ptr<const CloudMetaData> CloudLoader1::metaData() const
{
    return meta_data_;
}

std::shared_ptr<PotreeNode> CloudLoader1::loadHierarchy() const
{
    std::shared_ptr<PotreeNode> root_node =
        std::make_shared<PotreeNode>("", meta_data_, meta_data_->bounding_box_);
    loadNodeHierarchy(root_node);
    return root_node;
}

std::size_t
CloudLoader1::estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const
{
    fs::path bin_file = fileName(meta_data_, node->name(), ".bin");
    return fs::file_size(bin_file) / meta_data_->point_byte_size_;
}

void CloudLoader1::loadNodeHierarchy(
    const std::shared_ptr<PotreeNode>& root_node) const
{
    std::queue<std::shared_ptr<PotreeNode>> next_nodes;
    next_nodes.push(root_node);
    char child_map;
    char broken_point_count[4];
    fs::path hrc_file = fileName(meta_data_, root_node->name(), ".hrc");
    std::ifstream f{hrc_file.c_str()};
    if (!f.good())
        ROS_ERROR_STREAM("failed to read file: " << hrc_file);
    f.read(&child_map, 1);
    f.read(broken_point_count, 4);
    while (f.good())
    {
        std::shared_ptr<PotreeNode> node = next_nodes.front();
        next_nodes.pop();
        for (int j = 0; j < 8; ++j)
        {
            if (child_map & (1 << j))
            {
                if (!node->children_[j])
                {
                    std::shared_ptr<PotreeNode> child =
                        std::make_shared<PotreeNode>(
                            node->name() + std::to_string(j), meta_data_,
                            childBB(node->boundingBox(), j), node);
                    node->children_[j] = child;
                }
                next_nodes.push(node->children_[j]);
            }
        }
        f.read(&child_map, 1);
        f.read(broken_point_count, 4);
    }
    std::set<PotreeNode*> seen;  // save the shared_ptr copy overhead and just
                                 // track seen nodes by their address
    while (!next_nodes.empty())
    {
        std::shared_ptr<PotreeNode> node = next_nodes.front()->parent().lock();
        next_nodes.pop();
        if (node && seen.insert(node.get()).second)
            loadNodeHierarchy(node);
    }
}

void CloudLoader1::loadPoints(const std::shared_ptr<PotreeNode>& node,
                              bool recursive) const
{
    fs::path bin_file = fileName(meta_data_, node->name(), ".bin");
    if (!fs::is_regular_file(bin_file))
    {
        ROS_ERROR_STREAM("file not found: " << bin_file);
        return;
    }
    std::size_t size = fs::file_size(bin_file);
    std::ifstream f{bin_file.c_str()};
    if (!f.good())
    {
        ROS_ERROR_STREAM("failed to open file: " << bin_file);
        return;
    }
    std::vector<char> data(size);
    if (!f.read(data.data(), size))
    {
        ROS_ERROR_STREAM("failed to read file: " << bin_file);
        return;
    }
    std::size_t point_count = data.size() / meta_data_->point_byte_size_;
    node->point_count_ = point_count;
    if (point_count == 0)
    {
        ROS_WARN_STREAM("empty node: " << node->name());
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->vertex_data_.reset();
        node->points_.clear();
        node->colors_.clear();
        node->loaded_ = true;
        return;
    }
    std::vector<Ogre::Vector3> points;
    std::vector<Ogre::ColourValue> colors;
    points.reserve(point_count);
    colors.reserve(point_count);
    std::size_t offset = 0;
    Ogre::Vector3 translate = node->bounding_box_.getMinimum();
    for (const CloudMetaData::PointAttribute& attr :
         meta_data_->pointAttributes())
    {
        if (attr.name == "POSITION_CARTESIAN")
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float x = *reinterpret_cast<std::uint32_t*>(&data[index + 0])
                              * meta_data_->scale_[0]
                          + translate.x;
                float y = *reinterpret_cast<std::uint32_t*>(&data[index + 4])
                              * meta_data_->scale_[1]
                          + translate.y;
                float z = *reinterpret_cast<std::uint32_t*>(&data[index + 8])
                              * meta_data_->scale_[2]
                          + translate.z;
                points.push_back(Ogre::Vector3(x, y, z));
            }
        }
        else if (attr.name == "COLOR_PACKED" || attr.name == "RGBA")
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float r =
                    1.f * static_cast<unsigned char>(data[index + 0]) / 255.f;
                float g =
                    1.f * static_cast<unsigned char>(data[index + 1]) / 255.f;
                float b =
                    1.f * static_cast<unsigned char>(data[index + 2]) / 255.f;
                float a =
                    1.f * static_cast<unsigned char>(data[index + 3]) / 255.f;
                colors.push_back(Ogre::ColourValue(r, g, b, a));
            }
        }
        offset += attr.size;
    }
    if (points.empty())
    {
        ROS_WARN_STREAM("no POSITION_CARTESIAN data: " << node->name());
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->vertex_data_.reset();
        node->points_.clear();
        node->colors_.clear();
        node->point_count_ = 0;
        node->loaded_ = true;
        return;
    }
    {
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->points_ = std::move(points);
        node->colors_ = std::move(colors);
        node->unique_id_ = "potree:" + bin_file.string();
        node->loaded_ = true;
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : node->children_)
        {
            if (child)
                loadPoints(child, true);
        }
    }
}

fs::path CloudLoader1::fileName(const std::shared_ptr<CloudMetaData>& meta_data,
                                const std::string& name,
                                const std::string& extension)
{
    fs::path octree_dir = meta_data->cloud_path_ / meta_data->octree_dir_;
    fs::path result;
    std::size_t levels = name.length() / meta_data->hierarchy_step_size_;
    for (std::size_t i = 0; i < levels; ++i)
    {
        result /= name.substr(i * meta_data->hierarchy_step_size_,
                              meta_data->hierarchy_step_size_);
    }
    result /= std::string("r") + name + extension;
    if (fs::is_regular_file(octree_dir / "u" / result))
        return octree_dir / "u" / result;
    return octree_dir / "r" / result;
}

}  // namespace fkie_potree_rviz_plugin
