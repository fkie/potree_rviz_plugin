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

#include "cloud_loader_2.h"

#include "potree_node.h"

#include <OgreManualObject.h>
#include <boost/filesystem.hpp>
#include <ros/console.h>

#include <queue>

namespace
{

std::size_t extractSize(const char* data)
{
    return (std::size_t(data[0]) & 0xff) | ((std::size_t(data[1] & 0xff)) << 8)
           | ((std::size_t(data[2]) & 0xff) << 16)
           | ((std::size_t(data[3]) & 0xff) << 24)
           | ((std::size_t(data[4]) & 0xff) << 32)
           | ((std::size_t(data[5]) & 0xff) << 40)
           | ((std::size_t(data[6]) & 0xff) << 48)
           | ((std::size_t(data[7]) & 0xff) << 56);
}

std::uint32_t extractUInt32(const char* data)
{
    return (std::uint32_t(data[0]) & 0xff)
           | ((std::uint32_t(data[1]) & 0xff) << 8)
           | ((std::uint32_t(data[2]) & 0xff) << 16)
           | ((std::uint32_t(data[3]) & 0xff) << 24);
}

std::uint16_t extractUInt16(const char* data)
{
    return (std::uint16_t(data[0]) & 0xff)
           | ((std::uint16_t(data[1]) & 0xff) << 8);
}

std::int32_t extractInt32(const char* data)
{
    return (std::int32_t(data[0]) & 0xff)
           | ((std::int32_t(data[1]) & 0xff) << 8)
           | ((std::int32_t(data[2]) & 0xff) << 16)
           | ((std::int32_t(data[3]) & 0xff) << 24);
}

}  // namespace

namespace fkie_potree_rviz_plugin
{

CloudLoader2::CloudLoader2(const std::shared_ptr<CloudMetaData>& meta_data)
    : meta_data_(meta_data)
{
}

std::shared_ptr<const CloudMetaData> CloudLoader2::metaData() const
{
    return meta_data_;
}

std::size_t
CloudLoader2::estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const
{
    return node->data_size_ / meta_data_->point_byte_size_;
}

std::shared_ptr<PotreeNode> CloudLoader2::loadHierarchy() const
{
    std::shared_ptr<PotreeNode> root_node =
        std::make_shared<PotreeNode>("", meta_data_, meta_data_->bounding_box_);
    root_node->hierarchy_offset_ = 0;
    root_node->hierarchy_size_ = meta_data_->hierarchy_root_size_;
    loadNodeHierarchy(root_node);
    return root_node;
}

void CloudLoader2::loadNodeHierarchy(
    const std::shared_ptr<PotreeNode>& root_node) const
{
    fs::path hierarchy_file = meta_data_->cloud_path_ / "hierarchy.bin";
    std::ifstream f{hierarchy_file.c_str()};
    if (!f.good())
    {
        ROS_ERROR_STREAM("failed to open file " << hierarchy_file);
        return;
    }
    f.seekg(root_node->hierarchy_offset_);
    if (!f.good())
    {
        ROS_ERROR_STREAM("cannot seek to hierarchy offset "
                         << root_node->hierarchy_offset_ << " for node "
                         << root_node->name());
        return;
    }
    std::vector<char> data(root_node->hierarchy_size_);
    f.read(data.data(), data.size());
    if (!f.good())
    {
        ROS_ERROR_STREAM("cannot read hierarchy (" << root_node->hierarchy_size_
                                                   << " bytes) for node "
                                                   << root_node->name());
        return;
    }
    f.close();
    if (data.size() < meta_data_->hierarchy_node_size_)
    {
        ROS_ERROR_STREAM("node " << root_node->name()
                                 << " has no valid hierarchy");
        return;
    }

    std::size_t max_offset = data.size() - meta_data_->hierarchy_node_size_;

    std::queue<std::shared_ptr<PotreeNode>> child_nodes, proxy_nodes;
    child_nodes.push(root_node);
    for (std::size_t offset = 0; offset < max_offset;
         offset += meta_data_->hierarchy_node_size_)
    {
        std::shared_ptr<PotreeNode> node = child_nodes.front();
        child_nodes.pop();
        char node_type = data[offset];
        char child_map = data[offset + 1];
        std::size_t point_count = extractUInt32(&data[offset + 2]);
        std::size_t byte_offset = extractSize(&data[offset + 6]);
        std::size_t byte_size = extractSize(&data[offset + 14]);
        if (node_type == 2)
        {
            node->hierarchy_offset_ = byte_offset;
            node->hierarchy_size_ = byte_size;
            node->point_count_ = 0;
            proxy_nodes.push(node);
        }
        else
        {
            node->data_offset_ = byte_offset;
            node->data_size_ = byte_size;
            node->point_count_ = point_count;
        }
        node->point_count_ =
            std::min(node->point_count_,
                     node->data_size_ / meta_data_->point_byte_size_);
        if (node_type == 2)
            continue;
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
                child_nodes.push(node->children_[j]);
            }
        }
    }
    std::vector<char>().swap(data); /* Force memory deallocation, so we don't
                                       keep the entire hierarchy in memory */
    while (!proxy_nodes.empty())
    {
        std::shared_ptr<PotreeNode> node = proxy_nodes.front();
        proxy_nodes.pop();
        loadNodeHierarchy(node);
    }
}

void CloudLoader2::loadPoints(const std::shared_ptr<PotreeNode>& node,
                              bool recursive) const
{
    fs::path octree_file = meta_data_->cloud_path_ / "octree.bin";
    if (node->point_count_ == 0)
    {
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->vertex_data_.reset();
        node->points_.clear();
        node->colors_.clear();
        node->loaded_ = true;
        return;
    }
    if (node->data_size_ / meta_data_->point_byte_size_ < node->point_count_)
    {
        ROS_WARN_STREAM("insufficient data points for node " << node->name());
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->vertex_data_.reset();
        node->points_.clear();
        node->colors_.clear();
        node->loaded_ = true;
        return;
    }
    std::ifstream f{octree_file.c_str()};
    if (!f.good())
    {
        ROS_ERROR_STREAM("failed to open file " << octree_file);
        return;
    }
    f.seekg(node->data_offset_);
    if (!f.good())
    {
        ROS_ERROR_STREAM("cannot seek to octree data offset "
                         << node->data_offset_ << " for node " << node->name());
        return;
    }
    std::vector<char> data(node->data_size_);
    f.read(data.data(), data.size());
    if (!f.good())
    {
        ROS_ERROR_STREAM("cannot read octree data (" << node->data_size_
                                                     << " bytes) for node "
                                                     << node->name());
        return;
    }

    std::vector<Ogre::Vector3> points;
    std::vector<Ogre::ColourValue> colors;
    points.reserve(node->point_count_);
    colors.reserve(node->point_count_);
    std::size_t point_count = node->point_count_;
    std::size_t offset = 0;
    for (const CloudMetaData::PointAttribute& attr :
         meta_data_->pointAttributes())
    {
        if ((attr.name == "POSITION_CARTESIAN" || attr.name == "position")
            && attr.type == CloudMetaData::PointAttribute::Int
            && attr.element_size == 4 && attr.num_elements == 3)
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float x = extractInt32(&data[index]) * meta_data_->scale_[0]
                          + meta_data_->offset_[0];
                float y = extractInt32(&data[index + 4]) * meta_data_->scale_[1]
                          + meta_data_->offset_[1];
                float z = extractInt32(&data[index + 8]) * meta_data_->scale_[2]
                          + meta_data_->offset_[2];
                points.push_back(Ogre::Vector3(x, y, z));
            }
        }
        else if ((attr.name == "RGBA" || attr.name == "rgba"
                  || attr.name == "rgb")
                 && attr.num_elements >= 3
                 && attr.type == CloudMetaData::PointAttribute::UInt)
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float r, g, b, a = 1.f;
                if (attr.element_size == 2)
                {
                    r = extractUInt16(&data[index]) / 65535.f;
                    g = extractUInt16(&data[index + 2]) / 65535.f;
                    b = extractUInt16(&data[index + 4]) / 65535.f;
                    if (attr.num_elements >= 4)
                        a = extractUInt16(&data[index + 6]) / 65535.f;
                }
                else if (attr.element_size == 1)
                {
                    r = static_cast<uint8_t>(data[index]) / 255.f;
                    g = static_cast<uint8_t>(data[index + 1]) / 255.f;
                    b = static_cast<uint8_t>(data[index + 2]) / 255.f;
                    if (attr.num_elements >= 4)
                        a = static_cast<uint8_t>(data[index + 3]) / 255.f;
                }
                else
                {
                    r = 0.f;
                    g = 0.f;
                    b = 0.f;
                }
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
        node->unique_id_ =
            "potree:" + octree_file.string() + "/" + node->name();
        node->loaded_ = true;
    }
    if (recursive)
    {
        std::vector<char>().swap(
            data); /* Force memory deallocation, so we don't keep the entire
                      dataset in memory */
        for (const std::shared_ptr<PotreeNode>& child : node->children_)
        {
            if (child)
                loadPoints(child, true);
        }
    }
}

}  // namespace fkie_potree_rviz_plugin
