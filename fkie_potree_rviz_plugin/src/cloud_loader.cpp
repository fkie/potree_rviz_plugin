/****************************************************************************
 *
 * fkie_potree_rviz_plugin
 * Copyright © 2018 Fraunhofer FKIE
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

#include "cloud_loader.h"
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include "potree_node.h"
#include <queue>
#include <OgreManualObject.h>

namespace fkie_potree_rviz_plugin
{

CloudLoader::CloudLoader(const fs::path& path)
{
    std::string error_msg;
    if (!isValid(path, error_msg)) throw std::runtime_error(error_msg);
    fs::path cloud_file = path / "cloud.js";
    meta_data_ = std::make_shared<CloudMetaData>();
    meta_data_->readFromJson(cloud_file);
}

bool CloudLoader::isValid(const fs::path& path, std::string& error_msg)
{
    error_msg.clear();
    if (!fs::is_directory(path))
    {
        error_msg = "not an existing folder"; return false;
    }
    fs::path cloud_file = path / "cloud.js";
    if (!fs::is_regular(cloud_file))
    {
        error_msg = "not a Potree folder"; return false;
    }
    try
    {
        CloudMetaData meta_data;
        fs::path cloud_file = path / "cloud.js";
        meta_data.readFromJson(cloud_file);
        return true;
    } catch (std::exception& e)
    {
        error_msg = e.what();
        return false;
    }
}

std::shared_ptr<PotreeNode> CloudLoader::loadHierarchy() const
{
    std::shared_ptr<PotreeNode> root_node = std::make_shared<PotreeNode>("", meta_data_, meta_data_->bounding_box_);
    loadNodeHierarchy(root_node);
    return root_node;
}

void CloudLoader::loadNodeHierarchy(const std::shared_ptr<PotreeNode>& root_node) const
{
    std::queue<std::shared_ptr<PotreeNode>> next_nodes;
    next_nodes.push(root_node);
    char cfg[5];
    fs::path hrc_file = fileName(meta_data_, root_node->name(), ".hrc");
    std::ifstream f{hrc_file.c_str()};
    if (!f.good()) ROS_ERROR_STREAM("failed to read file: " << hrc_file);
    f.read(cfg, 5);
    while (f.good())
    {
        std::shared_ptr<PotreeNode> node = next_nodes.front(); next_nodes.pop();
        for (int j = 0; j < 8; ++j)
        {
            if (cfg[0] & (1 << j))
            {
                if (!node->children_[j])
                {
                    std::shared_ptr<PotreeNode> child = std::make_shared<PotreeNode>(node->name() + std::to_string(j), meta_data_, childBB(node->boundingBox(), j), node);
                    node->children_[j] = child;
                }
                next_nodes.push(node->children_[j]);
            }
        }
        f.read(cfg, 5);
    }
    std::set<PotreeNode*> seen; // save the shared_ptr copy overhead and just track seen nodes by their address
    while (!next_nodes.empty())
    {
        std::shared_ptr<PotreeNode> node = next_nodes.front()->parent().lock(); next_nodes.pop();
        if (node && seen.insert(node.get()).second) loadNodeHierarchy(node);
    }
}

void CloudLoader::loadPoints(const std::shared_ptr<PotreeNode>& node, bool recursive) const
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
    std::vector<char> data;
    data.resize(size);
    if (!f.read(data.data(), size))
    {
        ROS_ERROR_STREAM("failed to read file: " << bin_file);
        return;
    }
    std::size_t point_count = data.size() / meta_data_->point_byte_size_;
    if (point_count == 0)
    {
        ROS_WARN_STREAM("empty node: " << node->name());
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->vertex_data_.reset();
        node->points_.clear();
        node->colors_.clear();
        node->point_count_ = 0;
        node->loaded_ = true;
        return;
    }
    std::vector<Ogre::Vector3> points;
    std::vector<Ogre::ColourValue> colors;
    points.reserve(point_count);
    colors.reserve(point_count);
    std::size_t offset = 0;
    Ogre::Vector3 translate = node->bounding_box_.getMinimum();
    for (const std::string& attr : meta_data_->point_attributes_)
    {
        if (attr == "POSITION_CARTESIAN")
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float x = *reinterpret_cast<std::uint32_t*>(&data[index + 0]) * meta_data_->scale_ + translate.x;
                float y = *reinterpret_cast<std::uint32_t*>(&data[index + 4]) * meta_data_->scale_ + translate.y;
                float z = *reinterpret_cast<std::uint32_t*>(&data[index + 8]) * meta_data_->scale_ + translate.z;
                points.push_back(Ogre::Vector3(x, y, z));
            }
        }
        else if (attr == "COLOR_PACKED")
        {
            for (std::size_t i = 0; i < point_count; ++i)
            {
                std::size_t index = offset + i * meta_data_->point_byte_size_;
                float r = 1.f * data[index + 0] / 255.f;
                float g = 1.f * data[index + 1] / 255.f;
                float b = 1.f * data[index + 2] / 255.f;
                float a = 1.f * data[index + 3] / 255.f;
                colors.push_back(Ogre::ColourValue(r, g, b, a));
            }
        }
        offset += CloudMetaData::sizeOf(attr);
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
        node->point_count_ = point_count;
        node->unique_id_ = "potree:" + bin_file.string();
        node->loaded_ = true;
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : node->children_)
        {
            if (child) loadPoints(child, true);
        }
    }
}

fs::path CloudLoader::fileName(const std::shared_ptr<CloudMetaData>& meta_data, const std::string& name, const std::string& extension)
{
    fs::path octree_dir = meta_data->cloud_path_ / meta_data->octree_dir_;
    fs::path result;
    std::size_t levels = name.length() / meta_data->hierarchy_step_size_;
    for (std::size_t i = 0; i < levels; ++i)
    {
        result /= name.substr(i * meta_data->hierarchy_step_size_, meta_data->hierarchy_step_size_);
    }
    result /= std::string("r") + name + extension;
    if (fs::is_regular_file(octree_dir / "u" / result))
        return octree_dir / "u" / result;
    return octree_dir / "r" / result;
}

Ogre::AxisAlignedBox CloudLoader::childBB(const Ogre::AxisAlignedBox& parent, int index)
{
    assert(!parent.isInfinite());
    Ogre::Vector3 min = parent.getMinimum(), max = parent.getMaximum(), half_size = parent.getHalfSize();
    if (index & 1) min.z += half_size.z; else max.z -= half_size.z;
    if (index & 2) min.y += half_size.y; else max.y -= half_size.y;
    if (index & 4) min.x += half_size.x; else max.x -= half_size.x;
    return Ogre::AxisAlignedBox(min, max);
}

} // namespace fkie_rviz_plugin_potree
