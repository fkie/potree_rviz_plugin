#include "cloud_loader_streaming.hpp"

#include "cloud_meta_data.hpp"
#include "potree_node.hpp"

#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fkie_potree_rviz_plugin
{
struct StreamingCloudLoader::DynamicPoint
{
    Ogre::Vector3 position;
    Ogre::ColourValue color;
    std::size_t index = 0;
};

struct StreamingCloudLoader::OctreeNodeData
{
    Ogre::AxisAlignedBox bounds;
    std::vector<StreamingCloudLoader::DynamicPoint> points;
    std::array<std::unique_ptr<OctreeNodeData>, 8> children;
};

std::unique_ptr<StreamingCloudLoader::OctreeNodeData> StreamingCloudLoader::buildOctree(
    const std::vector<DynamicPoint>& points, const Ogre::AxisAlignedBox& bounds, std::size_t max_points_per_node,
    std::size_t max_depth, std::size_t depth)
{
    auto node = std::make_unique<OctreeNodeData>();
    node->bounds = bounds;
    node->points = downsamplePoints(points, max_points_per_node);
    if (points.size() <= max_points_per_node || depth >= max_depth)
        return node;

    std::unordered_set<std::size_t> parent_ids;
    parent_ids.reserve(node->points.size());
    for (const DynamicPoint& point : node->points)
        parent_ids.insert(point.index);

    std::array<std::vector<DynamicPoint>, 8> child_points;
    Ogre::Vector3 center = bounds.getCenter();
    for (const DynamicPoint& point : points)
    {
        if (parent_ids.find(point.index) != parent_ids.end())
            continue;
        int idx = 0;
        if (point.position.x >= center.x)
            idx |= 4;
        if (point.position.y >= center.y)
            idx |= 2;
        if (point.position.z >= center.z)
            idx |= 1;
        child_points[idx].push_back(point);
    }
    for (int i = 0; i < 8; ++i)
    {
        if (child_points[i].empty())
            continue;
        Ogre::AxisAlignedBox child_bounds = CloudLoader::childBB(bounds, i);
        node->children[i] =
            buildOctree(child_points[i], child_bounds, max_points_per_node, max_depth, depth + 1);
    }
    return node;
}

std::shared_ptr<PotreeNode> StreamingCloudLoader::createPotreeNode(const OctreeNodeData& octree_node,
                                                                    const std::shared_ptr<CloudMetaData>& meta_data,
                                                                    const std::string& name,
                                                                    const std::weak_ptr<PotreeNode>& parent,
                                                                    std::size_t& unique_index, bool has_color)
{
    std::shared_ptr<PotreeNode> node = std::make_shared<PotreeNode>(name, meta_data, octree_node.bounds, parent);
    {
        std::lock_guard<std::mutex> lock{node->mutex_};
        node->points_.reserve(octree_node.points.size());
        if (has_color)
            node->colors_.reserve(octree_node.points.size());
        for (const DynamicPoint& point : octree_node.points)
        {
            node->points_.push_back(point.position);
            if (has_color)
                node->colors_.push_back(point.color);
        }
        node->point_count_ = node->points_.size();
        node->loaded_ = true;
        node->vertex_data_.reset();
        node->unique_id_ = "dynamic:" + std::to_string(unique_index++) + ":" + name;
    }
    for (int i = 0; i < 8; ++i)
    {
        if (octree_node.children[i])
        {
            node->children_[i] = createPotreeNode(*octree_node.children[i], meta_data, name + std::to_string(i), node,
                                                  unique_index, has_color);
        }
    }
    return node;
}

std::vector<StreamingCloudLoader::DynamicPoint> StreamingCloudLoader::downsamplePoints(
    const std::vector<DynamicPoint>& input, std::size_t max_points)
{
    if (max_points == 0 || input.size() <= max_points)
        return input;
    std::vector<DynamicPoint> result;
    result.reserve(max_points);
    double step = static_cast<double>(input.size()) / static_cast<double>(max_points);
    double index = 0.0;
    for (std::size_t i = 0; i < max_points; ++i)
    {
        std::size_t idx = static_cast<std::size_t>(std::floor(index));
        if (idx >= input.size())
            idx = input.size() - 1;
        result.push_back(input[idx]);
        index += step;
    }
    return result;
}

StreamingCloudLoader::StreamingCloudLoader(const std::shared_ptr<CloudMetaData>& meta_data,
                                           const std::shared_ptr<PotreeNode>& root_node)
    : meta_data_(meta_data), root_node_(root_node)
{
}

std::shared_ptr<StreamingCloudLoader>
StreamingCloudLoader::fromPointCloud(const sensor_msgs::msg::PointCloud& msg, std::size_t max_points_per_node,
                                     std::size_t max_depth)
{
    if (msg.points.empty())
        throw std::runtime_error("incoming point cloud contains no points");
    if (max_depth == 0)
        throw std::runtime_error("maximum depth must be greater than zero");

    const std::size_t point_count = msg.points.size();

    int rgb_channel = -1, rgba_channel = -1, r_channel = -1, g_channel = -1, b_channel = -1, a_channel = -1;
    for (std::size_t i = 0; i < msg.channels.size(); ++i)
    {
        const sensor_msgs::msg::ChannelFloat32& channel = msg.channels[i];
        if (channel.values.size() != point_count)
            continue;
        if (channel.name == "rgb")
            rgb_channel = static_cast<int>(i);
        else if (channel.name == "rgba")
            rgba_channel = static_cast<int>(i);
        else if (channel.name == "r")
            r_channel = static_cast<int>(i);
        else if (channel.name == "g")
            g_channel = static_cast<int>(i);
        else if (channel.name == "b")
            b_channel = static_cast<int>(i);
        else if (channel.name == "a")
            a_channel = static_cast<int>(i);
    }

    bool has_packed_color = rgba_channel >= 0 || rgb_channel >= 0;
    bool has_separate_color = r_channel >= 0 && g_channel >= 0 && b_channel >= 0;
    bool has_color = has_packed_color || has_separate_color;

    std::vector<DynamicPoint> points;
    points.reserve(point_count);
    Ogre::Vector3 min_point{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max()};
    Ogre::Vector3 max_point{-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                            -std::numeric_limits<float>::max()};

    std::size_t point_index = 0;
    for (std::size_t idx = 0; idx < point_count; ++idx)
    {
        const geometry_msgs::msg::Point32& p = msg.points[idx];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        DynamicPoint point;
        point.position = Ogre::Vector3{p.x, p.y, p.z};
        float r = 1.f, g = 1.f, b = 1.f, a = 1.f;
        if (has_color)
        {
            if (has_packed_color)
            {
                int channel_index = rgba_channel >= 0 ? rgba_channel : rgb_channel;
                float packed_value = msg.channels[channel_index].values[idx];
                std::uint32_t rgba_value = 0;
                static_assert(sizeof(float) == sizeof(std::uint32_t), "float and uint32_t must match");
                std::memcpy(&rgba_value, &packed_value, sizeof(std::uint32_t));
                std::uint8_t bytes[4];
                std::memcpy(bytes, &rgba_value, sizeof(bytes));
                // Assume layout B,G,R,A as used by common RGB-packed clouds
                b = bytes[0] / 255.f;
                g = bytes[1] / 255.f;
                r = bytes[2] / 255.f;
                if (rgba_channel >= 0)
                    a = bytes[3] / 255.f;
            }
            else if (has_separate_color)
            {
                auto channel_value = [&](int channel_idx) -> float {
                    if (channel_idx < 0)
                        return 1.f;
                    float value = msg.channels[channel_idx].values[idx];
                    if (!std::isfinite(value))
                        return 1.f;
                    if (value > 1.f)
                        return std::clamp(value / 255.f, 0.f, 1.f);
                    return std::clamp(value, 0.f, 1.f);
                };
                r = channel_value(r_channel);
                g = channel_value(g_channel);
                b = channel_value(b_channel);
                if (a_channel >= 0)
                    a = channel_value(a_channel);
            }
        }
        point.color = Ogre::ColourValue{r, g, b, a};
        min_point.makeFloor(point.position);
        max_point.makeCeil(point.position);
        point.index = point_index++;
        points.push_back(std::move(point));
    }

    if (points.empty())
        throw std::runtime_error("incoming point cloud has no finite points");

    Ogre::Vector3 extents = max_point - min_point;
    const float epsilon = 1e-3f;
    if (extents.x < epsilon)
    {
        max_point.x += epsilon;
        min_point.x -= epsilon;
    }
    if (extents.y < epsilon)
    {
        max_point.y += epsilon;
        min_point.y -= epsilon;
    }
    if (extents.z < epsilon)
    {
        max_point.z += epsilon;
        min_point.z -= epsilon;
    }
    Ogre::AxisAlignedBox bounds{min_point, max_point};

    if (max_points_per_node == 0)
        max_points_per_node = 1;

    auto octree = buildOctree(points, bounds, max_points_per_node, max_depth);
    std::size_t unique_index = 0;
    std::vector<CloudMetaData::PointAttribute> attributes;
    attributes.push_back({"POSITION_CARTESIAN", 12, 3, 4, CloudMetaData::PointAttribute::Float});
    std::size_t point_byte_size = sizeof(float) * 3;
    if (has_color)
    {
        attributes.push_back({"COLOR_PACKED", 4, 4, 1, CloudMetaData::PointAttribute::UInt});
        point_byte_size += 4;
    }

    float max_extent = std::max({extents.x, extents.y, extents.z});
    float spacing = max_extent;
    if (max_depth > 0)
        spacing = max_extent / std::pow(2.f, static_cast<float>(max_depth));
    if (!std::isfinite(spacing) || spacing <= 0.f)
        spacing = 1.f;

    std::shared_ptr<CloudMetaData> meta_data =
        std::make_shared<CloudMetaData>(bounds, spacing, attributes, point_byte_size, points.size());
    std::shared_ptr<PotreeNode> root =
        createPotreeNode(*octree, meta_data, "", std::weak_ptr<PotreeNode>(), unique_index, has_color);

    return std::shared_ptr<StreamingCloudLoader>(new StreamingCloudLoader(meta_data, root));
}

std::shared_ptr<const CloudMetaData> StreamingCloudLoader::metaData() const
{
    return meta_data_;
}

std::shared_ptr<PotreeNode> StreamingCloudLoader::loadHierarchy() const
{
    return root_node_;
}

std::size_t StreamingCloudLoader::estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const
{
    return node->pointCount();
}

void StreamingCloudLoader::loadPoints(const std::shared_ptr<PotreeNode>& node, bool recursive) const
{
    (void)node;
    (void)recursive;
    /* no-op: points are already in memory */
}

}  // namespace fkie_potree_rviz_plugin
