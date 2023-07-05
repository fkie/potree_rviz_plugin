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
#ifndef SRC_CLOUD_META_DATA_H_
#define SRC_CLOUD_META_DATA_H_

#include <OgreAxisAlignedBox.h>
#include <boost/filesystem/path.hpp>

#include <string>
#include <vector>

namespace Json
{

class Value;

}

namespace fkie_potree_rviz_plugin
{

class CloudLoader1;
class CloudLoader2;

namespace fs = boost::filesystem;

class CloudMetaData
{
public:
    static constexpr unsigned POTREE_1_X = 0x0100;
    static constexpr unsigned POTREE_2_X = 0x0200;

    struct PointAttribute
    {
        enum Type
        {
            None,
            UInt,
            Int,
            Float
        };
        std::string name;
        std::size_t size;
        std::size_t num_elements;
        std::size_t element_size;
        Type type;
    };

    CloudMetaData(const fs::path& file_name);

    std::size_t pointCount() const
    {
        return point_count_;
    }
    float spacing() const
    {
        return spacing_;
    }
    unsigned version() const
    {
        return version_;
    }
    const std::vector<PointAttribute>& pointAttributes() const
    {
        return point_attributes_;
    }

private:
    void parsePotree1(Json::Value& data);
    void parsePotree2(Json::Value& data);

    friend class CloudLoader1;
    friend class CloudLoader2;

    fs::path octree_dir_;
    fs::path cloud_path_;
    unsigned version_ = 0;
    std::size_t point_count_ = 0;
    std::size_t hierarchy_step_size_ = 0;
    std::size_t point_byte_size_ = 0;
    std::size_t hierarchy_root_size_ = 0;
    std::size_t hierarchy_node_size_ = 22;
    Ogre::AxisAlignedBox bounding_box_;
    float spacing_ = 0;
    std::array<float, 3> scale_ = {1.f, 1.f, 1.f};
    std::array<float, 3> offset_ = {0, 0, 0};
    std::vector<PointAttribute> point_attributes_;
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_CLOUD_META_DATA_H_ */
