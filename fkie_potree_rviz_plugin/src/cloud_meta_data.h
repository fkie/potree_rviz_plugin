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
#ifndef SRC_CLOUD_META_DATA_H_
#define SRC_CLOUD_META_DATA_H_

#include <string>
#include <boost/filesystem/path.hpp>
#include <OgreAxisAlignedBox.h>

namespace fkie_potree_rviz_plugin
{

class CloudLoader;

namespace fs = boost::filesystem;

class CloudMetaData
{
public:
    std::size_t pointCount() const { return point_count_; }
    void readFromJson(const fs::path& file_name);
    static std::size_t sizeOf(const std::string& attr);

private:
    friend class CloudLoader;
    fs::path octree_dir_;
    fs::path cloud_path_;
    std::size_t point_count_ = 0;
    std::size_t hierarchy_step_size_ = 0;
    std::size_t point_byte_size_;
    Ogre::AxisAlignedBox bounding_box_;
    float spacing_ = 0;
    float scale_ = 0;
    std::vector<std::string> point_attributes_;
};

} // namespace fkie_rviz_plugin_potree

#endif /* SRC_CLOUD_META_DATA_H_ */
