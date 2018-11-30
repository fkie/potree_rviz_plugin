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
#ifndef SRC_CLOUD_LOADER_H_
#define SRC_CLOUD_LOADER_H_

#include <boost/filesystem/path.hpp>
#include "cloud_meta_data.h"

namespace fkie_potree_rviz_plugin
{

namespace fs = boost::filesystem;

class PotreeNode;

class CloudLoader
{
public:
    CloudLoader(const fs::path& path);
    const std::shared_ptr<CloudMetaData>& metaData() const { return meta_data_; }
    std::shared_ptr<PotreeNode> loadHierarchy() const;
    void loadPoints(const std::shared_ptr<PotreeNode>& node, bool recursive = false) const;

    static bool isValid(const fs::path& path, std::string& error_msg);
    static fs::path fileName(const std::shared_ptr<CloudMetaData>& meta_data, const std::string& name, const std::string& extension);

private:
    void loadNodeHierarchy(const std::shared_ptr<PotreeNode>& root_node) const;
    static Ogre::AxisAlignedBox childBB(const Ogre::AxisAlignedBox& parent, int index);

    std::shared_ptr<CloudMetaData> meta_data_;
};

} // namespace fkie_rviz_plugin_potree

#endif /* SRC_CLOUD_LOADER_H_ */
