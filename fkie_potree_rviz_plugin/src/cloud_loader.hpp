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
#ifndef SRC_CLOUD_LOADER_H_
#define SRC_CLOUD_LOADER_H_

#include "cloud_meta_data.hpp"

#include <OgreAxisAlignedBox.h>

#include <memory>
#include <filesystem>

namespace fkie_potree_rviz_plugin
{

namespace fs = std::filesystem;

class PotreeNode;

class CloudLoader
{
public:
    virtual std::shared_ptr<const CloudMetaData> metaData() const = 0;
    virtual std::shared_ptr<PotreeNode> loadHierarchy() const = 0;
    virtual std::size_t estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const = 0;
    virtual void loadPoints(const std::shared_ptr<PotreeNode>& node, bool recursive = false) const = 0;

    static std::shared_ptr<CloudLoader> create(const fs::path& path);

protected:
    static Ogre::AxisAlignedBox childBB(const Ogre::AxisAlignedBox& parent, int index);
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_CLOUD_LOADER_H_ */
