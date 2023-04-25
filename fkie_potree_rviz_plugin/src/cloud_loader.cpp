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

#include "cloud_loader.h"

#include "cloud_loader_1.h"
#include "cloud_loader_2.h"

#include <boost/filesystem.hpp>

#include <stdexcept>

namespace fkie_potree_rviz_plugin
{

std::shared_ptr<CloudLoader> CloudLoader::create(const fs::path& path)
{
    if (!fs::is_directory(path))
        throw std::runtime_error("not an existing folder");

    std::shared_ptr<CloudMetaData> meta_data;
    if (fs::is_regular(path / "cloud.js"))
    {
        meta_data = std::make_shared<CloudMetaData>(path / "cloud.js");
    }
    else if (fs::is_regular(path / "metadata.json"))
    {
        meta_data = std::make_shared<CloudMetaData>(path / "metadata.json");
    }

    if (meta_data)
    {
        switch (meta_data->version())
        {
            case CloudMetaData::POTREE_1_X:
                return std::make_shared<CloudLoader1>(meta_data);
            case CloudMetaData::POTREE_2_X:
                return std::make_shared<CloudLoader2>(meta_data);
            default:
                throw std::runtime_error("unsupported Potree format");
        }
    }
    throw std::runtime_error("not a Potree folder");
}

Ogre::AxisAlignedBox CloudLoader::childBB(const Ogre::AxisAlignedBox& parent,
                                          int index)
{
    assert(!parent.isInfinite());
    Ogre::Vector3 min = parent.getMinimum(), max = parent.getMaximum(),
                  half_size = parent.getHalfSize();
    if (index & 1)
        min.z += half_size.z;
    else
        max.z -= half_size.z;
    if (index & 2)
        min.y += half_size.y;
    else
        max.y -= half_size.y;
    if (index & 4)
        min.x += half_size.x;
    else
        max.x -= half_size.x;
    return Ogre::AxisAlignedBox(min, max);
}

}  // namespace fkie_potree_rviz_plugin
