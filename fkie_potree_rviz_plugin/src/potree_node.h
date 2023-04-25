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
#ifndef SRC_POTREE_NODE_H_
#define SRC_POTREE_NODE_H_

#include <OgreAxisAlignedBox.h>
#include <OgreColourValue.h>
#include <OgreVector3.h>

#include <array>
#include <memory>
#include <mutex>

namespace Ogre
{
class ManualObject;
class SceneNode;
}  // namespace Ogre

namespace fkie_potree_rviz_plugin
{

class CloudMetaData;
class CloudLoader1;
class CloudLoader2;

class PotreeNode
{
public:
    PotreeNode(
        const std::string& name,
        const std::shared_ptr<CloudMetaData>& meta_data,
        const Ogre::AxisAlignedBox& bounding_box,
        const std::weak_ptr<PotreeNode>& parent = std::weak_ptr<PotreeNode>());
    ~PotreeNode();

    const std::string& name() const
    {
        return name_;
    }
    std::size_t level() const
    {
        return name_.length();
    }
    const Ogre::AxisAlignedBox& boundingBox() const
    {
        return bounding_box_;
    }
    const std::weak_ptr<PotreeNode>& parent() const
    {
        return parent_;
    }
    const std::array<std::shared_ptr<PotreeNode>, 8>& children() const
    {
        return children_;
    }
    bool isLoaded() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return loaded_;
    }
    bool hasVertexBuffer() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return !!vertex_data_;
    };
    bool isAttached() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return attached_scene_ != nullptr;
    }
    bool isVisible() const;
    float spacing() const;
    std::size_t pointCount() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return point_count_;
    }
    void enableSplatRendering(bool enable, bool recursive = false);
    void unload(bool recursive = false);
    void setVisible(bool visible, bool recursive = false);
    void createVertexBuffer();
    void setPointSize(float size, bool recursive = false);
    void updateShaderParameters(bool is_ortho_projection, float spacing);
    void attachToScene(Ogre::SceneNode* scene, bool recursive = false);
    void detachFromScene(bool recursive = false);

private:
    friend class CloudLoader1;
    friend class CloudLoader2;

    std::string getMaterial();

    mutable std::mutex mutex_;
    std::string name_;
    std::shared_ptr<CloudMetaData> meta_data_;
    Ogre::AxisAlignedBox bounding_box_;
    std::weak_ptr<PotreeNode> parent_;
    std::size_t point_count_ = 0;
    bool loaded_ = false, splat_rendering_ = false;
    float point_size_ = 1.f;
    std::array<std::shared_ptr<PotreeNode>, 8> children_;
    std::shared_ptr<Ogre::ManualObject> vertex_data_;
    Ogre::SceneNode* attached_scene_ = nullptr;
    std::string unique_id_;
    std::size_t hierarchy_offset_ = 0, hierarchy_size_ = 0;
    std::size_t data_offset_ = 0, data_size_ = 0;
    std::vector<Ogre::Vector3> points_;
    std::vector<Ogre::ColourValue> colors_;
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_POTREE_NODE_H_ */
