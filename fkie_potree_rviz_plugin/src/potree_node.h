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
#ifndef SRC_POTREE_NODE_H_
#define SRC_POTREE_NODE_H_

#include <array>
#include <memory>
#include <mutex>
#include <OgreAxisAlignedBox.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>

namespace Ogre
{
class ManualObject;
class SceneNode;
}

namespace fkie_potree_rviz_plugin
{

class CloudMetaData;
class CloudLoader;

class PotreeNode
{
public:
    PotreeNode(const std::string& name, const std::shared_ptr<CloudMetaData>& meta_data, const Ogre::AxisAlignedBox& bounding_box, const std::weak_ptr<PotreeNode>& parent = std::weak_ptr<PotreeNode>());
    ~PotreeNode();

    const std::string& name() const { return name_; }
    std::size_t level() const { return name_.length(); }
    const Ogre::AxisAlignedBox& boundingBox() const { return bounding_box_; }
    const std::weak_ptr<PotreeNode>& parent() const { return parent_; }
    const std::array<std::shared_ptr<PotreeNode>, 8>& children() const { return children_; }
    bool isLoaded() const { std::lock_guard<std::mutex> lock{mutex_}; return loaded_; }
    bool hasVertexBuffer() const { std::lock_guard<std::mutex> lock{mutex_}; return !!vertex_data_; };
    bool isAttached() const { std::lock_guard<std::mutex> lock{mutex_}; return attached_scene_ != nullptr; }
    bool isVisible() const;
    std::size_t pointCount() const { std::lock_guard<std::mutex> lock{mutex_}; return point_count_; }
    void enableHQRendering(bool enable, bool use_shading, bool recursive = false);
    void unload(bool recursive = false);
    void setVisible(bool visible, bool recursive = false);
    void createVertexBuffer();
    void setPointSize (float size, bool recursive = false);
    void updateShaderParameters (float size_per_pixel, bool is_ortho_projection, float z_scale);
    void attachToScene(Ogre::SceneNode* scene, bool recursive = false);
    void detachFromScene(bool recursive = false);

private:
    friend class CloudLoader;
    std::string getMaterial();

    mutable std::mutex mutex_;
    std::string name_;
    std::shared_ptr<CloudMetaData> meta_data_;
    Ogre::AxisAlignedBox bounding_box_;
    std::weak_ptr<PotreeNode> parent_;
    bool loaded_, hq_render_, use_shading_;
    float point_size_;
    std::array<std::shared_ptr<PotreeNode>, 8> children_;
    std::shared_ptr<Ogre::ManualObject> vertex_data_;
    std::size_t point_count_;
    Ogre::SceneNode* attached_scene_;
    std::string unique_id_;
    std::vector<Ogre::Vector3> points_;
    std::vector<Ogre::ColourValue> colors_;
};

} // namespace fkie_rviz_plugin_potree

#endif /* SRC_POTREE_NODE_H_ */
