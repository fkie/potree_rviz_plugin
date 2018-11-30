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
#ifndef SRC_POTREE_VISUAL_H_
#define SRC_POTREE_VISUAL_H_

#include <memory>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include "priority_queue.h"
#include <ros/time.h>

namespace Ogre
{
class SceneNode;
class Camera;
}

namespace fkie_potree_rviz_plugin
{

class PotreeNode;
class CloudLoader;
class LoadingThread;

class PotreeVisual : protected Ogre::SceneManager::Listener
{
public:
    PotreeVisual(const std::shared_ptr<CloudLoader>& loader, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
    ~PotreeVisual();
    void setVisible(bool visible);
    void setOrigin(const Ogre::Vector3& p, const Ogre::Quaternion& q);
    void setPointBudget(std::size_t budget);
    void setPointSize (float size);
    void setMinimumNodeSize(float size);
    void enableHQRendering (bool enable, bool use_shading);
protected:
    virtual void preFindVisibleObjects(Ogre::SceneManager* source, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport* viewport) override;
private:
    void onNodeLoaded();
    float priority (const std::shared_ptr<PotreeNode>& node, const Ogre::Matrix4& world, Ogre::Viewport* viewport) const;
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;
    Ogre::Vector3 last_cam_pos_;
    Ogre::Matrix4 last_proj_matrix_;
    Ogre::Quaternion last_cam_ori_;
    std::size_t point_budget_ = 0;
    float minimum_node_size_ = 30.f;
    float point_size_ = 1.f;
    bool appearance_changed_ = false;
    bool force_update_ = false;
    bool hq_render_ = false;
    bool use_shading_ = true;
    std::shared_ptr<CloudLoader> loader_;
    std::shared_ptr<PotreeNode> root_node_;
    std::shared_ptr<LoadingThread> loading_thread_;
};

} // namespace fkie_rviz_plugin_potree

#endif
