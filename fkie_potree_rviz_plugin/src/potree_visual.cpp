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
#include "potree_visual.h"

#include "cloud_loader.h"
#include "loading_thread.h"
#include "potree_node.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <ros/console.h>

#include <cmath>

namespace fkie_potree_rviz_plugin
{

namespace
{

bool is_different(const Ogre::Matrix4& m1, const Ogre::Matrix4& m2,
                  double eps = 1e-3)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (std::abs(m1[i][j] - m2[i][j]) > eps)
                return true;
        }
    }
    return false;
}

}  // namespace

PotreeVisual::PotreeVisual(const std::shared_ptr<CloudLoader>& loader,
                           Ogre::SceneManager* scene_manager,
                           Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      scene_node_(
          (parent_node ? parent_node : scene_manager->getRootSceneNode())
              ->createChildSceneNode()),
      loader_(loader), root_node_(loader_->loadHierarchy()),
      loading_thread_(std::make_shared<LoadingThread>(loader_))
{
    scene_manager_->addListener(this);
    loading_thread_->setNodeLoadedCallback(
        std::bind(&PotreeVisual::onNodeLoaded, this, std::placeholders::_1));
}

PotreeVisual::~PotreeVisual()
{
    scene_manager_->removeListener(this);
    // We delete everything explicitly that might access the scene node
    loaded_.clear();
    loading_thread_.reset();
    root_node_.reset();
    // Now that everything is detached from the scene, we can safely delete the
    // scene node itself
    scene_manager_->destroySceneNode(scene_node_);
}

void PotreeVisual::setVisible(bool visible)
{
    scene_node_->setVisible(visible);
}

void PotreeVisual::setOrigin(const Ogre::Vector3& p, const Ogre::Quaternion& q)
{
    scene_node_->setPosition(p);
    scene_node_->setOrientation(q);
    appearance_changed_ = true;
}

void PotreeVisual::setPointBudget(std::size_t budget)
{
    point_budget_ = budget;
    appearance_changed_ = true;
}

void PotreeVisual::setPointSize(float size)
{
    point_size_ = size;
    appearance_changed_ = true;
}

void PotreeVisual::enableSplatRendering(bool enable)
{
    splat_rendering_ = enable;
    appearance_changed_ = true;
}

void PotreeVisual::setMinimumNodeSize(float size)
{
    minimum_node_size_ = size;
    appearance_changed_ = true;
}

void PotreeVisual::onNodeLoaded(const std::shared_ptr<PotreeNode>&)
{
    force_update_ = true;
}

void PotreeVisual::preFindVisibleObjects(
    Ogre::SceneManager*, Ogre::SceneManager::IlluminationRenderStage irs,
    Ogre::Viewport* viewport)
{
    if (irs != Ogre::SceneManager::IRS_NONE)
        return;
    if (viewport->getActualHeight() < 10)
        return; /* weird stuff going on here */
    Ogre::Camera* cam = viewport->getCamera();
    Ogre::Vector3 cam_pos = cam->getRealPosition();
    Ogre::Quaternion cam_ori = cam->getRealOrientation();
    Ogre::Matrix4 proj_matrix = cam->getProjectionMatrix();
    /* No need to update anything if the camera did not move */
    if (!force_update_ && !appearance_changed_
        && !is_different(proj_matrix, last_proj_matrix_)
        && (cam_pos - last_cam_pos_).length() < 0.01f
        && (cam_ori - last_cam_ori_).Norm() < 0.01f)
        return;
    last_cam_pos_ = cam_pos;
    last_cam_ori_ = cam_ori;
    last_proj_matrix_ = proj_matrix;
    force_update_ = false;
    if (appearance_changed_)
    {
        root_node_->setPointSize(point_size_, true);
        root_node_->enableSplatRendering(splat_rendering_, true);
        appearance_changed_ = false;
    }
    PriorityQueue<std::shared_ptr<PotreeNode>, float> process_queue;
    std::vector<std::shared_ptr<PotreeNode>> active_nodes;
    process_queue.push(root_node_, 0);
    std::size_t remaining_points = point_budget_;
    std::size_t remaining_nodes = node_budget_;
    Ogre::Matrix4 world = scene_node_->_getFullTransform();
    float lowest_spacing = std::numeric_limits<float>::max();
    while (!process_queue.empty())
    {
        std::shared_ptr<PotreeNode> node = process_queue.top();
        process_queue.pop();
        Ogre::AxisAlignedBox bb = node->boundingBox();
        bb.transform(world);
        if (cam->isVisible(bb))
        {
            std::size_t point_count = node->pointCount();
            if (point_count == 0)
                point_count = loader_->estimatedPointCount(node);
            if (point_count <= remaining_points && remaining_nodes > 0)
            {
                remaining_points -= point_count;
                --remaining_nodes;
                if (node->isLoaded())
                {
                    if (!node->hasVertexBuffer())
                        node->createVertexBuffer();
                    if (!node->isAttached())
                        node->attachToScene(scene_node_);
                    node->setVisible(true);
                    float spacing = node->spacing();
                    if (spacing < lowest_spacing)
                        lowest_spacing = spacing;
                    active_nodes.push_back(node);
                    updateLRU(node);
                    for (const std::shared_ptr<PotreeNode>& child :
                         node->children())
                    {
                        if (child)
                        {
                            float p = priority(child, world, viewport);
                            if (p > 0)
                            {
                                process_queue.push(child, p);
                            }
                            else
                            {
                                child->setVisible(false, true);
                            }
                        }
                    }
                }
                else
                {
                    loading_thread_->scheduleForLoading(node);
                }
            }
            else
            {
                node->setVisible(false, true);
            }
        }
        else
        {
            node->setVisible(false, true);
        }
    }
    for (std::shared_ptr<PotreeNode>& node : active_nodes)
    {
        node->updateShaderParameters(
            cam->getProjectionType() == Ogre::ProjectionType::PT_ORTHOGRAPHIC,
            lowest_spacing);
    }
    unloadUnused();
}

float PotreeVisual::priority(const std::shared_ptr<PotreeNode>& node,
                             const Ogre::Matrix4& world,
                             Ogre::Viewport* viewport) const
{
    Ogre::Camera* cam = viewport->getCamera();
    Ogre::AxisAlignedBox bb = node->boundingBox();
    bb.transform(world);
    Ogre::Vector3 center = bb.getCenter();
    Ogre::Vector3 cam_pos = cam->getRealPosition();
    float bounding_radius = node->boundingBox().getHalfSize().length();
    float projected_size;
    if (cam->getProjectionType() == Ogre::ProjectionType::PT_PERSPECTIVE)
    {
        float slope = std::tan(0.5 * cam->getFOVy().valueRadians());
        float distance = (center - cam_pos).length();
        projected_size = 0.5f * viewport->getActualHeight() * bounding_radius
                         / (slope * distance);
        if (projected_size < minimum_node_size_)
            return -1; /* ignore */
        if (distance - bounding_radius < 0)
            return std::numeric_limits<float>::max(); /* camera is inside,
                                                         prioritize */
    }
    else
    {
        projected_size = bounding_radius;
    }
    return projected_size;
}

void PotreeVisual::updateLRU(const std::shared_ptr<PotreeNode>& node)
{
    auto it = load_map_.find(node.get());
    loaded_.push_front(node);
    if (it != load_map_.end())
        loaded_.erase(it->second);
    load_map_[node.get()] = loaded_.begin();
}

void PotreeVisual::unloadUnused()
{
    std::size_t total = loaded_.size();
    if (total <= node_budget_)
        return;
    std::size_t excess = total - node_budget_;
    while (excess > 0)
    {
        auto it = --loaded_.end();
        load_map_.erase(it->get());
        (*it)->unload(true);
        loaded_.erase(it);
        excess--;
    }
}

}  // namespace fkie_potree_rviz_plugin
