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
#include "potree_visual.h"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include "cloud_loader.h"
#include "loading_thread.h"
#include "potree_node.h"
#include <cmath>
#include <ros/console.h>

namespace fkie_potree_rviz_plugin
{

namespace
{

bool is_different (const Ogre::Matrix4& m1, const Ogre::Matrix4& m2, double eps = 1e-3)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (std::abs(m1[i][j] - m2[i][j]) > eps) return true;
        }
    }
    return false;
}

}

PotreeVisual::PotreeVisual(const std::shared_ptr<CloudLoader>& loader, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
: scene_manager_(scene_manager),
  scene_node_((parent_node ? parent_node : scene_manager->getRootSceneNode())->createChildSceneNode()),
  loader_(loader), root_node_(loader_->loadHierarchy()), loading_thread_(std::make_shared<LoadingThread>(loader_))
{
    scene_manager_->addListener(this);
    loading_thread_->setNodeLoadedCallback(std::bind(&PotreeVisual::onNodeLoaded, this));
}

PotreeVisual::~PotreeVisual()
{
    scene_manager_->removeListener(this);
    // We delete everything explicitly that might access the scene node
    root_node_.reset();
    loading_thread_.reset();
    // Now that everything is detached from the scene, we can safely delete the scene node itself
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

void PotreeVisual::enableHQRendering(bool enable, bool use_shading)
{
    hq_render_ = enable;
    use_shading_ = use_shading;
    appearance_changed_ = true;
}

void PotreeVisual::setMinimumNodeSize (float size)
{
    minimum_node_size_ = size;
    appearance_changed_ = true;
}

void PotreeVisual::onNodeLoaded()
{
    force_update_ = true;
}

void PotreeVisual::preFindVisibleObjects(Ogre::SceneManager*, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport* viewport)
{
    if (irs != Ogre::SceneManager::IRS_NONE) return;
    if (viewport->getActualHeight() < 10) return; /* weird stuff going on here */
    Ogre::Camera* cam = viewport->getCamera();
    Ogre::Vector3 cam_pos = cam->getRealPosition();
    Ogre::Quaternion cam_ori = cam->getRealOrientation();
    Ogre::Matrix4 proj_matrix = cam->getProjectionMatrix();
    /* No need to update anything if the camera did not move */
    if (!force_update_ && !appearance_changed_ && !is_different(proj_matrix, last_proj_matrix_) && (cam_pos - last_cam_pos_).length() < 0.01f && (cam_ori - last_cam_ori_).Norm() < 0.01f) return;
    last_cam_pos_ = cam_pos;
    last_cam_ori_ = cam_ori;
    last_proj_matrix_ = proj_matrix;
    force_update_ = false;
    if (appearance_changed_)
    {
        root_node_->setPointSize(point_size_, true);
        root_node_->enableHQRendering(hq_render_, use_shading_, true);
        appearance_changed_ = false;
    }
    PriorityQueue<std::shared_ptr<PotreeNode>, float> process_queue;
    process_queue.push(root_node_, 0);
    std::size_t remaining_points = point_budget_;
    float size_per_pixel = std::tan(cam->getFOVy().valueRadians()) / viewport->getActualHeight();
    float z_scale = std::sqrt(proj_matrix[0][0] * proj_matrix[0][0] + proj_matrix[1][0] * proj_matrix[1][0] + proj_matrix[2][0] * proj_matrix[2][0]);
    Ogre::Matrix4 world = scene_node_->_getFullTransform();
    while (!process_queue.empty())
    {
        std::shared_ptr<PotreeNode> node = process_queue.top(); process_queue.pop();
        Ogre::AxisAlignedBox bb = node->boundingBox();
        bb.transform(world);
        if (cam->isVisible(bb))
        {
            if (node->isLoaded())
            {
                if (node->pointCount() <= remaining_points)
                {
                    if (!node->hasVertexBuffer()) node->createVertexBuffer();
                    if (!node->isAttached()) node->attachToScene(scene_node_);
                    remaining_points -= node->pointCount();
                    node->updateShaderParameters(size_per_pixel, cam->getProjectionType() == Ogre::ProjectionType::PT_PERSPECTIVE, z_scale);
                    node->setVisible(true);
                    for (const std::shared_ptr<PotreeNode>& child : node->children())
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
                    node->setVisible(false, true);
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
}

float PotreeVisual::priority(const std::shared_ptr<PotreeNode>& node, const Ogre::Matrix4& world, Ogre::Viewport* viewport) const
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
        float slope = std::tan(cam->getFOVy().valueRadians());
        float distance = (center - cam_pos).length();
        projected_size = 0.5f * viewport->getActualHeight() * bounding_radius / (slope * distance);
        if (projected_size < minimum_node_size_) return -1; /* ignore */
    }
    else
    {
        projected_size = bounding_radius;
    }
    Ogre::Vector3 cam_forward = cam->getRealDirection();
    Ogre::Vector3 cam_to_node = (center - cam_pos).normalisedCopy();
    float angle = std::acos(cam_forward.x * cam_to_node.x + cam_forward.y * cam_to_node.y + cam_forward.z * cam_to_node.z);
    float angle_weight = std::abs(angle) + 1;
    return projected_size / angle_weight;
}

} // namespace fkie_rviz_plugin_potree
