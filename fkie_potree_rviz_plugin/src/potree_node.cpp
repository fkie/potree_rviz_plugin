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

#include "potree_node.h"
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreGpuProgramParams.h>
#include <ros/console.h>

namespace fkie_potree_rviz_plugin
{

PotreeNode::PotreeNode (const std::string& name, const std::shared_ptr<CloudMetaData>& meta_data, const Ogre::AxisAlignedBox& bounding_box, const std::weak_ptr<PotreeNode>& parent)
: name_(name), meta_data_(meta_data), bounding_box_(bounding_box), parent_(parent), loaded_(false), hq_render_(false), point_size_(1), attached_scene_(nullptr)
{
}

PotreeNode::~PotreeNode()
{
    detachFromScene();
}

void PotreeNode::createVertexBuffer()
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (!loaded_ || vertex_data_) return;
    vertex_data_ = std::make_shared<Ogre::ManualObject>(unique_id_);
    vertex_data_->estimateVertexCount(point_count_);
    vertex_data_->begin(getMaterial(), Ogre::RenderOperation::OT_POINT_LIST, "rviz");
    for (std::size_t i = 0; i < point_count_; ++i)
    {
        vertex_data_->position(points_[i]);
        if (!colors_.empty()) vertex_data_->colour(colors_[i]);
    }
    vertex_data_->end();
    points_.clear();
    colors_.clear();

}

void PotreeNode::enableHQRendering(bool enable, bool use_shading, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (enable != hq_render_)
    {
        hq_render_ = enable;
        if (vertex_data_)
        {
            vertex_data_->setMaterialName(0, getMaterial(), "rviz");
        }
    }
    use_shading_ = use_shading;
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->enableHQRendering(enable, use_shading, true);
        }
    }
}

void PotreeNode::setPointSize (float point_size, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    point_size_ = point_size;
    if (vertex_data_)
    {
        vertex_data_->setMaterialName(0, getMaterial(), "rviz");
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->setPointSize(point_size, true);
        }
    }
}

void PotreeNode::updateShaderParameters(float size_per_pixel, bool is_perspective_projection, float z_scale)
{
    if (vertex_data_)
    {
        if (vertex_data_->getNumSections() > 0)
        {
            vertex_data_->getSection(0)->setCustomParameter(0, Ogre::Vector4(point_size_ * size_per_pixel, is_perspective_projection ? 1 : 0, z_scale / point_size_, 0));
            vertex_data_->getSection(0)->setCustomParameter(1, Ogre::Vector4(use_shading_ ? 1 : 0, 0, 0, 0));
        }
    }
}

void PotreeNode::attachToScene(Ogre::SceneNode* scene, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (attached_scene_ && vertex_data_) attached_scene_->detachObject(vertex_data_.get());
    if (scene && vertex_data_)
    {
        scene->attachObject(vertex_data_.get());
        attached_scene_ = scene;
    }
    else
    {
        attached_scene_ = nullptr;
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->attachToScene(scene, true);
        }
    }
}

void PotreeNode::detachFromScene(bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (attached_scene_ && vertex_data_)
        attached_scene_->detachObject(vertex_data_.get());
    attached_scene_ = nullptr;
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->detachFromScene(true);
        }
    }
}

void PotreeNode::unload(bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (attached_scene_ && vertex_data_)
        attached_scene_->detachObject(vertex_data_.get());
    attached_scene_ = nullptr;
    vertex_data_.reset();
    points_.clear();
    colors_.clear();
    point_count_ = 0;
    loaded_ = false;
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->unload(true);
        }
    }
}

bool PotreeNode::isVisible() const
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (vertex_data_) return vertex_data_->isVisible();
    return false;
}

void PotreeNode::setVisible(bool visible, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (vertex_data_) vertex_data_->setVisible(visible);
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child) child->setVisible(visible, true);
        }
    }
}

std::string PotreeNode::getMaterial()
{
    if (hq_render_)
    {
        Ogre::MaterialPtr m = Ogre::MaterialManager::getSingleton().getByName("rviz/potree_splat");
        if (!m.isNull()) return "rviz/potree_splat";
    }
    std::string material = "rviz/potree_point" + std::to_string(point_size_);
    Ogre::MaterialPtr m = Ogre::MaterialManager::getSingleton().getByName(material);
    if (m.isNull())
    {
        Ogre::MaterialPtr m0 = Ogre::MaterialManager::getSingleton().getByName("rviz/potree_point");
        m = m0->clone(material);
        m->getTechnique(0)->getPass(0)->setPointSize(point_size_);
    }
    return material;
}

} // namespace fkie_rviz_plugin_potree
