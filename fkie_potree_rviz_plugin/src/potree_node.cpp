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

#include "potree_node.h"

#include "cloud_meta_data.h"

#include <OgreGpuProgramParams.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <ros/console.h>

namespace fkie_potree_rviz_plugin
{

PotreeNode::PotreeNode(const std::string& name,
                       const std::shared_ptr<CloudMetaData>& meta_data,
                       const Ogre::AxisAlignedBox& bounding_box,
                       const std::weak_ptr<PotreeNode>& parent)
    : name_(name), meta_data_(meta_data), bounding_box_(bounding_box),
      parent_(parent)
{
}

PotreeNode::~PotreeNode()
{
    detachFromScene();
}

void PotreeNode::createVertexBuffer()
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (!loaded_ || point_count_ == 0 || vertex_data_)
        return;
    vertex_data_ = std::make_shared<Ogre::ManualObject>(unique_id_);
    vertex_data_->estimateVertexCount(point_count_);
    vertex_data_->begin(getMaterial(), Ogre::RenderOperation::OT_POINT_LIST);
    for (std::size_t i = 0; i < point_count_; ++i)
    {
        vertex_data_->position(points_[i]);
        if (!colors_.empty())
            vertex_data_->colour(colors_[i]);
    }
    vertex_data_->end();
    points_.clear();
    colors_.clear();
}

void PotreeNode::enableSplatRendering(bool enable, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (enable != splat_rendering_)
    {
        splat_rendering_ = enable;
        if (vertex_data_)
        {
            vertex_data_->setMaterialName(0, getMaterial());
        }
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child)
                child->enableSplatRendering(enable, true);
        }
    }
}

void PotreeNode::setPointSize(float point_size, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    point_size_ = point_size;
    if (vertex_data_)
    {
        vertex_data_->setMaterialName(0, getMaterial());
    }
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child)
                child->setPointSize(point_size, true);
        }
    }
}

float PotreeNode::spacing() const
{
    return meta_data_->spacing() / (1 << name_.length());
}

void PotreeNode::updateShaderParameters(bool is_ortho_projection, float spacing)
{
    if (vertex_data_)
    {
        if (vertex_data_->getNumSections() > 0)
        {
            Ogre::Technique* technique =
                vertex_data_->getSection(0)->getMaterial()->getTechnique(0);
            for (std::size_t i = 0; i < technique->getNumPasses(); ++i)
            {
                Ogre::Pass* pass = technique->getPass(i);
                if (pass && pass->hasVertexProgram())
                {
                    Ogre::GpuProgramParametersSharedPtr params =
                        pass->getVertexProgramParameters();
                    params->setNamedConstant("is_ortho_projection",
                                             is_ortho_projection ? 1 : 0);
                    params->setNamedConstant("spacing", spacing);
                    params->setNamedConstant("splat_size", point_size_);
                }
            }
        }
    }
}

void PotreeNode::attachToScene(Ogre::SceneNode* scene, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (attached_scene_ && vertex_data_)
        attached_scene_->detachObject(vertex_data_.get());
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
            if (child)
                child->attachToScene(scene, true);
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
            if (child)
                child->detachFromScene(true);
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
    loaded_ = false;
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child)
                child->unload(true);
        }
    }
}

bool PotreeNode::isVisible() const
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (vertex_data_)
        return vertex_data_->isVisible();
    return false;
}

void PotreeNode::setVisible(bool visible, bool recursive)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (vertex_data_)
        vertex_data_->setVisible(visible);
    if (recursive)
    {
        for (const std::shared_ptr<PotreeNode>& child : children_)
        {
            if (child)
                child->setVisible(visible, true);
        }
    }
}

std::string PotreeNode::getMaterial()
{
    if (splat_rendering_)
    {
        Ogre::MaterialPtr m = Ogre::MaterialManager::getSingleton().getByName(
            "potree_splat",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        if (!m.isNull())
            return "potree_splat";
    }
    std::string material = "potree_point" + std::to_string(point_size_);
    Ogre::MaterialPtr m =
        Ogre::MaterialManager::getSingleton().getByName(material);
    if (m.isNull())
    {
        Ogre::MaterialPtr m0 = Ogre::MaterialManager::getSingleton().getByName(
            "potree_point",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        m = m0->clone(material);
        m->getTechnique(0)->getPass(0)->setPointSize(point_size_);
    }
    return material;
}

}  // namespace fkie_potree_rviz_plugin
