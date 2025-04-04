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
#ifndef SRC_FS_PATH_PROPERTY_H_
#define SRC_FS_PATH_PROPERTY_H_

#include <rviz_common/properties/string_property.hpp>

#include <filesystem>

namespace fkie_potree_rviz_plugin
{

/** @brief Property specialized for filesystem path values. */
class FsPathProperty : public rviz_common::properties::StringProperty
{
public:
    FsPathProperty(const QString& name = QString(), const QString& default_value = QString(),
                   const QString& description = QString(), Property* parent = nullptr,
                   const char* changed_slot = nullptr, QObject* receiver = nullptr);
    std::filesystem::path getFsPath() const;
    virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem&) override;
};

}  // namespace fkie_potree_rviz_plugin

#endif
