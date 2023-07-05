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
#include "fs_path_property.h"

#include "fs_path_editor.h"

namespace fkie_potree_rviz_plugin
{

FsPathProperty::FsPathProperty(const QString& name,
                               const QString& default_value,
                               const QString& description, Property* parent,
                               const char* changed_slot, QObject* receiver)
    : rviz::StringProperty(name, default_value, description, parent,
                           changed_slot, receiver)
{
}

boost::filesystem::path FsPathProperty::getFsPath() const
{
    return boost::filesystem::path(getValue().toString().toStdString());
}

QWidget* FsPathProperty::createEditor(QWidget* parent,
                                      const QStyleOptionViewItem&)
{
    FsPathEditor* editor = new FsPathEditor(this, parent);
    editor->setFrame(false);
    return editor;
}

}  // namespace fkie_potree_rviz_plugin
