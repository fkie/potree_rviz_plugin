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
#include "fs_path_editor.h"

#include "fs_path_property.h"

#include <QFileDialog>

namespace fkie_potree_rviz_plugin
{

FsPathEditor::FsPathEditor(FsPathProperty* property, QWidget* parent)
    : rviz::LineEditWithButton(parent), property_(property)
{
}

void FsPathEditor::onButtonClick()
{
    FsPathProperty* prop = property_;
    QString path = prop->getValue().toString();

    deleteLater();

    QString new_path =
        QFileDialog::getExistingDirectory(window(), QString(), path);
    if (!new_path.isNull())
        prop->setString(new_path);
}

}  // namespace fkie_potree_rviz_plugin