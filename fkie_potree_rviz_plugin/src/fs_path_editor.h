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
#ifndef SRC_FS_PATH_EDITOR_H_
#define SRC_FS_PATH_EDITOR_H_

#include <rviz/properties/line_edit_with_button.h>

namespace fkie_potree_rviz_plugin
{

class FsPathProperty;

class FsPathEditor : public rviz::LineEditWithButton
{
    Q_OBJECT
public:
    FsPathEditor(FsPathProperty* property = nullptr, QWidget* parent = nullptr);

protected Q_SLOTS:
    void onButtonClick() override;

private:
    FsPathProperty* property_;
};

};  // namespace fkie_potree_rviz_plugin

#endif
