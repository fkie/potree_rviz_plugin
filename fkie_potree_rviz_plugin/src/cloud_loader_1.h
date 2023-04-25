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
#ifndef SRC_CLOUD_LOADER_1_H_
#define SRC_CLOUD_LOADER_1_H_

#include "cloud_loader.h"

namespace fkie_potree_rviz_plugin
{

class CloudLoader1 : public CloudLoader
{
public:
    explicit CloudLoader1(const std::shared_ptr<CloudMetaData>& meta_data);
    virtual std::shared_ptr<const CloudMetaData> metaData() const override;
    virtual std::shared_ptr<PotreeNode> loadHierarchy() const override;
    virtual std::size_t
    estimatedPointCount(const std::shared_ptr<PotreeNode>& node) const override;
    virtual void loadPoints(const std::shared_ptr<PotreeNode>& node,
                            bool recursive = false) const override;

private:
    void loadNodeHierarchy(const std::shared_ptr<PotreeNode>& root_node) const;
    static fs::path fileName(const std::shared_ptr<CloudMetaData>& meta_data,
                             const std::string& name,
                             const std::string& extension);

    std::shared_ptr<CloudMetaData> meta_data_;
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_CLOUD_LOADER_1_H_ */
