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
#ifndef SRC_LOADING_THREAD_H_
#define SRC_LOADING_THREAD_H_

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace fkie_potree_rviz_plugin
{
class PotreeNode;
class CloudLoader;

class LoadingThread
{
public:
    explicit LoadingThread (const std::shared_ptr<CloudLoader>& loader);
    ~LoadingThread();
    void unscheduleAll();
    void scheduleForLoading (const std::shared_ptr<PotreeNode>& node);
    void setNodeLoadedCallback (const std::function<void()>& func);
private:
    void run();
    std::function<void()> func_;
    bool running_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::shared_ptr<CloudLoader> loader_;
    std::queue<std::shared_ptr<PotreeNode>> need_to_load_;
    std::thread thread_;
};

} // namespace fkie_rviz_plugin_potree

#endif /* SRC_LOADING_THREAD_H_ */
