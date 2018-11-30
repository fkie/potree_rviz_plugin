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
#include <functional>
#include "cloud_loader.h"
#include "loading_thread.h"
#include "potree_node.h"

namespace fkie_potree_rviz_plugin
{

LoadingThread::LoadingThread (const std::shared_ptr<CloudLoader>& loader)
: running_(true), loader_(loader), thread_(std::bind(&LoadingThread::run, this))
{
}

LoadingThread::~LoadingThread()
{
    std::unique_lock<std::mutex> lock{mutex_};
    running_ = false;
    cond_.notify_all();
    lock.unlock();
    if (thread_.joinable()) thread_.join();
}

void LoadingThread::unscheduleAll()
{
    std::lock_guard<std::mutex> lock{mutex_};
    while (!need_to_load_.empty()) need_to_load_.pop();
}

void LoadingThread::setNodeLoadedCallback(const std::function<void()>& func)
{
    // This function gets called whenever a new node has been loaded
    func_ = func;
}

void LoadingThread::scheduleForLoading(const std::shared_ptr<PotreeNode>& node)
{
    std::lock_guard<std::mutex> lock{mutex_};
    need_to_load_.push(node);
    cond_.notify_one();
}

void LoadingThread::run()
{
    std::unique_lock<std::mutex> lock{mutex_};
    while (running_)
    {
        while (need_to_load_.empty())
        {
            cond_.wait(lock);
            if (!running_) return;
        }
        std::shared_ptr<PotreeNode> node = need_to_load_.front();
        need_to_load_.pop();
        if (node->isLoaded()) continue;
        lock.unlock();
        loader_->loadPoints(node);
        if (func_) func_();
        lock.lock();
    }
}

}
