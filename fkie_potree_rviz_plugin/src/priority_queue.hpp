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
#ifndef SRC_PRIORITY_QUEUE_H_
#define SRC_PRIORITY_QUEUE_H_

#include <queue>
#include <tuple>

namespace fkie_potree_rviz_plugin
{

template<class T, class P>
class PriorityQueue
{
public:
    void push(const T& item, const P& priority)
    {
        queue_.emplace(item, priority);
    }
    const T& top() const
    {
        return std::get<0>(queue_.top());
    }
    bool empty() const
    {
        return queue_.empty();
    }
    std::size_t size() const
    {
        return queue_.size();
    }
    void pop()
    {
        queue_.pop();
    }
    void clear()
    {
        queue_ = std::move(Queue());
    }

private:
    using Element = std::tuple<T, P>;
    struct Compare
    {
        bool operator()(const Element& e1, const Element& e2)
        {
            return std::get<1>(e1) < std::get<1>(e2);
        }
    };
    using Queue = std::priority_queue<Element, std::vector<Element>, Compare>;
    Queue queue_;
};

}  // namespace fkie_potree_rviz_plugin

#endif /* SRC_PRIORITY_QUEUE_H_ */
