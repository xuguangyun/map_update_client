/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H
#define CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H

namespace cartographer {
namespace cloud {

template <class SubmapType>
MapBuilderContext<SubmapType>::MapBuilderContext(
    MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

template <class SubmapType>
common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
MapBuilderContext<SubmapType>::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::EnqueueSensorData(std::string& sensor_id, std::string& client_id,transform::Rigid3d& pose_data,
                                                      sensor::TimedPointCloudData& data) {
  map_builder_server_->incoming_data_queue_.Push(
      absl::make_unique<Data>(Data{sensor_id,client_id,pose_data,data}));
}


}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H
