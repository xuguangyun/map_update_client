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

#include "cartographer/cloud/handlers/add_rangefinder_data_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void AddRangefinderDataHandler::OnRequest(const proto::AddRangefinderDataRequest& request) {
  // The 'BlockingQueue' returned by 'sensor_data_queue()' is already
  // thread-safe. Therefore it suffices to get an unsynchronized reference to
  // the 'MapBuilderContext'.
    std::string sensor_id=request.sensor_metadata().sensor_id();
    std::string client_id=request.sensor_metadata().client_id();
    transform::Rigid3d pose=transform::ToRigid3(request.local_pose());
    sensor::TimedPointCloudData point_data=sensor::FromProto(request.timed_point_cloud_data());
  GetUnsynchronizedContext<MapBuilderContextInterface>()->EnqueueSensorData(sensor_id,
       client_id,
       pose,
       point_data);
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
