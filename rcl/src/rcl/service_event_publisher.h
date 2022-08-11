// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCL__SERVICE_EVENT_PUBLISHER_H_
#define RCL__SERVICE_EVENT_PUBLISHER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/client.h"
#include "rcl/service.h"
#include "rcl/publisher.h"
#include "rcl/time.h"
#include "rmw/rmw.h"
#include "stdbool.h"

typedef struct rcl_service_event_impl_s {
  /// Handle to clock for timestamping service events
  rcl_clock_t * clock;
  /// Handle to publisher for publishing service events
  rcl_publisher_t * publisher;
  /// Handle to service typesupport
  const rosidl_service_type_support_t * service_type_support;
  /// Name of service introspection topic: <service_name>/RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX
  char service_event_topic_name[255];
  // Enable/disable service introspection during runtime
  bool _enabled;
  // Enable/disable including request/response payload in service event message during runtime
  bool _content_enabled;
} rcl_service_event_publisher_impl_t;

typedef struct rcl_service_event_publisher_s {
  rcl_service_event_publisher_impl_t * impl;
} rcl_service_event_publisher_t;

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_typesupport_to_message_typesupport(
  const rosidl_service_type_support_t * service_typesupport,
  rosidl_message_type_support_t ** request_typesupport,
  rosidl_message_type_support_t ** response_typesupport,
  const rcl_allocator_t * allocator);

RCL_LOCAL
RCL_WARN_UNUSED
rcl_service_event_publisher_t
rcl_get_zero_initialized_service_event_publisher();

RCL_LOCAL
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_event_publisher_init(
  rcl_service_event_publisher_t * service_event_publisher,
  const rosidl_service_type_support_t * service_type_support,
  const char * service_name,
  const rcl_node_t * node,
  rcl_clock_t * clock,
  rcl_publisher_options_t publisher_options,
  rcl_allocator_t * allocator);

RCL_LOCAL
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_event_publisher_fini(
  rcl_service_event_publisher_t * service_event_publisher,
  rcl_allocator_t * allocator,
  rcl_node_t * node);

RCL_LOCAL
RCL_WARN_UNUSED
rcl_ret_t
rcl_send_service_event_message(
  const rcl_service_event_publisher_t * service_event_publisher,
  uint8_t event_type,
  const void * ros_response_request,
  int64_t sequence_number,
  const uint8_t uuid[16], // uuid is uint8_t but the guid is int8_t
  const rcl_allocator_t * allocator);

/*  Enables service introspection by reconstructing the introspection clock and publisher
 *  
 *  Does nothing and returns RCL_RET_OK if already enabled
 */
RCL_LOCAL
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_introspection_enable(
  rcl_service_event_publisher_t * service_event_publisher,
  const rcl_node_t * node,
  rcl_publisher_options_t publisher_options,
  rcl_allocator_t * allocator);

/*  Disables service introspection by fini-ing and freeing the introspection clock and publisher
 *  
 *  Does nothing and returns RCL_RET_OK if already disabled
 */
RCL_LOCAL
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_introspection_disable(
  rcl_service_event_publisher_t * service_event_publisher,
  rcl_node_t * node,
  const rcl_allocator_t * allocator);

#ifdef __cplusplus
}
#endif
#endif // RCL__SERVICE_EVENT_PUBLISHER_H_
