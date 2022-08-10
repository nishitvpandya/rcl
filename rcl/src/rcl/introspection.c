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

#ifdef __cplusplus
extern "C" {
#endif

#include "rcl/introspection.h"

#include <string.h>

#include "./client_impl.h"
#include "./service_impl.h"
#include "builtin_interfaces/msg/time.h"
#include "dlfcn.h"
#include "rcl/error_handling.h"
#include "rcutils/logging_macros.h"
#include "rcutils/macros.h"
#include "rcutils/shared_library.h"
#include "rmw/error_handling.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "unique_identifier_msgs/msg/uuid.h"
#include "service_msgs/msg/service_event_info.h"

rcl_service_introspection_utils_t rcl_get_zero_initialized_introspection_utils()
{
  static rcl_service_introspection_utils_t null_introspection_utils = {
    .service_type_support = NULL,
    .publisher = NULL,
    .clock = NULL,
    .service_event_topic_name = {0},
    ._enabled = true,
    ._content_enabled = true,
  };
  return null_introspection_utils;
}

rcl_ret_t rcl_service_typesupport_to_message_typesupport(
  const rosidl_service_type_support_t * service_typesupport,
  rosidl_message_type_support_t ** request_typesupport,
  rosidl_message_type_support_t ** response_typesupport,
  const rcl_allocator_t * allocator)
{
  rcutils_ret_t ret;
  type_support_map_t * map = (type_support_map_t *)service_typesupport->data;
  // TODO(ihasdapie): #define this
  const char * typesupport_library_fmt = "lib%s__rosidl_typesupport_c.so";
  const char * service_message_fmt =  // package_name, type name, Request/Response
    "rosidl_typesupport_c__get_message_type_support_handle__%s__srv__%s_%s";

  const char * service_type_name = rcl_service_get_service_type_name(service_typesupport);

  // build out typesupport library and symbol names
  char * typesupport_library_name = allocator->allocate(
    sizeof(char) * ((strlen(typesupport_library_fmt) - 2) + strlen(map->package_name) + 1),
    allocator->state);
  char * request_message_symbol = allocator->allocate(
    sizeof(char) * ((strlen(service_message_fmt) - 6) + strlen(map->package_name) +
                    strlen(service_type_name) + strlen("Request") + 1),
    allocator->state);
  char * response_message_symbol = allocator->allocate(
    sizeof(char) * ((strlen(service_message_fmt) - 6) + strlen(map->package_name) +
                    strlen(service_type_name) + strlen("Request") + 1),
    allocator->state);

  sprintf(typesupport_library_name, typesupport_library_fmt, map->package_name);
  sprintf(
    request_message_symbol, service_message_fmt, map->package_name, service_type_name, "Request");
  sprintf(
    response_message_symbol, service_message_fmt, map->package_name, service_type_name, "Response");

  rcutils_shared_library_t typesupport_library = rcutils_get_zero_initialized_shared_library();
  ret = rcutils_load_shared_library(&typesupport_library, typesupport_library_name, *allocator);
  if (RCUTILS_RET_OK != ret) {
    return RCL_RET_ERROR;
  }

  rosidl_message_type_support_t * (*req_typesupport_func_handle)() =
    (rosidl_message_type_support_t * (*)())
      rcutils_get_symbol(&typesupport_library, request_message_symbol);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    req_typesupport_func_handle, "Looking up request type support failed", return RCL_RET_ERROR);

  rosidl_message_type_support_t * (*resp_typesupport_func_handle)() =
    (rosidl_message_type_support_t * (*)())
      rcutils_get_symbol(&typesupport_library, response_message_symbol);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    resp_typesupport_func_handle, "Looking up response type support failed", return RCL_RET_ERROR);

  *request_typesupport = req_typesupport_func_handle();
  *response_typesupport = resp_typesupport_func_handle();
  allocator->deallocate(typesupport_library_name, allocator->state);
  allocator->deallocate(request_message_symbol, allocator->state);
  allocator->deallocate(response_message_symbol, allocator->state);

  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_init(
  rcl_service_introspection_utils_t * introspection_utils,
  const rosidl_service_type_support_t * service_type_support,
  const char * service_name,
  const rcl_node_t * node,
  rcl_clock_t * clock,
  rcl_allocator_t * allocator)
{
  rcl_ret_t ret;

  // TODO(ihasdapie): Use maximum length of service name and type name?

  // Can do this because typesupports have static lifetimes
  introspection_utils->service_type_support = service_type_support;

  // Make a publisher
  if (strlen(service_name) > 255 - strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) {
    // topics have a maximum length of 255 characters
    RCL_SET_ERROR_MSG("Service name is too long");
    return RCL_RET_ERROR;
  }
  strcpy(introspection_utils->service_event_topic_name, service_name);
  strcat(introspection_utils->service_event_topic_name, RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX);

  introspection_utils->publisher = allocator->allocate(sizeof(rcl_publisher_t), allocator->state);
  *introspection_utils->publisher = rcl_get_zero_initialized_publisher();

  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(introspection_utils->publisher, node,
      service_type_support->event_typesupport, introspection_utils->service_event_topic_name,
      &publisher_options);

  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rcl_get_error_string().str);
    return RCL_RET_ERROR;
  }

  introspection_utils->clock = clock;
  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_fini(
  rcl_service_introspection_utils_t * introspection_utils, rcl_allocator_t * allocator,
  rcl_node_t * node)
{
  rcl_ret_t ret;
  if (NULL != introspection_utils->publisher) {
    ret = rcl_publisher_fini(introspection_utils->publisher, node);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG(rcl_get_error_string().str);
      return RCL_RET_ERROR;
    }
  }
  
  if (NULL != introspection_utils->publisher) {
    allocator->deallocate(introspection_utils->publisher, allocator->state);
  }
  allocator->deallocate(introspection_utils->service_event_topic_name, allocator->state);

  introspection_utils->publisher = NULL;
  introspection_utils->service_event_topic_name = NULL;

  return RCL_RET_OK;
}

rcl_ret_t rcl_introspection_send_message(
  const rcl_service_introspection_utils_t * introspection_utils, const uint8_t event_type,
  const void * ros_response_request, const int64_t sequence_number, const uint8_t uuid[16],
  const rcl_allocator_t * allocator)
{
  // Early exit of service introspection if it isn't enabled
  // TODO(ihasdapie): Different return code?
  if (!introspection_utils->_enabled) return RCL_RET_OK;

  rcl_ret_t ret;

  rcl_time_point_value_t now;
  ret = rcl_clock_get_now(introspection_utils->clock, &now); // ERROR: Clock null?
  if (RMW_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }

  rosidl_service_introspection_info_t info = {
    .event_type = event_type,
    .stamp_sec = RCL_NS_TO_S(now),
    .stamp_nanosec = now % (1000LL * 1000LL * 1000LL),
    .client_id = {uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
      uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]}, // or just memcpy it in? is there a shorthand?
    .sequence_number = sequence_number,
  };


  
  void * service_introspection_message;
  switch (event_type) {
    case service_msgs__msg__ServiceEventInfo__REQUEST_RECEIVED:
    case service_msgs__msg__ServiceEventInfo__REQUEST_SENT:
      service_introspection_message = introspection_utils->service_type_support->introspection_message_create_handle(
          &info, allocator, ros_response_request, NULL, introspection_utils->_content_enabled); // Add switch depending on req/resp
          break;
    case service_msgs__msg__ServiceEventInfo__RESPONSE_RECEIVED:
    case service_msgs__msg__ServiceEventInfo__RESPONSE_SENT: // error on send_response
      service_introspection_message = introspection_utils->service_type_support->introspection_message_create_handle(
          &info, allocator, NULL, ros_response_request, introspection_utils->_content_enabled); // Add switch depending on req/resp
          break;
  }

  if (NULL == service_introspection_message) {
    RCL_SET_ERROR_MSG("Failed to create service introspection message");
    return RCL_RET_ERROR;
  }



  // and publish it out!
  // TODO(ihasdapie): Publisher context can become invalidated on shutdown
  ret = rcl_publish(introspection_utils->publisher, service_introspection_message, NULL);
  if (RMW_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }

  // clean up
  introspection_utils->service_type_support->introspection_message_destroy_handle(service_introspection_message, allocator);

  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_enable(
  rcl_service_introspection_utils_t * introspection_utils, const rcl_node_t * node,
  rcl_allocator_t * allocator)
{
  rcl_ret_t ret;

  introspection_utils->publisher = allocator->allocate(sizeof(rcl_publisher_t), allocator->state);
  *introspection_utils->publisher = rcl_get_zero_initialized_publisher();

  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(
    introspection_utils->publisher, node, 
    introspection_utils->service_type_support->event_typesupport,
    introspection_utils->service_event_topic_name, &publisher_options);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rcl_get_error_string().str);
    return RCL_RET_ERROR;
  }

  introspection_utils->_enabled = true;
  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_disable(
  rcl_service_introspection_utils_t * introspection_utils, rcl_node_t * node,
  const rcl_allocator_t * allocator)
{
  rcl_ret_t ret;
  ret = rcl_publisher_fini(introspection_utils->publisher, node);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }

  allocator->deallocate(introspection_utils->publisher, allocator->state);
  introspection_utils->publisher = NULL;

  introspection_utils->_enabled = false;
  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_configure_service_events(
  rcl_service_t * service, rcl_node_t * node, bool enable)
{
  rcl_service_introspection_utils_t * introspection_utils = service->impl->introspection_utils;

  if (enable == introspection_utils->_enabled) {
    return RCL_RET_OK;
  }

  rcl_ret_t ret;

  if (enable) {
    ret = rcl_service_introspection_enable(
      introspection_utils, node, &service->impl->options.allocator);
  } else {
    ret = rcl_service_introspection_disable(
      introspection_utils, node, &service->impl->options.allocator);
  }
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }
  return RCL_RET_OK;
}

rcl_ret_t rcl_service_introspection_configure_client_events(
  rcl_client_t * client, rcl_node_t * node, bool enable)
{
  rcl_service_introspection_utils_t * introspection_utils = client->impl->introspection_utils;

  if (enable == introspection_utils->_enabled) {
    return RCL_RET_OK;
  }

  rcl_ret_t ret;
  if (enable) {
    ret = rcl_service_introspection_enable(
        introspection_utils, node, &client->impl->options.allocator);
  } else {
    ret = rcl_service_introspection_disable(
      introspection_utils, node, &client->impl->options.allocator);
  }
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }
  return RCL_RET_OK;
}

void rcl_service_introspection_configure_client_content(rcl_client_t * client, bool enable)
{
  client->impl->introspection_utils->_content_enabled = enable;
}

void rcl_service_introspection_configure_service_content(rcl_service_t * service, bool enable)
{
  service->impl->introspection_utils->_content_enabled = enable;
}

#ifdef __cplusplus
}
#endif
