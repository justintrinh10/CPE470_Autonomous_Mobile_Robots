// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#ifndef CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "checkpoint_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "checkpoint_interfaces/msg/detail/parameters_to_target__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace checkpoint_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_checkpoint_interfaces
cdr_serialize(
  const checkpoint_interfaces::msg::ParametersToTarget & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_checkpoint_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  checkpoint_interfaces::msg::ParametersToTarget & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_checkpoint_interfaces
get_serialized_size(
  const checkpoint_interfaces::msg::ParametersToTarget & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_checkpoint_interfaces
max_serialized_size_ParametersToTarget(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace checkpoint_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_checkpoint_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, checkpoint_interfaces, msg, ParametersToTarget)();

#ifdef __cplusplus
}
#endif

#endif  // CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
