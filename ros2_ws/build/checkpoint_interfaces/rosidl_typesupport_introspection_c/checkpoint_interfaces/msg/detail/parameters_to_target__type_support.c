// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "checkpoint_interfaces/msg/detail/parameters_to_target__rosidl_typesupport_introspection_c.h"
#include "checkpoint_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "checkpoint_interfaces/msg/detail/parameters_to_target__functions.h"
#include "checkpoint_interfaces/msg/detail/parameters_to_target__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  checkpoint_interfaces__msg__ParametersToTarget__init(message_memory);
}

void checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_fini_function(void * message_memory)
{
  checkpoint_interfaces__msg__ParametersToTarget__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_member_array[3] = {
  {
    "marker_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(checkpoint_interfaces__msg__ParametersToTarget, marker_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(checkpoint_interfaces__msg__ParametersToTarget, alignment_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_seperation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(checkpoint_interfaces__msg__ParametersToTarget, distance_seperation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_members = {
  "checkpoint_interfaces__msg",  // message namespace
  "ParametersToTarget",  // message name
  3,  // number of fields
  sizeof(checkpoint_interfaces__msg__ParametersToTarget),
  checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_member_array,  // message members
  checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_init_function,  // function to initialize message memory (memory has to be allocated)
  checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_type_support_handle = {
  0,
  &checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_checkpoint_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, checkpoint_interfaces, msg, ParametersToTarget)() {
  if (!checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_type_support_handle.typesupport_identifier) {
    checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &checkpoint_interfaces__msg__ParametersToTarget__rosidl_typesupport_introspection_c__ParametersToTarget_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
