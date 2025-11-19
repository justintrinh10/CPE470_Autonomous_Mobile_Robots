// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#ifndef CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_H_
#define CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ParametersToTarget in the package checkpoint_interfaces.
typedef struct checkpoint_interfaces__msg__ParametersToTarget
{
  float alignment_error;
  float distance_seperation;
} checkpoint_interfaces__msg__ParametersToTarget;

// Struct for a sequence of checkpoint_interfaces__msg__ParametersToTarget.
typedef struct checkpoint_interfaces__msg__ParametersToTarget__Sequence
{
  checkpoint_interfaces__msg__ParametersToTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} checkpoint_interfaces__msg__ParametersToTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_H_
