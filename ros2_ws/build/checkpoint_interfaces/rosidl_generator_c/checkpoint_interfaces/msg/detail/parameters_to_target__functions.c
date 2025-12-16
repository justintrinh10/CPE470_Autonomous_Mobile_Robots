// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice
#include "checkpoint_interfaces/msg/detail/parameters_to_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
checkpoint_interfaces__msg__ParametersToTarget__init(checkpoint_interfaces__msg__ParametersToTarget * msg)
{
  if (!msg) {
    return false;
  }
  // marker_id
  // alignment_error
  // distance_seperation
  return true;
}

void
checkpoint_interfaces__msg__ParametersToTarget__fini(checkpoint_interfaces__msg__ParametersToTarget * msg)
{
  if (!msg) {
    return;
  }
  // marker_id
  // alignment_error
  // distance_seperation
}

bool
checkpoint_interfaces__msg__ParametersToTarget__are_equal(const checkpoint_interfaces__msg__ParametersToTarget * lhs, const checkpoint_interfaces__msg__ParametersToTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // marker_id
  if (lhs->marker_id != rhs->marker_id) {
    return false;
  }
  // alignment_error
  if (lhs->alignment_error != rhs->alignment_error) {
    return false;
  }
  // distance_seperation
  if (lhs->distance_seperation != rhs->distance_seperation) {
    return false;
  }
  return true;
}

bool
checkpoint_interfaces__msg__ParametersToTarget__copy(
  const checkpoint_interfaces__msg__ParametersToTarget * input,
  checkpoint_interfaces__msg__ParametersToTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // marker_id
  output->marker_id = input->marker_id;
  // alignment_error
  output->alignment_error = input->alignment_error;
  // distance_seperation
  output->distance_seperation = input->distance_seperation;
  return true;
}

checkpoint_interfaces__msg__ParametersToTarget *
checkpoint_interfaces__msg__ParametersToTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  checkpoint_interfaces__msg__ParametersToTarget * msg = (checkpoint_interfaces__msg__ParametersToTarget *)allocator.allocate(sizeof(checkpoint_interfaces__msg__ParametersToTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(checkpoint_interfaces__msg__ParametersToTarget));
  bool success = checkpoint_interfaces__msg__ParametersToTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
checkpoint_interfaces__msg__ParametersToTarget__destroy(checkpoint_interfaces__msg__ParametersToTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    checkpoint_interfaces__msg__ParametersToTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
checkpoint_interfaces__msg__ParametersToTarget__Sequence__init(checkpoint_interfaces__msg__ParametersToTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  checkpoint_interfaces__msg__ParametersToTarget * data = NULL;

  if (size) {
    data = (checkpoint_interfaces__msg__ParametersToTarget *)allocator.zero_allocate(size, sizeof(checkpoint_interfaces__msg__ParametersToTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = checkpoint_interfaces__msg__ParametersToTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        checkpoint_interfaces__msg__ParametersToTarget__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
checkpoint_interfaces__msg__ParametersToTarget__Sequence__fini(checkpoint_interfaces__msg__ParametersToTarget__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      checkpoint_interfaces__msg__ParametersToTarget__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

checkpoint_interfaces__msg__ParametersToTarget__Sequence *
checkpoint_interfaces__msg__ParametersToTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  checkpoint_interfaces__msg__ParametersToTarget__Sequence * array = (checkpoint_interfaces__msg__ParametersToTarget__Sequence *)allocator.allocate(sizeof(checkpoint_interfaces__msg__ParametersToTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = checkpoint_interfaces__msg__ParametersToTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
checkpoint_interfaces__msg__ParametersToTarget__Sequence__destroy(checkpoint_interfaces__msg__ParametersToTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    checkpoint_interfaces__msg__ParametersToTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
checkpoint_interfaces__msg__ParametersToTarget__Sequence__are_equal(const checkpoint_interfaces__msg__ParametersToTarget__Sequence * lhs, const checkpoint_interfaces__msg__ParametersToTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!checkpoint_interfaces__msg__ParametersToTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
checkpoint_interfaces__msg__ParametersToTarget__Sequence__copy(
  const checkpoint_interfaces__msg__ParametersToTarget__Sequence * input,
  checkpoint_interfaces__msg__ParametersToTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(checkpoint_interfaces__msg__ParametersToTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    checkpoint_interfaces__msg__ParametersToTarget * data =
      (checkpoint_interfaces__msg__ParametersToTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!checkpoint_interfaces__msg__ParametersToTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          checkpoint_interfaces__msg__ParametersToTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!checkpoint_interfaces__msg__ParametersToTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
