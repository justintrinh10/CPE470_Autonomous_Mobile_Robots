// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#ifndef CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_HPP_
#define CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__checkpoint_interfaces__msg__ParametersToTarget __attribute__((deprecated))
#else
# define DEPRECATED__checkpoint_interfaces__msg__ParametersToTarget __declspec(deprecated)
#endif

namespace checkpoint_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ParametersToTarget_
{
  using Type = ParametersToTarget_<ContainerAllocator>;

  explicit ParametersToTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_id = 0l;
      this->alignment_error = 0.0f;
      this->distance_seperation = 0.0f;
    }
  }

  explicit ParametersToTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_id = 0l;
      this->alignment_error = 0.0f;
      this->distance_seperation = 0.0f;
    }
  }

  // field types and members
  using _marker_id_type =
    int32_t;
  _marker_id_type marker_id;
  using _alignment_error_type =
    float;
  _alignment_error_type alignment_error;
  using _distance_seperation_type =
    float;
  _distance_seperation_type distance_seperation;

  // setters for named parameter idiom
  Type & set__marker_id(
    const int32_t & _arg)
  {
    this->marker_id = _arg;
    return *this;
  }
  Type & set__alignment_error(
    const float & _arg)
  {
    this->alignment_error = _arg;
    return *this;
  }
  Type & set__distance_seperation(
    const float & _arg)
  {
    this->distance_seperation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__checkpoint_interfaces__msg__ParametersToTarget
    std::shared_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__checkpoint_interfaces__msg__ParametersToTarget
    std::shared_ptr<checkpoint_interfaces::msg::ParametersToTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParametersToTarget_ & other) const
  {
    if (this->marker_id != other.marker_id) {
      return false;
    }
    if (this->alignment_error != other.alignment_error) {
      return false;
    }
    if (this->distance_seperation != other.distance_seperation) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParametersToTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParametersToTarget_

// alias to use template instance with default allocator
using ParametersToTarget =
  checkpoint_interfaces::msg::ParametersToTarget_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace checkpoint_interfaces

#endif  // CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__STRUCT_HPP_
