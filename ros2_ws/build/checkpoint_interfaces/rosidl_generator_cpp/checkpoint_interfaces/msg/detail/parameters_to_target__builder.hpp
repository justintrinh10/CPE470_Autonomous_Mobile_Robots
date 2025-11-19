// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#ifndef CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__BUILDER_HPP_
#define CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "checkpoint_interfaces/msg/detail/parameters_to_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace checkpoint_interfaces
{

namespace msg
{

namespace builder
{

class Init_ParametersToTarget_distance_seperation
{
public:
  explicit Init_ParametersToTarget_distance_seperation(::checkpoint_interfaces::msg::ParametersToTarget & msg)
  : msg_(msg)
  {}
  ::checkpoint_interfaces::msg::ParametersToTarget distance_seperation(::checkpoint_interfaces::msg::ParametersToTarget::_distance_seperation_type arg)
  {
    msg_.distance_seperation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::checkpoint_interfaces::msg::ParametersToTarget msg_;
};

class Init_ParametersToTarget_alignment_error
{
public:
  Init_ParametersToTarget_alignment_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParametersToTarget_distance_seperation alignment_error(::checkpoint_interfaces::msg::ParametersToTarget::_alignment_error_type arg)
  {
    msg_.alignment_error = std::move(arg);
    return Init_ParametersToTarget_distance_seperation(msg_);
  }

private:
  ::checkpoint_interfaces::msg::ParametersToTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::checkpoint_interfaces::msg::ParametersToTarget>()
{
  return checkpoint_interfaces::msg::builder::Init_ParametersToTarget_alignment_error();
}

}  // namespace checkpoint_interfaces

#endif  // CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__BUILDER_HPP_
