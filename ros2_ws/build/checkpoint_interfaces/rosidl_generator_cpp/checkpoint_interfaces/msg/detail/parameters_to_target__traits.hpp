// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice

#ifndef CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__TRAITS_HPP_
#define CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "checkpoint_interfaces/msg/detail/parameters_to_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace checkpoint_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParametersToTarget & msg,
  std::ostream & out)
{
  out << "{";
  // member: marker_id
  {
    out << "marker_id: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_id, out);
    out << ", ";
  }

  // member: alignment_error
  {
    out << "alignment_error: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_error, out);
    out << ", ";
  }

  // member: distance_seperation
  {
    out << "distance_seperation: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_seperation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParametersToTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: marker_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "marker_id: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_id, out);
    out << "\n";
  }

  // member: alignment_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_error: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_error, out);
    out << "\n";
  }

  // member: distance_seperation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_seperation: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_seperation, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParametersToTarget & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace checkpoint_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use checkpoint_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const checkpoint_interfaces::msg::ParametersToTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  checkpoint_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use checkpoint_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const checkpoint_interfaces::msg::ParametersToTarget & msg)
{
  return checkpoint_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<checkpoint_interfaces::msg::ParametersToTarget>()
{
  return "checkpoint_interfaces::msg::ParametersToTarget";
}

template<>
inline const char * name<checkpoint_interfaces::msg::ParametersToTarget>()
{
  return "checkpoint_interfaces/msg/ParametersToTarget";
}

template<>
struct has_fixed_size<checkpoint_interfaces::msg::ParametersToTarget>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<checkpoint_interfaces::msg::ParametersToTarget>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<checkpoint_interfaces::msg::ParametersToTarget>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHECKPOINT_INTERFACES__MSG__DETAIL__PARAMETERS_TO_TARGET__TRAITS_HPP_
