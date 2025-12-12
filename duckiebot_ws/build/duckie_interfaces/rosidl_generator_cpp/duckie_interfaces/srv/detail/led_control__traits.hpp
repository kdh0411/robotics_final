// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from duckie_interfaces:srv/LedControl.idl
// generated code does not contain a copyright notice

#ifndef DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__TRAITS_HPP_
#define DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "duckie_interfaces/srv/detail/led_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace duckie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LedControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LedControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LedControl_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace duckie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use duckie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const duckie_interfaces::srv::LedControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  duckie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use duckie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const duckie_interfaces::srv::LedControl_Request & msg)
{
  return duckie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<duckie_interfaces::srv::LedControl_Request>()
{
  return "duckie_interfaces::srv::LedControl_Request";
}

template<>
inline const char * name<duckie_interfaces::srv::LedControl_Request>()
{
  return "duckie_interfaces/srv/LedControl_Request";
}

template<>
struct has_fixed_size<duckie_interfaces::srv::LedControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<duckie_interfaces::srv::LedControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<duckie_interfaces::srv::LedControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace duckie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LedControl_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LedControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LedControl_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace duckie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use duckie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const duckie_interfaces::srv::LedControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  duckie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use duckie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const duckie_interfaces::srv::LedControl_Response & msg)
{
  return duckie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<duckie_interfaces::srv::LedControl_Response>()
{
  return "duckie_interfaces::srv::LedControl_Response";
}

template<>
inline const char * name<duckie_interfaces::srv::LedControl_Response>()
{
  return "duckie_interfaces/srv/LedControl_Response";
}

template<>
struct has_fixed_size<duckie_interfaces::srv::LedControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<duckie_interfaces::srv::LedControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<duckie_interfaces::srv::LedControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<duckie_interfaces::srv::LedControl>()
{
  return "duckie_interfaces::srv::LedControl";
}

template<>
inline const char * name<duckie_interfaces::srv::LedControl>()
{
  return "duckie_interfaces/srv/LedControl";
}

template<>
struct has_fixed_size<duckie_interfaces::srv::LedControl>
  : std::integral_constant<
    bool,
    has_fixed_size<duckie_interfaces::srv::LedControl_Request>::value &&
    has_fixed_size<duckie_interfaces::srv::LedControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<duckie_interfaces::srv::LedControl>
  : std::integral_constant<
    bool,
    has_bounded_size<duckie_interfaces::srv::LedControl_Request>::value &&
    has_bounded_size<duckie_interfaces::srv::LedControl_Response>::value
  >
{
};

template<>
struct is_service<duckie_interfaces::srv::LedControl>
  : std::true_type
{
};

template<>
struct is_service_request<duckie_interfaces::srv::LedControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<duckie_interfaces::srv::LedControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__TRAITS_HPP_
