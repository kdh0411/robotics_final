// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from duckie_interfaces:srv/LedControl.idl
// generated code does not contain a copyright notice

#ifndef DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__BUILDER_HPP_
#define DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "duckie_interfaces/srv/detail/led_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace duckie_interfaces
{

namespace srv
{

namespace builder
{

class Init_LedControl_Request_color
{
public:
  Init_LedControl_Request_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::duckie_interfaces::srv::LedControl_Request color(::duckie_interfaces::srv::LedControl_Request::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::duckie_interfaces::srv::LedControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::duckie_interfaces::srv::LedControl_Request>()
{
  return duckie_interfaces::srv::builder::Init_LedControl_Request_color();
}

}  // namespace duckie_interfaces


namespace duckie_interfaces
{

namespace srv
{

namespace builder
{

class Init_LedControl_Response_message
{
public:
  explicit Init_LedControl_Response_message(::duckie_interfaces::srv::LedControl_Response & msg)
  : msg_(msg)
  {}
  ::duckie_interfaces::srv::LedControl_Response message(::duckie_interfaces::srv::LedControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::duckie_interfaces::srv::LedControl_Response msg_;
};

class Init_LedControl_Response_success
{
public:
  Init_LedControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LedControl_Response_message success(::duckie_interfaces::srv::LedControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LedControl_Response_message(msg_);
  }

private:
  ::duckie_interfaces::srv::LedControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::duckie_interfaces::srv::LedControl_Response>()
{
  return duckie_interfaces::srv::builder::Init_LedControl_Response_success();
}

}  // namespace duckie_interfaces

#endif  // DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__BUILDER_HPP_
