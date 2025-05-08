// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef MSGS__MSG__DETAIL__JOYSTICK__BUILDER_HPP_
#define MSGS__MSG__DETAIL__JOYSTICK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs/msg/detail/joystick__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs
{

namespace msg
{

namespace builder
{

class Init_Joystick_button_states
{
public:
  explicit Init_Joystick_button_states(::msgs::msg::Joystick & msg)
  : msg_(msg)
  {}
  ::msgs::msg::Joystick button_states(::msgs::msg::Joystick::_button_states_type arg)
  {
    msg_.button_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

class Init_Joystick_button_names
{
public:
  explicit Init_Joystick_button_names(::msgs::msg::Joystick & msg)
  : msg_(msg)
  {}
  Init_Joystick_button_states button_names(::msgs::msg::Joystick::_button_names_type arg)
  {
    msg_.button_names = std::move(arg);
    return Init_Joystick_button_states(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

class Init_Joystick_right_y_axis
{
public:
  explicit Init_Joystick_right_y_axis(::msgs::msg::Joystick & msg)
  : msg_(msg)
  {}
  Init_Joystick_button_names right_y_axis(::msgs::msg::Joystick::_right_y_axis_type arg)
  {
    msg_.right_y_axis = std::move(arg);
    return Init_Joystick_button_names(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

class Init_Joystick_right_x_axis
{
public:
  explicit Init_Joystick_right_x_axis(::msgs::msg::Joystick & msg)
  : msg_(msg)
  {}
  Init_Joystick_right_y_axis right_x_axis(::msgs::msg::Joystick::_right_x_axis_type arg)
  {
    msg_.right_x_axis = std::move(arg);
    return Init_Joystick_right_y_axis(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

class Init_Joystick_left_y_axis
{
public:
  explicit Init_Joystick_left_y_axis(::msgs::msg::Joystick & msg)
  : msg_(msg)
  {}
  Init_Joystick_right_x_axis left_y_axis(::msgs::msg::Joystick::_left_y_axis_type arg)
  {
    msg_.left_y_axis = std::move(arg);
    return Init_Joystick_right_x_axis(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

class Init_Joystick_left_x_axis
{
public:
  Init_Joystick_left_x_axis()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Joystick_left_y_axis left_x_axis(::msgs::msg::Joystick::_left_x_axis_type arg)
  {
    msg_.left_x_axis = std::move(arg);
    return Init_Joystick_left_y_axis(msg_);
  }

private:
  ::msgs::msg::Joystick msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::msg::Joystick>()
{
  return msgs::msg::builder::Init_Joystick_left_x_axis();
}

}  // namespace msgs

#endif  // MSGS__MSG__DETAIL__JOYSTICK__BUILDER_HPP_
