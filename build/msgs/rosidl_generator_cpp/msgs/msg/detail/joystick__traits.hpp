// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef MSGS__MSG__DETAIL__JOYSTICK__TRAITS_HPP_
#define MSGS__MSG__DETAIL__JOYSTICK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs/msg/detail/joystick__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Joystick & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_x_axis
  {
    out << "left_x_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.left_x_axis, out);
    out << ", ";
  }

  // member: left_y_axis
  {
    out << "left_y_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.left_y_axis, out);
    out << ", ";
  }

  // member: right_x_axis
  {
    out << "right_x_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.right_x_axis, out);
    out << ", ";
  }

  // member: right_y_axis
  {
    out << "right_y_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.right_y_axis, out);
    out << ", ";
  }

  // member: button_names
  {
    if (msg.button_names.size() == 0) {
      out << "button_names: []";
    } else {
      out << "button_names: [";
      size_t pending_items = msg.button_names.size();
      for (auto item : msg.button_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: button_states
  {
    if (msg.button_states.size() == 0) {
      out << "button_states: []";
    } else {
      out << "button_states: [";
      size_t pending_items = msg.button_states.size();
      for (auto item : msg.button_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Joystick & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_x_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_x_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.left_x_axis, out);
    out << "\n";
  }

  // member: left_y_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_y_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.left_y_axis, out);
    out << "\n";
  }

  // member: right_x_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_x_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.right_x_axis, out);
    out << "\n";
  }

  // member: right_y_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_y_axis: ";
    rosidl_generator_traits::value_to_yaml(msg.right_y_axis, out);
    out << "\n";
  }

  // member: button_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.button_names.size() == 0) {
      out << "button_names: []\n";
    } else {
      out << "button_names:\n";
      for (auto item : msg.button_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: button_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.button_states.size() == 0) {
      out << "button_states: []\n";
    } else {
      out << "button_states:\n";
      for (auto item : msg.button_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Joystick & msg, bool use_flow_style = false)
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

}  // namespace msgs

namespace rosidl_generator_traits
{

[[deprecated("use msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msgs::msg::Joystick & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const msgs::msg::Joystick & msg)
{
  return msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::msg::Joystick>()
{
  return "msgs::msg::Joystick";
}

template<>
inline const char * name<msgs::msg::Joystick>()
{
  return "msgs/msg/Joystick";
}

template<>
struct has_fixed_size<msgs::msg::Joystick>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msgs::msg::Joystick>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msgs::msg::Joystick>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS__MSG__DETAIL__JOYSTICK__TRAITS_HPP_
