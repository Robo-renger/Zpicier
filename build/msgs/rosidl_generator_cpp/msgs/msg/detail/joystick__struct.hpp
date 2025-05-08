// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef MSGS__MSG__DETAIL__JOYSTICK__STRUCT_HPP_
#define MSGS__MSG__DETAIL__JOYSTICK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msgs__msg__Joystick __attribute__((deprecated))
#else
# define DEPRECATED__msgs__msg__Joystick __declspec(deprecated)
#endif

namespace msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joystick_
{
  using Type = Joystick_<ContainerAllocator>;

  explicit Joystick_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_x_axis = 0.0;
      this->left_y_axis = 0.0;
      this->right_x_axis = 0.0;
      this->right_y_axis = 0.0;
    }
  }

  explicit Joystick_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_x_axis = 0.0;
      this->left_y_axis = 0.0;
      this->right_x_axis = 0.0;
      this->right_y_axis = 0.0;
    }
  }

  // field types and members
  using _left_x_axis_type =
    double;
  _left_x_axis_type left_x_axis;
  using _left_y_axis_type =
    double;
  _left_y_axis_type left_y_axis;
  using _right_x_axis_type =
    double;
  _right_x_axis_type right_x_axis;
  using _right_y_axis_type =
    double;
  _right_y_axis_type right_y_axis;
  using _button_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _button_names_type button_names;
  using _button_states_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _button_states_type button_states;

  // setters for named parameter idiom
  Type & set__left_x_axis(
    const double & _arg)
  {
    this->left_x_axis = _arg;
    return *this;
  }
  Type & set__left_y_axis(
    const double & _arg)
  {
    this->left_y_axis = _arg;
    return *this;
  }
  Type & set__right_x_axis(
    const double & _arg)
  {
    this->right_x_axis = _arg;
    return *this;
  }
  Type & set__right_y_axis(
    const double & _arg)
  {
    this->right_y_axis = _arg;
    return *this;
  }
  Type & set__button_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->button_names = _arg;
    return *this;
  }
  Type & set__button_states(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->button_states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::msg::Joystick_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::msg::Joystick_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::msg::Joystick_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::msg::Joystick_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::msg::Joystick_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::msg::Joystick_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::msg::Joystick_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::msg::Joystick_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::msg::Joystick_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::msg::Joystick_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__msg__Joystick
    std::shared_ptr<msgs::msg::Joystick_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__msg__Joystick
    std::shared_ptr<msgs::msg::Joystick_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joystick_ & other) const
  {
    if (this->left_x_axis != other.left_x_axis) {
      return false;
    }
    if (this->left_y_axis != other.left_y_axis) {
      return false;
    }
    if (this->right_x_axis != other.right_x_axis) {
      return false;
    }
    if (this->right_y_axis != other.right_y_axis) {
      return false;
    }
    if (this->button_names != other.button_names) {
      return false;
    }
    if (this->button_states != other.button_states) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joystick_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joystick_

// alias to use template instance with default allocator
using Joystick =
  msgs::msg::Joystick_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs

#endif  // MSGS__MSG__DETAIL__JOYSTICK__STRUCT_HPP_
