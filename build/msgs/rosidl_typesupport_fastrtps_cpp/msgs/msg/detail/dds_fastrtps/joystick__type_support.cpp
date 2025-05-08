// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice
#include "msgs/msg/detail/joystick__rosidl_typesupport_fastrtps_cpp.hpp"
#include "msgs/msg/detail/joystick__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs
cdr_serialize(
  const msgs::msg::Joystick & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: left_x_axis
  cdr << ros_message.left_x_axis;
  // Member: left_y_axis
  cdr << ros_message.left_y_axis;
  // Member: right_x_axis
  cdr << ros_message.right_x_axis;
  // Member: right_y_axis
  cdr << ros_message.right_y_axis;
  // Member: button_names
  {
    cdr << ros_message.button_names;
  }
  // Member: button_states
  {
    cdr << ros_message.button_states;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msgs::msg::Joystick & ros_message)
{
  // Member: left_x_axis
  cdr >> ros_message.left_x_axis;

  // Member: left_y_axis
  cdr >> ros_message.left_y_axis;

  // Member: right_x_axis
  cdr >> ros_message.right_x_axis;

  // Member: right_y_axis
  cdr >> ros_message.right_y_axis;

  // Member: button_names
  {
    cdr >> ros_message.button_names;
  }

  // Member: button_states
  {
    cdr >> ros_message.button_states;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs
get_serialized_size(
  const msgs::msg::Joystick & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: left_x_axis
  {
    size_t item_size = sizeof(ros_message.left_x_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_y_axis
  {
    size_t item_size = sizeof(ros_message.left_y_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_x_axis
  {
    size_t item_size = sizeof(ros_message.right_x_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_y_axis
  {
    size_t item_size = sizeof(ros_message.right_y_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: button_names
  {
    size_t array_size = ros_message.button_names.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.button_names[index].size() + 1);
    }
  }
  // Member: button_states
  {
    size_t array_size = ros_message.button_states.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.button_states[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs
max_serialized_size_Joystick(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: left_x_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_y_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_x_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_y_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: button_names
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: button_states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = msgs::msg::Joystick;
    is_plain =
      (
      offsetof(DataType, button_states) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Joystick__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const msgs::msg::Joystick *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Joystick__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<msgs::msg::Joystick *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Joystick__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const msgs::msg::Joystick *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Joystick__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Joystick(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Joystick__callbacks = {
  "msgs::msg",
  "Joystick",
  _Joystick__cdr_serialize,
  _Joystick__cdr_deserialize,
  _Joystick__get_serialized_size,
  _Joystick__max_serialized_size
};

static rosidl_message_type_support_t _Joystick__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Joystick__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<msgs::msg::Joystick>()
{
  return &msgs::msg::typesupport_fastrtps_cpp::_Joystick__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msgs, msg, Joystick)() {
  return &msgs::msg::typesupport_fastrtps_cpp::_Joystick__handle;
}

#ifdef __cplusplus
}
#endif
