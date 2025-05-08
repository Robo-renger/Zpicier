// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice
#include "msgs/msg/detail/joystick__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "msgs/msg/detail/joystick__struct.h"
#include "msgs/msg/detail/joystick__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // button_states
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // button_states
#include "rosidl_runtime_c/string.h"  // button_names
#include "rosidl_runtime_c/string_functions.h"  // button_names

// forward declare type support functions


using _Joystick__ros_msg_type = msgs__msg__Joystick;

static bool _Joystick__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Joystick__ros_msg_type * ros_message = static_cast<const _Joystick__ros_msg_type *>(untyped_ros_message);
  // Field name: left_x_axis
  {
    cdr << ros_message->left_x_axis;
  }

  // Field name: left_y_axis
  {
    cdr << ros_message->left_y_axis;
  }

  // Field name: right_x_axis
  {
    cdr << ros_message->right_x_axis;
  }

  // Field name: right_y_axis
  {
    cdr << ros_message->right_y_axis;
  }

  // Field name: button_names
  {
    size_t size = ros_message->button_names.size;
    auto array_ptr = ros_message->button_names.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: button_states
  {
    size_t size = ros_message->button_states.size;
    auto array_ptr = ros_message->button_states.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Joystick__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Joystick__ros_msg_type * ros_message = static_cast<_Joystick__ros_msg_type *>(untyped_ros_message);
  // Field name: left_x_axis
  {
    cdr >> ros_message->left_x_axis;
  }

  // Field name: left_y_axis
  {
    cdr >> ros_message->left_y_axis;
  }

  // Field name: right_x_axis
  {
    cdr >> ros_message->right_x_axis;
  }

  // Field name: right_y_axis
  {
    cdr >> ros_message->right_y_axis;
  }

  // Field name: button_names
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->button_names.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->button_names);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->button_names, size)) {
      fprintf(stderr, "failed to create array for field 'button_names'");
      return false;
    }
    auto array_ptr = ros_message->button_names.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'button_names'\n");
        return false;
      }
    }
  }

  // Field name: button_states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->button_states.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->button_states);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->button_states, size)) {
      fprintf(stderr, "failed to create array for field 'button_states'");
      return false;
    }
    auto array_ptr = ros_message->button_states.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msgs
size_t get_serialized_size_msgs__msg__Joystick(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Joystick__ros_msg_type * ros_message = static_cast<const _Joystick__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name left_x_axis
  {
    size_t item_size = sizeof(ros_message->left_x_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_y_axis
  {
    size_t item_size = sizeof(ros_message->left_y_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_x_axis
  {
    size_t item_size = sizeof(ros_message->right_x_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_y_axis
  {
    size_t item_size = sizeof(ros_message->right_y_axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name button_names
  {
    size_t array_size = ros_message->button_names.size;
    auto array_ptr = ros_message->button_names.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name button_states
  {
    size_t array_size = ros_message->button_states.size;
    auto array_ptr = ros_message->button_states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Joystick__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_msgs__msg__Joystick(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msgs
size_t max_serialized_size_msgs__msg__Joystick(
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

  // member: left_x_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_y_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_x_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_y_axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: button_names
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
  // member: button_states
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
    using DataType = msgs__msg__Joystick;
    is_plain =
      (
      offsetof(DataType, button_states) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Joystick__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_msgs__msg__Joystick(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Joystick = {
  "msgs::msg",
  "Joystick",
  _Joystick__cdr_serialize,
  _Joystick__cdr_deserialize,
  _Joystick__get_serialized_size,
  _Joystick__max_serialized_size
};

static rosidl_message_type_support_t _Joystick__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Joystick,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, msgs, msg, Joystick)() {
  return &_Joystick__type_support;
}

#if defined(__cplusplus)
}
#endif
