// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msgs/msg/detail/joystick__rosidl_typesupport_introspection_c.h"
#include "msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msgs/msg/detail/joystick__functions.h"
#include "msgs/msg/detail/joystick__struct.h"


// Include directives for member types
// Member `button_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `button_states`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msgs__msg__Joystick__init(message_memory);
}

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_fini_function(void * message_memory)
{
  msgs__msg__Joystick__fini(message_memory);
}

size_t msgs__msg__Joystick__rosidl_typesupport_introspection_c__size_function__Joystick__button_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__fetch_function__Joystick__button_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__assign_function__Joystick__button_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool msgs__msg__Joystick__rosidl_typesupport_introspection_c__resize_function__Joystick__button_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t msgs__msg__Joystick__rosidl_typesupport_introspection_c__size_function__Joystick__button_states(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_states(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_states(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__fetch_function__Joystick__button_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_states(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void msgs__msg__Joystick__rosidl_typesupport_introspection_c__assign_function__Joystick__button_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_states(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool msgs__msg__Joystick__rosidl_typesupport_introspection_c__resize_function__Joystick__button_states(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_member_array[6] = {
  {
    "left_x_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, left_x_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_y_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, left_y_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_x_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, right_x_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_y_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, right_y_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "button_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, button_names),  // bytes offset in struct
    NULL,  // default value
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__size_function__Joystick__button_names,  // size() function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_names,  // get_const(index) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_names,  // get(index) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__fetch_function__Joystick__button_names,  // fetch(index, &value) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__assign_function__Joystick__button_names,  // assign(index, value) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__resize_function__Joystick__button_names  // resize(index) function pointer
  },
  {
    "button_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs__msg__Joystick, button_states),  // bytes offset in struct
    NULL,  // default value
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__size_function__Joystick__button_states,  // size() function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_const_function__Joystick__button_states,  // get_const(index) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__get_function__Joystick__button_states,  // get(index) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__fetch_function__Joystick__button_states,  // fetch(index, &value) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__assign_function__Joystick__button_states,  // assign(index, value) function pointer
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__resize_function__Joystick__button_states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_members = {
  "msgs__msg",  // message namespace
  "Joystick",  // message name
  6,  // number of fields
  sizeof(msgs__msg__Joystick),
  msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_member_array,  // message members
  msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_init_function,  // function to initialize message memory (memory has to be allocated)
  msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_type_support_handle = {
  0,
  &msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msgs, msg, Joystick)() {
  if (!msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_type_support_handle.typesupport_identifier) {
    msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msgs__msg__Joystick__rosidl_typesupport_introspection_c__Joystick_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
