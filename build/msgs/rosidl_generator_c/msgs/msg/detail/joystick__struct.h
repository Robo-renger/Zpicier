// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef MSGS__MSG__DETAIL__JOYSTICK__STRUCT_H_
#define MSGS__MSG__DETAIL__JOYSTICK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'button_names'
#include "rosidl_runtime_c/string.h"
// Member 'button_states'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Joystick in the package msgs.
typedef struct msgs__msg__Joystick
{
  double left_x_axis;
  double left_y_axis;
  double right_x_axis;
  double right_y_axis;
  rosidl_runtime_c__String__Sequence button_names;
  rosidl_runtime_c__boolean__Sequence button_states;
} msgs__msg__Joystick;

// Struct for a sequence of msgs__msg__Joystick.
typedef struct msgs__msg__Joystick__Sequence
{
  msgs__msg__Joystick * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs__msg__Joystick__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS__MSG__DETAIL__JOYSTICK__STRUCT_H_
