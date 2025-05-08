// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msgs:msg/Joystick.idl
// generated code does not contain a copyright notice
#include "msgs/msg/detail/joystick__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `button_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `button_states`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
msgs__msg__Joystick__init(msgs__msg__Joystick * msg)
{
  if (!msg) {
    return false;
  }
  // left_x_axis
  // left_y_axis
  // right_x_axis
  // right_y_axis
  // button_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->button_names, 0)) {
    msgs__msg__Joystick__fini(msg);
    return false;
  }
  // button_states
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->button_states, 0)) {
    msgs__msg__Joystick__fini(msg);
    return false;
  }
  return true;
}

void
msgs__msg__Joystick__fini(msgs__msg__Joystick * msg)
{
  if (!msg) {
    return;
  }
  // left_x_axis
  // left_y_axis
  // right_x_axis
  // right_y_axis
  // button_names
  rosidl_runtime_c__String__Sequence__fini(&msg->button_names);
  // button_states
  rosidl_runtime_c__boolean__Sequence__fini(&msg->button_states);
}

bool
msgs__msg__Joystick__are_equal(const msgs__msg__Joystick * lhs, const msgs__msg__Joystick * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_x_axis
  if (lhs->left_x_axis != rhs->left_x_axis) {
    return false;
  }
  // left_y_axis
  if (lhs->left_y_axis != rhs->left_y_axis) {
    return false;
  }
  // right_x_axis
  if (lhs->right_x_axis != rhs->right_x_axis) {
    return false;
  }
  // right_y_axis
  if (lhs->right_y_axis != rhs->right_y_axis) {
    return false;
  }
  // button_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->button_names), &(rhs->button_names)))
  {
    return false;
  }
  // button_states
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->button_states), &(rhs->button_states)))
  {
    return false;
  }
  return true;
}

bool
msgs__msg__Joystick__copy(
  const msgs__msg__Joystick * input,
  msgs__msg__Joystick * output)
{
  if (!input || !output) {
    return false;
  }
  // left_x_axis
  output->left_x_axis = input->left_x_axis;
  // left_y_axis
  output->left_y_axis = input->left_y_axis;
  // right_x_axis
  output->right_x_axis = input->right_x_axis;
  // right_y_axis
  output->right_y_axis = input->right_y_axis;
  // button_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->button_names), &(output->button_names)))
  {
    return false;
  }
  // button_states
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->button_states), &(output->button_states)))
  {
    return false;
  }
  return true;
}

msgs__msg__Joystick *
msgs__msg__Joystick__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs__msg__Joystick * msg = (msgs__msg__Joystick *)allocator.allocate(sizeof(msgs__msg__Joystick), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msgs__msg__Joystick));
  bool success = msgs__msg__Joystick__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msgs__msg__Joystick__destroy(msgs__msg__Joystick * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msgs__msg__Joystick__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msgs__msg__Joystick__Sequence__init(msgs__msg__Joystick__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs__msg__Joystick * data = NULL;

  if (size) {
    data = (msgs__msg__Joystick *)allocator.zero_allocate(size, sizeof(msgs__msg__Joystick), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msgs__msg__Joystick__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msgs__msg__Joystick__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msgs__msg__Joystick__Sequence__fini(msgs__msg__Joystick__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msgs__msg__Joystick__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msgs__msg__Joystick__Sequence *
msgs__msg__Joystick__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs__msg__Joystick__Sequence * array = (msgs__msg__Joystick__Sequence *)allocator.allocate(sizeof(msgs__msg__Joystick__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msgs__msg__Joystick__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msgs__msg__Joystick__Sequence__destroy(msgs__msg__Joystick__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msgs__msg__Joystick__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msgs__msg__Joystick__Sequence__are_equal(const msgs__msg__Joystick__Sequence * lhs, const msgs__msg__Joystick__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msgs__msg__Joystick__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msgs__msg__Joystick__Sequence__copy(
  const msgs__msg__Joystick__Sequence * input,
  msgs__msg__Joystick__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msgs__msg__Joystick);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msgs__msg__Joystick * data =
      (msgs__msg__Joystick *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msgs__msg__Joystick__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msgs__msg__Joystick__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msgs__msg__Joystick__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
