// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from duckie_interfaces:srv/LedControl.idl
// generated code does not contain a copyright notice
#include "duckie_interfaces/srv/detail/led_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
duckie_interfaces__srv__LedControl_Request__init(duckie_interfaces__srv__LedControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    duckie_interfaces__srv__LedControl_Request__fini(msg);
    return false;
  }
  return true;
}

void
duckie_interfaces__srv__LedControl_Request__fini(duckie_interfaces__srv__LedControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // color
  rosidl_runtime_c__String__fini(&msg->color);
}

bool
duckie_interfaces__srv__LedControl_Request__are_equal(const duckie_interfaces__srv__LedControl_Request * lhs, const duckie_interfaces__srv__LedControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
duckie_interfaces__srv__LedControl_Request__copy(
  const duckie_interfaces__srv__LedControl_Request * input,
  duckie_interfaces__srv__LedControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

duckie_interfaces__srv__LedControl_Request *
duckie_interfaces__srv__LedControl_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Request * msg = (duckie_interfaces__srv__LedControl_Request *)allocator.allocate(sizeof(duckie_interfaces__srv__LedControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(duckie_interfaces__srv__LedControl_Request));
  bool success = duckie_interfaces__srv__LedControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
duckie_interfaces__srv__LedControl_Request__destroy(duckie_interfaces__srv__LedControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    duckie_interfaces__srv__LedControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
duckie_interfaces__srv__LedControl_Request__Sequence__init(duckie_interfaces__srv__LedControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Request * data = NULL;

  if (size) {
    data = (duckie_interfaces__srv__LedControl_Request *)allocator.zero_allocate(size, sizeof(duckie_interfaces__srv__LedControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = duckie_interfaces__srv__LedControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        duckie_interfaces__srv__LedControl_Request__fini(&data[i - 1]);
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
duckie_interfaces__srv__LedControl_Request__Sequence__fini(duckie_interfaces__srv__LedControl_Request__Sequence * array)
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
      duckie_interfaces__srv__LedControl_Request__fini(&array->data[i]);
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

duckie_interfaces__srv__LedControl_Request__Sequence *
duckie_interfaces__srv__LedControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Request__Sequence * array = (duckie_interfaces__srv__LedControl_Request__Sequence *)allocator.allocate(sizeof(duckie_interfaces__srv__LedControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = duckie_interfaces__srv__LedControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
duckie_interfaces__srv__LedControl_Request__Sequence__destroy(duckie_interfaces__srv__LedControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    duckie_interfaces__srv__LedControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
duckie_interfaces__srv__LedControl_Request__Sequence__are_equal(const duckie_interfaces__srv__LedControl_Request__Sequence * lhs, const duckie_interfaces__srv__LedControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!duckie_interfaces__srv__LedControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
duckie_interfaces__srv__LedControl_Request__Sequence__copy(
  const duckie_interfaces__srv__LedControl_Request__Sequence * input,
  duckie_interfaces__srv__LedControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(duckie_interfaces__srv__LedControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    duckie_interfaces__srv__LedControl_Request * data =
      (duckie_interfaces__srv__LedControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!duckie_interfaces__srv__LedControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          duckie_interfaces__srv__LedControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!duckie_interfaces__srv__LedControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
duckie_interfaces__srv__LedControl_Response__init(duckie_interfaces__srv__LedControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    duckie_interfaces__srv__LedControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
duckie_interfaces__srv__LedControl_Response__fini(duckie_interfaces__srv__LedControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
duckie_interfaces__srv__LedControl_Response__are_equal(const duckie_interfaces__srv__LedControl_Response * lhs, const duckie_interfaces__srv__LedControl_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
duckie_interfaces__srv__LedControl_Response__copy(
  const duckie_interfaces__srv__LedControl_Response * input,
  duckie_interfaces__srv__LedControl_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

duckie_interfaces__srv__LedControl_Response *
duckie_interfaces__srv__LedControl_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Response * msg = (duckie_interfaces__srv__LedControl_Response *)allocator.allocate(sizeof(duckie_interfaces__srv__LedControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(duckie_interfaces__srv__LedControl_Response));
  bool success = duckie_interfaces__srv__LedControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
duckie_interfaces__srv__LedControl_Response__destroy(duckie_interfaces__srv__LedControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    duckie_interfaces__srv__LedControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
duckie_interfaces__srv__LedControl_Response__Sequence__init(duckie_interfaces__srv__LedControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Response * data = NULL;

  if (size) {
    data = (duckie_interfaces__srv__LedControl_Response *)allocator.zero_allocate(size, sizeof(duckie_interfaces__srv__LedControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = duckie_interfaces__srv__LedControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        duckie_interfaces__srv__LedControl_Response__fini(&data[i - 1]);
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
duckie_interfaces__srv__LedControl_Response__Sequence__fini(duckie_interfaces__srv__LedControl_Response__Sequence * array)
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
      duckie_interfaces__srv__LedControl_Response__fini(&array->data[i]);
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

duckie_interfaces__srv__LedControl_Response__Sequence *
duckie_interfaces__srv__LedControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  duckie_interfaces__srv__LedControl_Response__Sequence * array = (duckie_interfaces__srv__LedControl_Response__Sequence *)allocator.allocate(sizeof(duckie_interfaces__srv__LedControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = duckie_interfaces__srv__LedControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
duckie_interfaces__srv__LedControl_Response__Sequence__destroy(duckie_interfaces__srv__LedControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    duckie_interfaces__srv__LedControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
duckie_interfaces__srv__LedControl_Response__Sequence__are_equal(const duckie_interfaces__srv__LedControl_Response__Sequence * lhs, const duckie_interfaces__srv__LedControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!duckie_interfaces__srv__LedControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
duckie_interfaces__srv__LedControl_Response__Sequence__copy(
  const duckie_interfaces__srv__LedControl_Response__Sequence * input,
  duckie_interfaces__srv__LedControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(duckie_interfaces__srv__LedControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    duckie_interfaces__srv__LedControl_Response * data =
      (duckie_interfaces__srv__LedControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!duckie_interfaces__srv__LedControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          duckie_interfaces__srv__LedControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!duckie_interfaces__srv__LedControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
