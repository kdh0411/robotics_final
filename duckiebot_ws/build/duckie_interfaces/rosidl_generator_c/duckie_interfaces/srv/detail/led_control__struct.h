// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from duckie_interfaces:srv/LedControl.idl
// generated code does not contain a copyright notice

#ifndef DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__STRUCT_H_
#define DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LedControl in the package duckie_interfaces.
typedef struct duckie_interfaces__srv__LedControl_Request
{
  rosidl_runtime_c__String color;
} duckie_interfaces__srv__LedControl_Request;

// Struct for a sequence of duckie_interfaces__srv__LedControl_Request.
typedef struct duckie_interfaces__srv__LedControl_Request__Sequence
{
  duckie_interfaces__srv__LedControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} duckie_interfaces__srv__LedControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LedControl in the package duckie_interfaces.
typedef struct duckie_interfaces__srv__LedControl_Response
{
  bool success;
  rosidl_runtime_c__String message;
} duckie_interfaces__srv__LedControl_Response;

// Struct for a sequence of duckie_interfaces__srv__LedControl_Response.
typedef struct duckie_interfaces__srv__LedControl_Response__Sequence
{
  duckie_interfaces__srv__LedControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} duckie_interfaces__srv__LedControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DUCKIE_INTERFACES__SRV__DETAIL__LED_CONTROL__STRUCT_H_
