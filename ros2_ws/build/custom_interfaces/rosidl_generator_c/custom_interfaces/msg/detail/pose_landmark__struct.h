// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/PoseLandmark.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/PoseLandmark in the package custom_interfaces.
typedef struct custom_interfaces__msg__PoseLandmark
{
  rosidl_runtime_c__String label;
  geometry_msgs__msg__Point point;
} custom_interfaces__msg__PoseLandmark;

// Struct for a sequence of custom_interfaces__msg__PoseLandmark.
typedef struct custom_interfaces__msg__PoseLandmark__Sequence
{
  custom_interfaces__msg__PoseLandmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__PoseLandmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__STRUCT_H_
