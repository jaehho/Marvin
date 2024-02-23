// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/PoseLandmark.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/pose_landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_PoseLandmark_point
{
public:
  explicit Init_PoseLandmark_point(::custom_interfaces::msg::PoseLandmark & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::PoseLandmark point(::custom_interfaces::msg::PoseLandmark::_point_type arg)
  {
    msg_.point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::PoseLandmark msg_;
};

class Init_PoseLandmark_label
{
public:
  Init_PoseLandmark_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseLandmark_point label(::custom_interfaces::msg::PoseLandmark::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_PoseLandmark_point(msg_);
  }

private:
  ::custom_interfaces::msg::PoseLandmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::PoseLandmark>()
{
  return custom_interfaces::msg::builder::Init_PoseLandmark_label();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__POSE_LANDMARK__BUILDER_HPP_
