// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:action/Patrol.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__PATROL__BUILDER_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__PATROL__BUILDER_HPP_

#include "custom_interfaces/action/detail/patrol__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_Goal_leg_number
{
public:
  explicit Init_Patrol_Goal_leg_number(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Patrol_Goal leg_number(::custom_interfaces::action::Patrol_Goal::_leg_number_type arg)
  {
    msg_.leg_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut9
{
public:
  explicit Init_Patrol_Goal_revolut9(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_leg_number revolut9(::custom_interfaces::action::Patrol_Goal::_revolut9_type arg)
  {
    msg_.revolut9 = std::move(arg);
    return Init_Patrol_Goal_leg_number(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut8
{
public:
  explicit Init_Patrol_Goal_revolut8(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut9 revolut8(::custom_interfaces::action::Patrol_Goal::_revolut8_type arg)
  {
    msg_.revolut8 = std::move(arg);
    return Init_Patrol_Goal_revolut9(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut7
{
public:
  explicit Init_Patrol_Goal_revolut7(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut8 revolut7(::custom_interfaces::action::Patrol_Goal::_revolut7_type arg)
  {
    msg_.revolut7 = std::move(arg);
    return Init_Patrol_Goal_revolut8(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut6
{
public:
  explicit Init_Patrol_Goal_revolut6(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut7 revolut6(::custom_interfaces::action::Patrol_Goal::_revolut6_type arg)
  {
    msg_.revolut6 = std::move(arg);
    return Init_Patrol_Goal_revolut7(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut5
{
public:
  explicit Init_Patrol_Goal_revolut5(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut6 revolut5(::custom_interfaces::action::Patrol_Goal::_revolut5_type arg)
  {
    msg_.revolut5 = std::move(arg);
    return Init_Patrol_Goal_revolut6(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut4
{
public:
  explicit Init_Patrol_Goal_revolut4(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut5 revolut4(::custom_interfaces::action::Patrol_Goal::_revolut4_type arg)
  {
    msg_.revolut4 = std::move(arg);
    return Init_Patrol_Goal_revolut5(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut3
{
public:
  explicit Init_Patrol_Goal_revolut3(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut4 revolut3(::custom_interfaces::action::Patrol_Goal::_revolut3_type arg)
  {
    msg_.revolut3 = std::move(arg);
    return Init_Patrol_Goal_revolut4(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut2
{
public:
  explicit Init_Patrol_Goal_revolut2(::custom_interfaces::action::Patrol_Goal & msg)
  : msg_(msg)
  {}
  Init_Patrol_Goal_revolut3 revolut2(::custom_interfaces::action::Patrol_Goal::_revolut2_type arg)
  {
    msg_.revolut2 = std::move(arg);
    return Init_Patrol_Goal_revolut3(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

class Init_Patrol_Goal_revolut1
{
public:
  Init_Patrol_Goal_revolut1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_Goal_revolut2 revolut1(::custom_interfaces::action::Patrol_Goal::_revolut1_type arg)
  {
    msg_.revolut1 = std::move(arg);
    return Init_Patrol_Goal_revolut2(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_Goal>()
{
  return custom_interfaces::action::builder::Init_Patrol_Goal_revolut1();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_Result_success
{
public:
  Init_Patrol_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Patrol_Result success(::custom_interfaces::action::Patrol_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_Result>()
{
  return custom_interfaces::action::builder::Init_Patrol_Result_success();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_Feedback_time_left
{
public:
  Init_Patrol_Feedback_time_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Patrol_Feedback time_left(::custom_interfaces::action::Patrol_Feedback::_time_left_type arg)
  {
    msg_.time_left = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_Feedback>()
{
  return custom_interfaces::action::builder::Init_Patrol_Feedback_time_left();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_SendGoal_Request_goal
{
public:
  explicit Init_Patrol_SendGoal_Request_goal(::custom_interfaces::action::Patrol_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Patrol_SendGoal_Request goal(::custom_interfaces::action::Patrol_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_SendGoal_Request msg_;
};

class Init_Patrol_SendGoal_Request_goal_id
{
public:
  Init_Patrol_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_SendGoal_Request_goal goal_id(::custom_interfaces::action::Patrol_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Patrol_SendGoal_Request_goal(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_SendGoal_Request>()
{
  return custom_interfaces::action::builder::Init_Patrol_SendGoal_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_SendGoal_Response_stamp
{
public:
  explicit Init_Patrol_SendGoal_Response_stamp(::custom_interfaces::action::Patrol_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Patrol_SendGoal_Response stamp(::custom_interfaces::action::Patrol_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_SendGoal_Response msg_;
};

class Init_Patrol_SendGoal_Response_accepted
{
public:
  Init_Patrol_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_SendGoal_Response_stamp accepted(::custom_interfaces::action::Patrol_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Patrol_SendGoal_Response_stamp(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_SendGoal_Response>()
{
  return custom_interfaces::action::builder::Init_Patrol_SendGoal_Response_accepted();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_GetResult_Request_goal_id
{
public:
  Init_Patrol_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Patrol_GetResult_Request goal_id(::custom_interfaces::action::Patrol_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_GetResult_Request>()
{
  return custom_interfaces::action::builder::Init_Patrol_GetResult_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_GetResult_Response_result
{
public:
  explicit Init_Patrol_GetResult_Response_result(::custom_interfaces::action::Patrol_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Patrol_GetResult_Response result(::custom_interfaces::action::Patrol_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_GetResult_Response msg_;
};

class Init_Patrol_GetResult_Response_status
{
public:
  Init_Patrol_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_GetResult_Response_result status(::custom_interfaces::action::Patrol_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Patrol_GetResult_Response_result(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_GetResult_Response>()
{
  return custom_interfaces::action::builder::Init_Patrol_GetResult_Response_status();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Patrol_FeedbackMessage_feedback
{
public:
  explicit Init_Patrol_FeedbackMessage_feedback(::custom_interfaces::action::Patrol_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Patrol_FeedbackMessage feedback(::custom_interfaces::action::Patrol_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_FeedbackMessage msg_;
};

class Init_Patrol_FeedbackMessage_goal_id
{
public:
  Init_Patrol_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_FeedbackMessage_feedback goal_id(::custom_interfaces::action::Patrol_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Patrol_FeedbackMessage_feedback(msg_);
  }

private:
  ::custom_interfaces::action::Patrol_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Patrol_FeedbackMessage>()
{
  return custom_interfaces::action::builder::Init_Patrol_FeedbackMessage_goal_id();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__PATROL__BUILDER_HPP_
