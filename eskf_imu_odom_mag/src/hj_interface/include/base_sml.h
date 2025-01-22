// @file base_sml.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#ifndef BASE_SML_H
#define BASE_SML_H
#include <boost/sml.hpp>
#include <cassert>
#include <iostream>
#include <functional>
#define CHANGE_NAME(name) hj_##name
#define GET_STRUCT_NAME(name) CHANGE_NAME(name)

#define GET_STATE_EVENT_NAME(in) sml::aux::get_type_name<in>()

#define HJ_SML_INIT(name) sml::sm<transitions> sm{};
/*//创建事件
 * \param event_name
 *      事件名
 */
#define HJ_EVENT(event_name) \
  struct event_name {};

//#define HJ_STATE(state_name) const auto state_name = sml::state<class state_name>;
#define HJ_STATE(state_name) struct state_name {};
/*//创建回调struct
 * \param name
 *      回调函数类名
 */
#define HJ_CALLBACK_DEFINE(name) \
  struct name {                  \
    void operator()();           \
  };
/*  //实现回调operator
 * \param name
 *      回调函数类名
 */
#define HJ_CALLBACK_INSTANCE(name) void name::operator()()
/*  //创建guard回调struct
 * \param name
 *      guard回调函数类名
 */
#define HJ_GUARD_CALLBACK_DEFINE(name) \
  struct name {                        \
    bool operator()();                 \
  };
/*  //实现guard回调operator
 * \param name
 *      guard回调函数类名
 */
#define HJ_GUARD_CALLBACK_INSTANCE(name) bool name::operator()()
/*  //创建pre_action回调struct
 * \param name
 *      pre_action回调函数类名
 */
#define HJ_PRE_ACTION_CALLBACK_DEFINE(name) \
  struct name {                        \
    void operator()();                 \
  };
/*  //实现pre_action回调operator
 * \param name
 *      pre_action回调函数类名
 */
#define HJ_PRE_ACTION_CALLBACK_INSTANCE(name) void name::operator()()

#define HJ_EMPTY_ACTION() ([]() {})
#define HJ_EMPTY_GUARD() ([]() -> bool { return true; })
/*
 * \param sml
 *      状态机实例
 * \param event_name
 *      事件名
 */
#define HJ_EVENT_PROCESS(sml, event_name) sml.process_event(event_name{})
/*
 * \param state_src
 *      原状态
 * \param event_name
 *      事件名
 * \param guard_func
 *      guard回调，返回true代表可切换状态，返回false代表维持现在状态
 * \param action_func
 *      原状态切到目标状态前做的回调函数
 * \param state_dst
 *      目标状态
 */
#define HJ_TRANSITION_BEGIN_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  *state_src##_s + event<event_name>[guard_func] / action_func = state_dst##_s

#define HJ_TRANSITION_CLASS_BEGIN_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  *state<state_src> + event<event_name>[guard_func] / action_func = state<state_dst>
/*
 * \param state_src
 *      原状态
 * \param event_name
 *      事件名
 * \param guard_func
 *      guard回调，返回true代表可切换状态，返回false代表维持现在状态
 * \param action_func
 *      原状态切到目标状态后做的回调函数
* \param pre_action
 *      原状态切到目标状态前做的回调函数
 * \param state_dst
 *      目标状态
 */
#define HJ_TRANSITION_EXIT_ITEM(state_dst, exit_func) , state_dst##_s + sml::on_exit<_> / exit_func
#define HJ_TRANSITION_CLASS_EXIT_ITEM(state_dst, exit_func) , state<state_dst> + sml::on_exit<_> / exit_func

#define HJ_TRANSITION_CLASS_ENTRY_ITEM(state_dst, entry_func) , state<state_dst> + sml::on_entry<_> / entry_func

#define HJ_TRANSITION_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  , state_src##_s + event<event_name>[guard_func] / action_func = state_dst##_s

#define HJ_TRANSITION_CLASS_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  , state<state_src> + event<event_name>[guard_func] / action_func = state<state_dst>

#define HJ_TRANSITION_CLASS_PRE_ACTION_ITEM(state_src, event_name, guard_func, pre_action, action_func, state_dst) \
  , state<state_src> + event<event_name>[guard_func && []() ->bool { pre_action(); return true; }] / action_func = state<state_dst>
/*
 * \param state_src
 *      原状态
 * \param event_name
 *      事件名
 * \param guard_func
 *      guard回调，返回true代表可切换状态，返回false代表维持现在状态
 * \param action_func
 *      原状态切到目标状态前做的回调函数
 */
#define HJ_TRANSITION_END_ITEM(state_src, event_name, guard_func, action_func) \
  , state_src##_s + event<event_name>[guard_func] / action_func = X
#define HJ_TRANSITION_CLASS_END_ITEM(state_src, event_name, guard_func, action_func) \
  , state<state_src> + event<event_name>[guard_func] / action_func = X
/*
 * 创建状态迁移表
 */
#define HJ_TRANSITION_TABLE(...) return make_transition_table(__VA_ARGS__);
/*
 * \param name
 *      类名
 * \param transition
 *      状态迁移表
 */
#define HJ_SML_STRUCT(name, transition) \
  struct name {                         \
    auto operator()() const noexcept {  \
      using namespace sml;              \
      transition                        \
    }                                   \
  };
/*
 * \param transition
 *      状态迁移表
 */
#define HJ_SML_OPERATOR(transition)  \
  auto operator()() const noexcept { \
    using namespace sml;             \
    transition                       \
  }

#endif
