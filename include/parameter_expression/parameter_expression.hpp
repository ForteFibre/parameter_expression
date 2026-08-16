// Copyright 2025 ForteFibre
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <muParser.h>

#include <map>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <set>
#include <string>

namespace parameter_expression
{
// ROS 2 dynamic typed parameter with mathematical expression.
//
// Value types:
//   - int / double  : returned as-is
//   - string        : parsed by muParser, evaluated
//
// Cross-parameter references (feature/cross-param-reference):
//   Expressions may reference other parameters on the SAME node by name.
//   Example yaml:
//     r_wheel: 0.05
//     v_max_rps: 30.0
//     v_max_m_s: "r_wheel * v_max_rps * 2 * _pi"
//
//   Referenced parameters may themselves be expressions (recursive).
//   Circular dependencies (a->b->a) are detected and throw at eval time.
//   When a referenced parameter changes, this expression re-evaluates
//   automatically via the shared on_set_parameters_callback.
//
//   Built-in muParser constants (_pi, _e) are not treated as parameters.
//   Standard functions (sin, cos, atan2, ...) work as before.
class ParameterExpression
{
public:
  ParameterExpression(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
    const std::string & name, const double default_value,
    rcl_interfaces::msg::ParameterDescriptor descriptor);

  template <typename Node>
  ParameterExpression(
    Node & node, const std::string & name, const double default_value = 0.0,
    rcl_interfaces::msg::ParameterDescriptor descriptor =
      rcl_interfaces::msg::ParameterDescriptor())
  : ParameterExpression(
      rclcpp::node_interfaces::get_node_parameters_interface(node), name, default_value, descriptor)
  {
  }

  virtual ~ParameterExpression();

  double get() const;

  RCLCPP_SMART_PTR_DEFINITIONS(ParameterExpression)

private:
  rcl_interfaces::msg::SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> &);
  // Post-set: re-eval when a parameter we depend on has been committed.
  void on_post_parameter(const std::vector<rclcpp::Parameter> &);
  void eval(const rclcpp::Parameter parameter_value);

  // Optional lookup map used during eval to resolve variables from a snapshot
  // of pending param values (populated by post-set callback). Fixes the case
  // where NodeParametersInterface::get_parameter still returns stale values
  // inside post-set for the same batch that triggered the callback.
  const std::vector<rclcpp::Parameter> * pending_snapshot_{nullptr};

  void eval_first();

  // muParser variable factory. Called for each unknown variable name in the
  // expression. Routes to `this->resolve_variable(name)`. `user_data` is `this`.
  static double * var_factory(const mu::char_type * name, void * user_data);

  // Resolve a variable name by looking up a ROS parameter on the same node.
  // If the parameter itself is a string expression, evaluate it recursively.
  // Throws mu::ParserError if not found or on circular dependency.
  double * resolve_variable(const std::string & name);

  // Evaluate an arbitrary expression string using a scratch parser that
  // shares this->resolve_variable via var_factory. Used to unpack a
  // referenced parameter that is itself a string expression.
  double eval_sub_expression(const std::string & expression);

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_callback_handle_;
  rclcpp::Parameter parameter_;
  const std::string name_;
  mu::Parser parser_;
  double value_;
  const double default_value_;

  // Backing storage for muParser DefineVar. muParser holds pointers into
  // this map; std::map guarantees pointers to existing elements remain
  // valid across insertions.
  std::map<std::string, double> ref_values_;

  // Names of ROS parameters this expression currently depends on. Populated
  // after each successful Eval via GetUsedVar. Used by on_parameter to
  // re-evaluate when any of them changes.
  std::set<std::string> ref_names_;

  // Cycle detection stack. thread_local because recursive eval_sub_expression
  // can traverse chains of expressions across multiple ParameterExpression
  // instances that share the resolve path.
  static thread_local std::set<std::string> resolving_;
};
}  // namespace parameter_expression
