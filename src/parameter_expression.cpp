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

#include "parameter_expression/parameter_expression.hpp"

#include <muParser.h>

namespace parameter_expression
{

thread_local std::set<std::string> ParameterExpression::resolving_;

ParameterExpression::ParameterExpression(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  const std::string & name, const double default_value,
  rcl_interfaces::msg::ParameterDescriptor descriptor)
: node_parameters_interface_(node_parameters_interface),
  name_(name),
  parser_(),
  value_(),
  default_value_(default_value)
{
  using namespace std::placeholders;
  const auto parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(descriptor).set__dynamic_typing(true);
  const auto empty_value = rclcpp::ParameterValue();
  node_parameters_interface_->declare_parameter(name, empty_value, parameter_descriptor);

  // Route unknown variables in expressions to the ROS parameter registry via
  // this->resolve_variable. User data carries `this`.
  parser_.SetVarFactory(&ParameterExpression::var_factory, this);

  callback_handle_ = node_parameters_interface_->add_on_set_parameters_callback(
    std::bind(&ParameterExpression::on_parameter, this, _1));

  // Post-set callback: called AFTER a parameter set commits. Used to catch
  // "a param we depend on changed" and re-eval with the fresh value.
  // (The pre-set callback above returns stale get_parameter() results for
  // params other than the one being set, so it can't handle dep changes.)
  post_callback_handle_ = node_parameters_interface_->add_post_set_parameters_callback(
    std::bind(&ParameterExpression::on_post_parameter, this, _1));

  eval_first();
}

ParameterExpression::~ParameterExpression() {}

void ParameterExpression::eval_first()
{
  const auto parameter = node_parameters_interface_->get_parameter(name_);
  eval(parameter);
}

rcl_interfaces::msg::SetParametersResult ParameterExpression::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Pre-set callback: only validate own parameter changes.
  // Dep-change re-eval happens in on_post_parameter (post-set) where the
  // referenced parameters are already committed and get_parameter returns
  // the new value.
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_) {
      try {
        eval(parameter);
      } catch (const std::exception & e) {
        return rcl_interfaces::msg::SetParametersResult()
          .set__reason(std::string("Failed to evaluate expression: ") + e.what())
          .set__successful(false);
      } catch (const mu::Parser::exception_type & e) {
        return rcl_interfaces::msg::SetParametersResult()
          .set__reason(std::string("Failed to evaluate expression: ") + e.GetMsg())
          .set__successful(false);
      }
    }
  }
  return rcl_interfaces::msg::SetParametersResult().set__successful(true);
}

void ParameterExpression::on_post_parameter(const std::vector<rclcpp::Parameter> & parameters)
{
  // Fires after commit. If any of our current deps just changed, re-eval.
  // Pass `parameters` as an override snapshot so resolve_variable sees the
  // fresh values for the pending batch (get_parameter can lag inside
  // post-set on some rclcpp versions).
  bool dep_touched = false;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_) continue;
    if (ref_names_.count(parameter.get_name()) > 0) {
      dep_touched = true;
      break;
    }
  }
  if (!dep_touched) return;
  pending_snapshot_ = &parameters;
  try {
    eval(node_parameters_interface_->get_parameter(name_));
  } catch (...) {
    // Post-set can't reject; best effort. Prior value_ stays valid.
  }
  pending_snapshot_ = nullptr;
}

void ParameterExpression::eval(const rclcpp::Parameter parameter_value)
{
  using ParameterType = rcl_interfaces::msg::ParameterType;
  const auto ty = parameter_value.get_type();
  if (ty == ParameterType::PARAMETER_NOT_SET) {
    value_ = default_value_;
    ref_names_.clear();
    ref_values_.clear();
    return;
  }
  if (ty == ParameterType::PARAMETER_INTEGER) {
    value_ = static_cast<double>(parameter_value.as_int());
    ref_names_.clear();
    ref_values_.clear();
    return;
  }
  if (ty == ParameterType::PARAMETER_DOUBLE) {
    value_ = parameter_value.as_double();
    ref_names_.clear();
    ref_values_.clear();
    return;
  }

  if (ty != ParameterType::PARAMETER_STRING) {
    throw std::runtime_error("Parameter type is not string");
  }
  const auto expression = parameter_value.as_string();

  // Fresh backing storage and dependency set; both re-populated via
  // var_factory / resolve_variable during the Eval below.
  // ClearVar() forces muParser to re-parse and re-call var_factory even when
  // the expression string is unchanged (otherwise it caches the parsed AST
  // and keeps pointers to previously returned ref_values_ addresses, which
  // we invalidate here).
  ref_values_.clear();
  ref_names_.clear();
  parser_.ClearVar();

  // Cycle guard: mark self as being resolved so any recursion back to name_
  // through resolve_variable trips the check.
  resolving_.insert(name_);
  try {
    parser_.SetExpr(expression);
    value_ = parser_.Eval();
  } catch (...) {
    resolving_.erase(name_);
    throw;
  }
  resolving_.erase(name_);

  // Note: ref_names_ is populated by resolve_variable during the Eval above
  // (via var_factory). GetUsedVar() reports parser-known variables including
  // those registered by var_factory, so it would also work; but keeping the
  // set built incrementally in resolve_variable avoids relying on that
  // implementation detail.
}

double * ParameterExpression::var_factory(const mu::char_type * name, void * user_data)
{
  auto * self = static_cast<ParameterExpression *>(user_data);
  return self->resolve_variable(name);
}

double * ParameterExpression::resolve_variable(const std::string & name)
{
  if (resolving_.count(name) > 0) {
    throw mu::ParserError(std::string("Circular dependency in parameter expression: ") + name);
  }

  rclcpp::Parameter param;
  bool from_snapshot = false;
  if (pending_snapshot_ != nullptr) {
    // Post-set callback path: the referenced param may be one of the
    // parameters currently being committed. get_parameter can return stale
    // values in this context, so check the snapshot first.
    for (const auto & p : *pending_snapshot_) {
      if (p.get_name() == name) {
        param = p;
        from_snapshot = true;
        break;
      }
    }
  }
  if (!from_snapshot) {
    try {
      param = node_parameters_interface_->get_parameter(name);
    } catch (const std::exception & e) {
      throw mu::ParserError(
        std::string("Referenced parameter '") + name + "' not found: " + e.what());
    }
  }

  double resolved;
  using ParameterType = rcl_interfaces::msg::ParameterType;
  const auto ty = param.get_type();
  if (ty == ParameterType::PARAMETER_NOT_SET) {
    resolved = 0.0;
  } else if (ty == ParameterType::PARAMETER_INTEGER) {
    resolved = static_cast<double>(param.as_int());
  } else if (ty == ParameterType::PARAMETER_DOUBLE) {
    resolved = param.as_double();
  } else if (ty == ParameterType::PARAMETER_STRING) {
    resolving_.insert(name);
    try {
      resolved = eval_sub_expression(param.as_string());
    } catch (...) {
      resolving_.erase(name);
      throw;
    }
    resolving_.erase(name);
  } else {
    throw mu::ParserError(
      std::string("Referenced parameter '") + name +
      "' has unsupported type (must be int/double/string)");
  }

  ref_values_[name] = resolved;
  ref_names_.insert(name);
  return &ref_values_[name];
}

double ParameterExpression::eval_sub_expression(const std::string & expression)
{
  // Scratch parser so recursion does not clobber this->parser_'s SetExpr.
  // Same var_factory so nested references still route through the cycle
  // detection above.
  mu::Parser sub;
  sub.SetVarFactory(&ParameterExpression::var_factory, this);
  sub.SetExpr(expression);
  return sub.Eval();
}

double ParameterExpression::get() const { return value_; }
}  // namespace parameter_expression
