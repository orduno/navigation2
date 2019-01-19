// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_SIMPLE_PLANNER__LOGGER_HPP_
#define NAV2_SIMPLE_PLANNER__LOGGER_HPP_

#include <iostream>
#include <sstream>
#include <string>

namespace nav2_simple_planner
{

/*
 * An abstract class for logging
 */

enum class LogLevel {Normal, Debug, Error};

struct Logger
{
  explicit Logger(const LogLevel & logLevel)
  : logLevel_(logLevel),
    enabled_(true)
  {
  }

  virtual ~Logger() = default;

  virtual std::ostream & info() = 0;
  virtual std::ostream & warn() = 0;
  virtual std::ostream & debug() = 0;
  virtual std::ostream & error() = 0;

  virtual void info(const std::string & s) = 0;
  virtual void warn(const std::string & s) = 0;
  virtual void debug(const std::string & s) = 0;
  virtual void error(const std::string & s) = 0;

  virtual void info(const std::ostringstream & oss) = 0;
  virtual void warn(const std::ostringstream & oss) = 0;
  virtual void debug(const std::ostringstream & oss) = 0;
  virtual void error(const std::ostringstream & oss) = 0;

  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }

protected:
  LogLevel logLevel_;
  bool enabled_;
};

/*
 * A logger that outputs to the console
 */

struct ConsoleLogger : Logger
{
  std::ostringstream garbage_;

  explicit ConsoleLogger(const LogLevel & logLevel)
  : Logger(logLevel)
  {
  }

  std::ostream & info() override
  {
    if (!enabled_) return garbage_;

    if (logLevel_ != LogLevel::Error) {
      return std::cout << std::endl << "[info] ";
    }

    return garbage_;
  }

  std::ostream & warn() override
  {
    if (!enabled_) return garbage_;

    if (logLevel_ != LogLevel::Error) {
      return std::cout << std::endl << "[warn] ";
    }

    return garbage_;
  }

  std::ostream & debug() override
  {
    if (!enabled_) return garbage_;

    if (logLevel_ == LogLevel::Debug) {
      return std::cout << std::endl << "[debug] ";
    }

    return garbage_;
  }

  std::ostream & error() override
  {
    if (!enabled_) return garbage_;

    return std::cout << std::endl << "[error] ";
  }

  void info(const std::string & s) override
  {
    if (!enabled_) return;

    if (logLevel_ != LogLevel::Error) {
      std::cout << std::endl << "[info]  " << s;
    }
  }

  void warn(const std::string & s) override
  {
    if (!enabled_) return;

    if (logLevel_ != LogLevel::Error) {
      std::cout << std::endl << "[warn]  " << s;
    }
  }

  void debug(const std::string & s) override
  {
    if (!enabled_) return;

    if (logLevel_ == LogLevel::Debug) {
      std::cout << std::endl << "[debug] " << s;
    }
  }

  void error(const std::string & s) override
  {
    if (!enabled_) return;

    std::cout << std::endl << "[error] " << s;
  }

  void info(const std::ostringstream & oss) override
  {
    info(oss.str());
  }

  void warn(const std::ostringstream & oss) override
  {
    warn(oss.str());
  }

  void debug(const std::ostringstream & oss) override
  {
    debug(oss.str());
  }

  void error(const std::ostringstream & oss) override
  {
    error(oss.str());
  }
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__LOGGER_HPP_
