// Copyright (c) 2019 Intel Corporation
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

#include "nav2_util/rate_constraint.hpp"

#include <sys/resource.h>
#include <sys/stat.h>

#include "rclcpp/time.hpp"

namespace nav2_util
{

RateConstraint::RateConstraint(
  std::string id, uint32_t rate, uint32_t jitter_margin,
  std::function<void(int iter_num, rclcpp::Duration looptime)> cb)

: rate_(rate), jitter_margin_(jitter_margin), overrun_cb_(cb)
{
  uint32_t period_ns = 1000000000 / rate;
  uint32_t jitter_ns = (period_ns * jitter_margin) / 100;

  acceptable_looptime_ = rclcpp::Duration(0, period_ns + jitter_ns);

  std::string directory = "/tmp/nav2/";
  mkdir(directory.c_str(), 0700);

  // TODO(mjeronimo): Create a unique file name?
  std::string filename = directory + id + ".log";

  if ((log_file_ = fopen(filename.c_str(), "w")) == nullptr) {
    throw std::runtime_error("Error: Could not open log file");
  }

  fprintf(log_file_, "Name: %s\n", id.c_str());
  fprintf(log_file_, "Desired rate: %ldns \n", (long)period_ns);
  fprintf(log_file_, "Jitter margin: %ld%% \n", (long)jitter_margin);
}

RateConstraint::~RateConstraint()
{
  fclose(log_file_);
}

void
RateConstraint::calc_looptime()
{
  if (first_time_) {
    start_ = prev_looptime_ = clock_.now();
	first_time_ = false;
	return;
  }

  // TODO(mjeronimo): check if now is before prev_looptime
  // TODO(mjeronimo): use C++ steady clock
  auto now = clock_.now();

  fprintf(log_file_, "Iteration: %d ", iter_cnt_);

  rclcpp::Duration looptime = now - prev_looptime_;
  print_duration(looptime);

  if (looptime > acceptable_looptime_ && overrun_cb_) {
    print_metrics();
    overrun_cb_(iter_cnt_, looptime);
  }

  prev_looptime_ = now;
  iter_cnt_++;
}

void
RateConstraint::print_duration(rclcpp::Duration duration)
{
  uint32_t nsecs = (duration.nanoseconds()) % 1000000000;
  uint32_t secs = ((duration.nanoseconds()) - nsecs) / 1000000000;

  fprintf(log_file_, "Looptime: %d secs %d nsecs\n", secs, nsecs);
}

void
RateConstraint::print_metrics()
{
  struct rusage r_usage;
  if (getrusage(RUSAGE_SELF, &r_usage) != 0) {
    return;
  }

  fprintf(log_file_, "Minor pagefaults: %lu\n", r_usage.ru_minflt);
  fprintf(log_file_, "Major pagefaults: %lu\n", r_usage.ru_majflt);
  fprintf(log_file_, "Memory usage: %lu\n", r_usage.ru_maxrss);
}

#if 0
  rclcpp::Duration looptime(0, 0);
  // MJ:
  auto target_looptime_ns = (long int) (1000000000 / rate_);
  auto target_duration_for_previous = rclcpp::Duration(((iter_cnt_ - 1) * target_looptime_ns));

  printf("tdp       : %ld\n", target_duration_for_previous.nanoseconds());
  printf("\n");
  printf("start     : %ld\n", start_.nanoseconds());
  printf("now       : %ld\n", now.nanoseconds());
  printf("start+tdp : %ld\n", (start_ + target_duration_for_previous).nanoseconds());
  printf("now-start : %ld\n", now.nanoseconds() - start_.nanoseconds());

  looptime = now - (start_ + target_duration_for_previous);

  printf("t_looptime: %ld\n", target_looptime_ns);
  printf("looptime  : %ld\n", looptime.nanoseconds());
  printf("\n");
#endif

}  // namespace nav2_util
