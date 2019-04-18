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

#include "nav2_util/real_time/real_time_monitor.hpp"

#include <sys/resource.h>

void RealTimeMonitor::print_duration(FILE * log_file_, rclcpp::Duration dur)
{
  uint32_t nsecs = (dur.nanoseconds()) % 1000000000;
  uint32_t secs = ((dur.nanoseconds()) - nsecs) / 1000000000;
  fprintf(log_file_, "Looptime: %d secs %d nsecs\n", secs, nsecs);
}

void RealTimeMonitor::print_metrics(FILE * log_file_)
{
  struct rusage r_usage;

  if (getrusage(RUSAGE_SELF, &r_usage) != 0) {
    return;
  }

  fprintf(log_file_, "Minor pagefaults: %lu\n", r_usage.ru_minflt);
  fprintf(log_file_, "Major pagefaults: %lu\n", r_usage.ru_majflt);
  fprintf(log_file_, "Memory usage: %lu\n", r_usage.ru_maxrss);
}

RealTimeMonitor::RealTimeMonitor()
{
}

RealTimeMonitor::~RealTimeMonitor()
{
  for (std::map<std::string, RealTimeData *>::iterator it=rtd_map_.begin(); it!=rtd_map_.end(); ++it) {
    if (it->second->log_file_) {
      fclose(it->second->log_file_);
    }
  }
}

int RealTimeMonitor::init(std::string id)
{
  RealTimeData *rtd = new RealTimeData();

  //TODO:: Check if file exists
  std::string filename = "/tmp/log_" + id + ".txt";
  rtd->log_file_ = fopen(filename.c_str(), "w");

  if (rtd->log_file_ == NULL) {
   printf("Error: Could not open log file");
  }

  rtd->prev_looptime_ = rclcpp::Time(0,0);
  rtd->init_ = true;

  rtd_map_[id] = rtd;
  return 0;
}

int RealTimeMonitor::init(std::string id, uint32_t rate, uint32_t jitter_margin,
                           std::function<void(int iter_num, rclcpp::Duration looptime)> cb)
{
  int ret;
  if ((ret = init(id)))
    return ret;

  RealTimeData *rtd;
  std::map<std::string, RealTimeData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
      rtd = it->second;
  } else {
      printf("Error: Initialization error %s", id.c_str());
      return -1;
  }

  //iter_cnt_ = 0;
  rtd->rate_ = rate;
  rtd->jitter_margin_ = jitter_margin;
  rtd->overrun_cb_ = cb;
  uint32_t looptime_ns = 1000000000/rate;
  uint32_t jitter_ns = (looptime_ns*jitter_margin)/100;
  uint32_t desired_looptime_ns = looptime_ns + jitter_ns;
  rtd->acceptable_looptime_ = rclcpp::Duration(0, desired_looptime_ns);
  fprintf(rtd->log_file_, "Desired looptime:%ld ns \n", long(rtd->acceptable_looptime_.nanoseconds()));

  return 0;
}

int RealTimeMonitor::deinit(std::string id)
{
  (void)(id);
  return 0;
}

int RealTimeMonitor::calc_looptime(std::string id, rclcpp::Time now)
{
  //printf("calc_looptime\n");

  RealTimeData * rtd;
  std::map<std::string, RealTimeData *>::iterator it = rtd_map_.find(id);

  if (it != rtd_map_.end()) {
      rtd = it->second;
  } else {
      printf("Error: No such topic monitored %s", id.c_str());
      return -1;
  }
  
  rclcpp::Duration looptime(0,0);

/*
  if (now < prev_looptime_) {
    printf("Invalid argument");
    return -1;
  }
*/

  //if (iter_cnt_ != 0) {
  if (!rtd->init_) {
    looptime = now - rtd->prev_looptime_;
  } else {
    rtd->init_ = false;
  }

  fprintf(rtd->log_file_, "Iteration: %d ", rtd->iter_cnt_);
  print_duration(rtd->log_file_, looptime);

  if (rtd->overrun_cb_) {
    if (looptime > rtd->acceptable_looptime_) {
      rtd->overrun_cb_(rtd->iter_cnt_, looptime);
      print_metrics(rtd->log_file_);
    }
  }

  //printf("looptime: %ld\n", looptime.nanoseconds());
  //printf("acceptable_looptime: %ld\n", rtd->acceptable_looptime_.nanoseconds());

#if 0
nav2_msgs::msg::LoopTime loop_time_msg;
loop_time_msg.header.stamp = now();
loop_time_msg.header.frame_id = "map"; 
loop_time_msg.topic = "Foobar";
loop_time_msg.pub = true;
loop_time_msg.rate = 0;
loop_time_msg.jitter = 0;
loop_time_msg.iteration = 0;
loop_time_msg.looptime = 0;

loop_time_pub_->publish(loop_time_msg);

std_msgs/Header header              # timestamp in header is the time when msg was dispatched
string topic                        # Name of the topic
bool pub                            # True if time captured at publisher end, false if subscription
int32 rate                          # Desired rate
int32 jitter                        # Acceptable jitter in percentage
int32 iteration                     # Iteration count
uint64 looptime                     # Looptime value in nanosecs

loop_time_pub_.publish(loop_time_msg);
#endif

  rtd->prev_looptime_ = now;
  rtd->iter_cnt_++;

  return 0;
}

int RealTimeMonitor::calc_latency(std::string /*id*/, builtin_interfaces::msg::Time & time, rclcpp::Time now)
{
  rclcpp::Duration latency(0,0);
  rclcpp::Time msg_time(time.sec,time.nanosec);

  latency = now - msg_time;
  return latency.nanoseconds();
}

