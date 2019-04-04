// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_controller/nav2_controller_client.hpp"

using namespace std::chrono_literals;

struct xytheta {
  double x;
  double y;
  double theta;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  nav2_controller::Nav2ControllerClient client;

  // Create a set of target poses across the map
  std::vector<xytheta> target_poses;
  target_poses.push_back({0.94, -0.55, 0});
  target_poses.push_back({1.7, 0.5, 1.4});
  target_poses.push_back({0.97, 1.68, 2.94});
  target_poses.push_back({0.02, 1.74, -2.9});

  // Wait for a few seconds to let all of the nodes come up
  std::this_thread::sleep_for(5s);

  // Start the nav2 system, bringing it to the ACTIVE state
  client.startup();

  // Set the robot's starting pose (approximately where it comes up in gazebo)
  client.set_initial_pose(-2.0, -0.5, 0);

  // Wait for a couple secs to let the rviz display update
  std::this_thread::sleep_for(2s);

  // Navigate through all of the poses
  for (std::vector<xytheta>::size_type i=0; i<target_poses.size(); i++) {
    auto pose = target_poses[i];
    auto ok = client.navigate_to_pose(pose.x, pose.y, pose.theta);

    if (!ok) {
      printf("Navigation FAILED!\n");
      break;
    }
  }

  // Shut down the nav2 system, bringing it to the FINALIZED state
  client.shutdown();

  rclcpp::shutdown();
  return 0;
}
