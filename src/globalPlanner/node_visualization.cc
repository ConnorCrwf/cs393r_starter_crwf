//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    node_visualization.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "node_visualization.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using visualization::DrawPoint;
using visualization::ClearVisualizationMsg;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace node_visualization {

NodeVisualization::NodeVisualization(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void NodeVisualization::SetNavGoal(const Vector2f& loc, float angle) {
}

void NodeVisualization::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void NodeVisualization::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void NodeVisualization::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

void NodeVisualization::initGrid(){
const double minX = -50, maxX = 50, minY= -50, maxY = 50;
const double xInc = 2.0;
const double yInc = 2.0;
for(double i = minX; i <= maxX; i += xInc){
    vector<Vector2f> a;
    for(double j = minY; j <= maxY; j+= yInc){
        a.push_back(Vector2f(i, j));
        }
    grid_.push_back(a);
    }

}

void NodeVisualization::DrawGrid(vector<vector<Vector2f>> grid){
for(const auto v: grid){
    for(const auto b: v){
        DrawPoint(b,0xFF00FF,global_viz_msg_);
        }
    }
}

void NodeVisualization::Run() {
    initGrid();
    DrawGrid(grid_);
    viz_pub_.publish(global_viz_msg_);
    ClearVisualizationMsg(global_viz_msg_);
}


}  // namespace node_visualization
