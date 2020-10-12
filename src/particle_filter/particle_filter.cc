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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using math_util::AngleDiff;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 8, "Number of particles");

namespace {
  int updates_since_last_resample_ = 0;
  Vector2f last_update_loc_(0,0);
  Vector2f last_resample_loc_(0,0);
} // namespace

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  max_log_particle_weight_ = 0;
  // should we do this or not?
  // if (particles_.empty()) return;

  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  Vector2f odom_trans_diff = odom_loc - prev_odom_loc_;

  // Only executes if odom is initialized and a realistic value
  if (odom_initialized_ and odom_trans_diff.norm() < 1.0)
  {
    float angle_diff = AngleDiff(odom_angle, prev_odom_angle_);
    if (std::abs(angle_diff) > M_2PI)
    {
      // Should never happen, but just in case:
      cout << "Error: reported change in angle exceeds 2pi" << endl;
    }
    for (auto &particle : particles_)
    {
      // Find the transformation between the map and odom frame for this particle
      Eigen::Rotation2Df R_Odom2Map(AngleDiff(particle.angle, prev_odom_angle_));
      Vector2f map_trans_diff = R_Odom2Map * odom_trans_diff;
      // Apply noise to pose of particle
      UpdateParticleLocation(map_trans_diff, angle_diff, &particle);
    }
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
  }
  //if it's false 
  //set prev odom loc and angle to be latest odom values
  //set bool flag to true and return
  else
  {
    ResetOdomVariables(odom_loc, odom_angle);
    odom_initialized_ = true;
    cout << "Odom reset due to initialization or large movement." << endl;
  }
}

void ParticleFilter::UpdateParticleLocation(Vector2f map_trans_diff, float dtheta_odom, Particle* p_ptr)
{
  // Use the motion model to update each particle's location
  // This function will probably be called in the ObserveOdometry callback
  // You can update the particle location directly by modifying the particle variable
  // defined above since it was passed by reference (using the "&" symbol).
  // this particle passed by reference comes from ObserveLaser for loop
  // and is modified by Update function similar to how it is being modified here
  // but this occurs at every timestep

  // noise constants to tune
  // Noise constants to tune
  const float k1 = 0.40;  // translation error per unit translation (suggested: 0.1-0.2) was 1
  const float k2 = 0.02;  // translation error per unit rotation (suggested: 0.01) was 0.25
  const float k3 = 0.20;  // angular error per unit translation (suggested: 0.02-0.1) was 0.5
  const float k4 = 0.40;  // angular error per unit rotation (suggested: 0.05-0.2) was 1
  
  Particle& particle = *p_ptr;
  const float abs_angle_diff = abs(dtheta_odom);

  //should the mean b
  //is this how it should be, the meant is the same but the standard deviation, sigma, changes based on k constants
  float translation_noise_x = rng_.Gaussian(0.0,k1*map_trans_diff.norm() + k2*abs_angle_diff);
  // future improvements wll use different constants for x and y to account for difference in slipping likelihood
  float translation_noise_y = rng_.Gaussian(0.0,k1*map_trans_diff.norm() + k2*abs_angle_diff);
  float rotation_noise = rng_.Gaussian(0.0,k3*map_trans_diff.norm() + k4*abs_angle_diff);
  particle.loc += map_trans_diff + Vector2f(translation_noise_x,translation_noise_y);
  particle.angle += dtheta_odom + rotation_noise;

  // cout << particle.loc << endl;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  particles_.clear(); // Need to get rid of particles from previous inits
  map_.Load("maps/" + map_file + ".txt"); // from Piazza
  odom_initialized_ = false;
  ResetOdomVariables(loc, angle);

  // Make initial guesses (particles) based on a Gaussian distribution about initial placement
  for (size_t i = 0; i < FLAGS_num_particles; i++){
    Particle particle_init;
    particle_init.loc.x() = rng_.Gaussian(loc.x(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.loc.y() = rng_.Gaussian(loc.y(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.angle   = rng_.Gaussian(angle, M_PI/6);  // std_dev of 30deg, to be tuned
    particle_init.log_weight = 0;
    particles_.push_back(particle_init);
  }
}


// Called when new pose is set or robot is moved substantially ("kidnapped")
void ParticleFilter::ResetOdomVariables(const Vector2f loc, const float angle) {
  init_offset_angle_ = angle - prev_odom_angle_;
  last_update_loc_ = loc;
  last_resample_loc_ = loc;
  prev_odom_loc_ = loc;
  prev_odom_angle_ = angle;
  updates_since_last_resample_ = 0;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them.

  // Just do weighted average of loc and angle
  Vector2f weighted_loc_sum(0.0, 0.0);
  float weighted_angle_sum = 0.0;
  float weight_sum = 0.0;
  for (auto &particle : particles_)
  {

    // Convert from log weight to normalized weight

    // TODO: how do i introduce max_log_particle_weight_ without also using ObserveLaser?

    // float normalized_log_weight = particle.log_weight - max_log_particle_weight_;
    float normalized_log_weight = particle.log_weight - max_log_particle_weight_;
    float normalized_weight = exp(normalized_log_weight);
    weighted_loc_sum += particle.loc * normalized_weight;
    weighted_angle_sum += particle.angle * normalized_weight;
    weight_sum += normalized_weight;
  }
  loc = weighted_loc_sum / weight_sum;
  angle = weighted_angle_sum / weight_sum;
}


}  // namespace particle_filter
