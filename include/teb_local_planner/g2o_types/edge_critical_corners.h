/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_CRITICAL_CORNERS_H_
#define EDGE_CRITICAL_CORNERS_H_

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>



namespace teb_local_planner
{

/**
 * @class EdgeCriticalCorners
 * @brief Edge defining the cost function for keeping a minimum distance from obstacles.
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point ) \cdot weight \f$. \n
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeInflatedObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */     
class EdgeCriticalCorners : public BaseTebMultiEdge<1, const Obstacle*>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeCriticalCorners() 
  {
    _measurement = NULL;
    this->resize(3);
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeCriticalCorners()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    double dist1 = robot_model_->calculateDistance(conf1->pose(), _measurement);
    double dist2 = robot_model_->calculateDistance(conf2->pose(), _measurement);

    // compute velocity
    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
    
    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();  

    // Original obstacle cost.
    //_error[0] = penaltyBoundCC(dist1, vel, cfg_->obstacles.critical_corner_vel_coeff, cfg_->optim.penalty_epsilon);
    //_error[0] = penaltyBoundFromBelow(dist1, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
    //_error[1] = penaltyBoundCC(vel, mean_dist, cfg_->obstacles.critical_corner_vel_coeff, cfg_->obstacles.critical_corner_epsilon);

    //_error[0] = std::pow(penaltyBoundFromAbove(vel, dist1*cfg_->obstacles.critical_corner_vel_coeff, cfg_->optim.penalty_epsilon),2);
    //_error[1] = std::pow(penaltyBoundFromBelow(dist1, vel*cfg_->obstacles.critical_corner_dist_coeff, cfg_->optim.penalty_epsilon),2);
    
    // check rel. distance (<1: closer then min_dist; ==1: at min_dist; >1: further away)
    double scale = 0;
    if (cfg_->obstacles.critical_corner_dist > 0)
        scale = dist1 / cfg_->obstacles.critical_corner_dist;

    // allow quadratic increase of velocity, if far away
    // (otherwise: within dist_min max. velocity is linear decreased)
    if (scale > 1)
        scale = std::pow(scale, 2);
        //scale = (1 + std::pow(scale, 2)) / 2;
    
    // calculate max. allowed velocity
    double max_vel = cfg_->obstacles.critical_corner_vel * scale;

    _error[0] = penaltyBoundFromAbove(vel, max_vel, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeCriticalCorners::computeError() _error[0]=%f \n",_error[0]);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }
    
  /**
   * @brief Set pointer to the robot model 
   * @param robot_model Robot model required for distance calculation
   */ 
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }
  
protected:

  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  

    

} // end namespace

#endif
