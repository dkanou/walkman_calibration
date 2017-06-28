/**
* @file: calibrate_urdfh
* @author: Vignesh Sushrutha Raghavan
* @date: June 2017
* @brief: Header file to describe the class and functions needed for calibrating walkman urdf
*/
#ifndef CALIBRATE_URDF_H
#define CALIBRATE_URDF_H

//Header includes

#include "ros/ros.h"
#include <Eigen/Dense>
#include <urdf/model.h>
#include <iostream>
#include <tinyxml.h>
#include <boost/bind.hpp>
#include <ceres/ceres.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "walkman_urdf_calibration/OptimizeUrdfJoints.h"

int test_one_;

struct JointToCalibrate{
  std::string joint_name_;
  std::string parent_name_;
  std::string child_name_;
  Eigen::Matrix4f parent_child_transform_;
  int joint_order_index_;
};

struct MinimizeUrdfError{
  MinimizeUrdfError(Eigen::Matrix4f waist_hand_opti_track,float* joint_changes, int number_of_joints):waist_hand_opti_track_(waist_hand_opti_track)
  {
    joint_changes_ = joint_changes;
    number_of_joints_=number_of_joints;
    test_one_=1;
  }
  virtual ~MinimizeUrdfError()
  {
    if(joint_changes_!=NULL)
    delete[] joint_changes_;
  }
  Eigen::Matrix3f eulerToRot( float roll,float pitch,float yaw ) const //Obtained from Urdf::model::joint::pose
  {
    Eigen::Matrix3f rot;
    rot= Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(pitch,  Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  return rot;
  }
  Eigen::Matrix4f vectorToTransformMat(float px,float py, float pz, float roll, float pitch, float yaw) const
  {
    Eigen::Matrix4f transform_matrix;
    transform_matrix.setZero();
    transform_matrix.block<3,3>(0,0) = eulerToRot(roll,pitch,yaw);
    transform_matrix(0,3) = px;
    transform_matrix(1,3) = py;
    transform_matrix(2,3) = pz;
    transform_matrix(3,3) = 1.0;
    return transform_matrix;
  }
  bool operator()(const double * const  joint_init_param, double* residual) const
  {
    Eigen::Matrix4f new_calculated_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f joint_init_matrix,joint_change_matrix;
    for(int i=0;i<number_of_joints_;i++)
    {
      joint_init_matrix=vectorToTransformMat(float(joint_init_param[(i+1)*6-6]),float(joint_init_param[(i+1)*6-5]),float(joint_init_param[(i+1)*6-4]),float(joint_init_param[(i+1)*6-3]),float(joint_init_param[(i+1)*6-2]),float(joint_init_param[(i+1)*6-1]));
      joint_change_matrix=vectorToTransformMat(joint_changes_[(i+1)*6-6],joint_changes_[(i+1)*6-5],joint_changes_[(i+1)*6-4],joint_changes_[(i+1)*6-3],joint_changes_[(i+1)*6-2],joint_changes_[(i+1)*6-1]);
      new_calculated_pose=(joint_change_matrix*joint_init_matrix)*new_calculated_pose;
    }
    if(test_one_==1)
    {
      std::cout<<"\n Blah blah "<<number_of_joints_;
      for(int k=0;k<60;k++)
      std::cout<<"\n"<<joint_changes_[k];
      test_one_++;
      std::cout<<"\nEstimated \n"<<new_calculated_pose<<"\n actual\n"<<waist_hand_opti_track_;
    }
    Eigen::Vector3f new_euler_angles=new_calculated_pose.block<3,3>(0,0).eulerAngles(0,1,2);
    Eigen::Vector3f observed_euler_angles=waist_hand_opti_track_.block<3,3>(0,0).eulerAngles(0,1,2);
    residual[0]=double(waist_hand_opti_track_(0,3)-new_calculated_pose(0,3));
    residual[1]=double(waist_hand_opti_track_(1,3)-new_calculated_pose(1,3));
    residual[2]=double(waist_hand_opti_track_(2,3)-new_calculated_pose(2,3));
    residual[3]=double(observed_euler_angles[0]-new_euler_angles[0]);
    residual[4]=double(observed_euler_angles[1]-new_euler_angles[1]);
    residual[5]=double(observed_euler_angles[2]-new_euler_angles[2]);

    return true;
  }

  /*static ceres::CostFunction* Create(Eigen::Matrix4f waist_hand_opti_track,float* joint_changes, int number_of_joints)
  {
  return( new ceres::AutoDiffCostFunction<MinimizeUrdfError,6,number_of_joints*6>(new MinimizeUrdfError(waist_hand_opti_track,joint_changes, number_of_joints)));


}*/
Eigen::Matrix4f waist_hand_opti_track_;
float* joint_changes_;
int number_of_joints_;
};
class UrdfCalibrator
{
public:

  UrdfCalibrator(ros::NodeHandle nh);
  ~UrdfCalibrator();
  /**
  * @brief: function to load all data of the joint to be calibrated in to the struct
  * @param: joint: joint to which data is loaded
  */
  void getJointInitFromUrdf(JointToCalibrate &joint);

  /**
  * @brief: function to wrtie all data of the calibrated joint to urdf
  * @param: joint: data of the joint which is written to urdf.
  */
  void writeJointsToUrdf(double* optimized_joint_values);

  /**
  * @brief: service server which will receive command to get data from tf and opti track and minimise overall error
  * @param: req request for service. if 1 then data is added to optimization block. if 2 optimizer solves the problem.
  * @return: always returns true.
  */
  bool calibrateUrdfSrvServer(walkman_urdf_calibration::OptimizeUrdfJoints::Request &req,walkman_urdf_calibration::OptimizeUrdfJoints::Response &res );

  /**
  * @brief: spinner for the class/Node
  */
  void spin();
  std::string urdf_file_name_,urdf_file_save_name_; /**<urdf file name */
  std::vector<std::string> joint_names_; /**< vector of names of joints to be calibrated */
  std::vector<JointToCalibrate> joints_to_calibrate_; /**< vector of joints to be calibrated */
  urdf::Model robot_urdf_model_; /**< variable to store the urdf model */
  TiXmlDocument urdf_xml_doc_; /**< object to xml doc, to be used for modifcation */
  //TiXmlElement robot_element_; /**< object to element in xml doc */
  ceres::Problem* ceres_problem_; /**< pointer to ceres problem **/
  ros::ServiceServer optimization_service_server_; /**< service server for recording and optimizing data at will */
  tf::TransformListener transform_listener_; /**< listens to all joint transforms */
  std::string waist_opti_frame_str_, hand_opti_frame_str_; /**<string frames for the bodies tracked by opti-track */
  double* free_params_init_; /**<pointer to the free params for optimizer */
  ceres::CostFunction * cost;
};

#endif
