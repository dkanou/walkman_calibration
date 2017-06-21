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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

struct joint_to_calibrate{
  std::string joint_name_;
  std::string parent_name_;
  std::string child_name_;
  Eigen::Matrix4f parent_child_transform_;
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
  void getJointInitFromUrdf(joint_to_calibrate &joint);

  /**
    * @brief: function to wrtie all data of the calibrated joint to urdf
    * @param: joint: data of the joint which is written to urdf.
    */
  void writeJointToUrdf(joint_to_calibrate calibrated_joint);

  std::string urdf_file_name_; /**<urdf file name */
  std::vector<std::string> joint_names_; /**< vector of names of joints to be calibrated */
  std::vector<joint_to_calibrate> joints_to_calibrate; /**< vector of joints to be calibrated */
  urdf::Model robot_urdf_model_; /**< variable to store the urdf model */
  TiXmlDocument urdf_xml_doc_; /**< object to xml doc, to be used for modifcation */
  //TiXmlElement robot_element_; /**< object to element in xml doc */

};

#endif
