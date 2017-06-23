
/**
* @file: calibrate_urdf.cpp
* @author: Vignesh Sushrutha Raghavan
* @date: June 2017
* @brief: Full descriptions of functions needed to read and modify urdf for calibration.
*/
#include "walkman_urdf_calibration/calibrate_urdf.h"

UrdfCalibrator::UrdfCalibrator(ros::NodeHandle nh)
{
  nh.getParam("/urdf_filename", urdf_file_name_); //@TODO: Add exception handling
  nh.getParam("joint_names", joint_names_);
  if (!robot_urdf_model_.initFile(urdf_file_name_))
  {
    ROS_ERROR("Failed to parse urdf file");
    return;
  }
  urdf_xml_doc_.Parse(urdf_file_name_.c_str());
  //robot_element_=urdf_xml_doc_.FirstChildElement("robot");
  JointToCalibrate joint_it;
  for(std::vector<std::string>::iterator it=joint_names_.begin(); it!=joint_names_.end(); ++it)
  {
    joint_it.joint_name_=*it;
    joint_it.parent_name_=robot_urdf_model_.getJoint(joint_it.joint_name_)->parent_link_name;
    joint_it.child_name_=robot_urdf_model_.getJoint(joint_it.joint_name_)->child_link_name;
    getJointInitFromUrdf(joint_it);
    ROS_INFO(" \nGot data of %s joint", joint_it.joint_name_.c_str());
  }

}

UrdfCalibrator::~UrdfCalibrator()
{

}

void UrdfCalibrator::getJointInitFromUrdf(JointToCalibrate &joint)
{
  joint.parent_child_transform_.setZero();
  joint.parent_child_transform_(0,3)=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.x);
  joint.parent_child_transform_(1,3)=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.y);
  joint.parent_child_transform_(2,3)=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.z);
  joint.parent_child_transform_(3,3)=1.0;
  float w,x,y,z;
  w=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.w);
  x=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.x);
  y=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.y);
  z=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.z);
  Eigen::Quaternionf temp_quat(w,x,y,z);
  joint.parent_child_transform_.block<3,3>(0,0)=temp_quat.toRotationMatrix();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_calibrator");
  ros::NodeHandle nh;
  UrdfCalibrator test1(nh);
  return 0;
}
