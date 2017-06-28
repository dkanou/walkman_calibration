
/**
* @file: calibrate_urdf.cpp
* @author: Vignesh Sushrutha Raghavan
* @date: June 2017
* @brief: Full descriptions of functions needed to read and modify urdf for calibration.
*/
#include "walkman_urdf_calibration/calibrate_urdf.h"

Eigen::Matrix4f tfToHomogenousMatrix(tf::StampedTransform stamped_tf)
{
  Eigen::Matrix4f transform_matrix;
  transform_matrix.setZero();
  Eigen::Matrix3d temp_rotation;
  tf::matrixTFToEigen(stamped_tf.getBasis(),temp_rotation);
  transform_matrix.block<3,3>(0,0)= temp_rotation.cast<float>();
  transform_matrix(0,3)=stamped_tf.getOrigin().x();
  transform_matrix(1,3)=stamped_tf.getOrigin().y();
  transform_matrix(2,3)=stamped_tf.getOrigin().z();
  transform_matrix(3,3)=1.0;
  return transform_matrix;
}
UrdfCalibrator::UrdfCalibrator(ros::NodeHandle nh)
{
  nh.getParam("/urdf_filename", urdf_file_name_); //@TODO: Add exception handling
  nh.getParam("/file_save_name", urdf_file_save_name_);
  nh.getParam("/joint_names", joint_names_);
  nh.getParam("/waist_in_optitrack",waist_opti_frame_str_);
  nh.getParam("/hand_in_optitrack", hand_opti_frame_str_);
  if (!robot_urdf_model_.initFile(urdf_file_name_))
  {
    ROS_ERROR("Failed to parse urdf file");
    return;
  }
  urdf_xml_doc_.LoadFile(urdf_file_name_.c_str());
  //robot_element_=urdf_xml_doc_.FirstChildElement("robot");
  JointToCalibrate joint_it;
  int i=0;
  free_params_init_=new double[joint_names_.size()*6];
  for(std::vector<std::string>::iterator it=joint_names_.begin(); it!=joint_names_.end(); ++it)
  {
    joint_it.joint_name_=*it;
    joint_it.parent_name_=robot_urdf_model_.getJoint(joint_it.joint_name_)->parent_link_name;
    joint_it.child_name_=robot_urdf_model_.getJoint(joint_it.joint_name_)->child_link_name;
    joint_it.joint_order_index_=i;
    getJointInitFromUrdf(joint_it);
    joints_to_calibrate_.push_back(joint_it);
    ROS_INFO(" \nGot data of %s joint", joint_it.joint_name_.c_str());
    Eigen::Vector3f euler_angles=joint_it.parent_child_transform_.block<3,3>(0,0).eulerAngles(0,1,2);
    free_params_init_[(i+1)*6-6]=double(joint_it.parent_child_transform_(0,3));
    free_params_init_[(i+1)*6-5]=double(joint_it.parent_child_transform_(1,3));
    free_params_init_[(i+1)*6-4]=double(joint_it.parent_child_transform_(2,3));
    free_params_init_[(i+1)*6-3]=double(euler_angles[0]);
    free_params_init_[(i+1)*6-2]=double(euler_angles[1]);
    free_params_init_[(i+1)*6-1]=double(euler_angles[2]);
    std::cout<<"\n"<<free_params_init_[i];
    i++;
  }

  ceres_problem_ = new ceres::Problem();
  optimization_service_server_= nh.advertiseService("/CalibrateUrdf_data", &UrdfCalibrator::calibrateUrdfSrvServer, this) ;
  spin();
}

UrdfCalibrator::~UrdfCalibrator()
{
  urdf_xml_doc_.SaveFile(urdf_file_save_name_);
  if(ceres_problem_!=NULL)
  delete ceres_problem_;
  if(free_params_init_!=NULL)
  delete[] free_params_init_;

}

bool UrdfCalibrator::calibrateUrdfSrvServer(walkman_urdf_calibration::OptimizeUrdfJoints::Request &req,walkman_urdf_calibration::OptimizeUrdfJoints::Response &res )
{
  if(req.optimization_function == req.ADD_OPTIMIZATION_DATA)
  {
    float* joint_change_vector;
    joint_change_vector=new float[joint_names_.size()*6];
    tf::StampedTransform joint_transform;
    Eigen::Matrix4f transform_matrix, change_in_transform_matrix,test_matrix;
    transform_matrix.setZero();
    change_in_transform_matrix.setZero();
    Eigen::Matrix3d temp_rotation;
    temp_rotation.setZero();
    int i=0;
    test_matrix=Eigen::Matrix4f::Identity();
    for(std::vector<JointToCalibrate>::iterator it=joints_to_calibrate_.begin();it!=joints_to_calibrate_.end();++it)
    {
      transform_listener_.waitForTransform(it->parent_name_,it->child_name_,ros::Time(0), ros::Duration(6.0));
      transform_listener_.lookupTransform(it->parent_name_,it->child_name_,ros::Time(0),joint_transform );
      //tf::matrixTFToEigen(joint_transform.getBasis(),temp_rotation);
      transform_matrix=tfToHomogenousMatrix(joint_transform);
      change_in_transform_matrix=transform_matrix*it->parent_child_transform_.inverse();
      test_matrix=(change_in_transform_matrix*it->parent_child_transform_)*test_matrix;
      Eigen::Vector3f euler_angles=change_in_transform_matrix.block<3,3>(0,0).eulerAngles(0,1,2);\
      std::cout<<"\n the goddamned change\n"<<change_in_transform_matrix<<"\n";
      joint_change_vector[(i+1)*6-6]=change_in_transform_matrix(0,3);
      joint_change_vector[(i+1)*6-5]=change_in_transform_matrix(1,3);
      joint_change_vector[(i+1)*6-4]=change_in_transform_matrix(2,3);
      joint_change_vector[(i+1)*6-3]=euler_angles[0];
      joint_change_vector[(i+1)*6-2]=euler_angles[1];
      joint_change_vector[(i+1)*6-1]=euler_angles[2];
      i++;
    }

    transform_listener_.waitForTransform(waist_opti_frame_str_,hand_opti_frame_str_,ros::Time(0), ros::Duration(6.0));
    transform_listener_.lookupTransform(waist_opti_frame_str_,hand_opti_frame_str_,ros::Time(0),joint_transform );
    transform_matrix=tfToHomogenousMatrix(joint_transform);
    std::cout<<"\nestimated pose \n"<<test_matrix<<"\n actual pose\n"<<transform_matrix;
    //cost = MinimizeUrdfError::Create(transform_matrix,joint_change_vector,joints_to_calibrate_.size());
    /*double* residuals=new double[6];
    double ** params = new double*[1];
    params[0]=free_params_init_;

    cost->Evaluate(params, residuals, NULL);*/

    ceres_problem_->AddResidualBlock(
      new ceres::NumericDiffCostFunction< MinimizeUrdfError,ceres::CENTRAL,6,60>(new MinimizeUrdfError(transform_matrix,joint_change_vector,joints_to_calibrate_.size())),NULL,free_params_init_);
    //delete[] residuals;
    ROS_INFO("\nAdded data");
  }
  else
  {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ROS_INFO("Starting optimization");
    ceres::Solve(options, ceres_problem_, &summary);
    std::cout<<"\n"<<summary.FullReport();
    writeJointsToUrdf(free_params_init_);
    res.success=true;
  }
  return true;

}


void UrdfCalibrator::getJointInitFromUrdf(JointToCalibrate &joint)
{
  joint.parent_child_transform_.setZero();
  joint.parent_child_transform_(0,3)=double(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.x);
  joint.parent_child_transform_(1,3)=double(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.y);
  joint.parent_child_transform_(2,3)=double(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.position.z);
  joint.parent_child_transform_(3,3)=1.0;
  float w,x,y,z;
  w=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.w);
  x=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.x);
  y=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.y);
  z=float(robot_urdf_model_.getJoint(joint.joint_name_)->parent_to_joint_origin_transform.rotation.z);
  Eigen::Quaternionf temp_quat(w,x,y,z);
  joint.parent_child_transform_.block<3,3>(0,0)=temp_quat.toRotationMatrix();

}

void UrdfCalibrator::writeJointsToUrdf(double* optimized_joint_values)
{
  TiXmlElement* robot_element_=urdf_xml_doc_.FirstChildElement("robot");
  int i;
  for(TiXmlElement* joint_xml=robot_element_->FirstChildElement("joint");joint_xml;joint_xml=joint_xml->NextSiblingElement("joint"))
  {
    std::vector<JointToCalibrate>::iterator it =
    std::find_if(joints_to_calibrate_.begin(),joints_to_calibrate_.end(),
    boost::bind(&JointToCalibrate::joint_name_, _1)==std::string(joint_xml->Attribute("name")));
    if(it!=joints_to_calibrate_.end())
    {
      i=it->joint_order_index_;
      std::stringstream ss_origin,ss_rpy;
      ss_origin<<optimized_joint_values[(i+1)*6-6]<<" "<<optimized_joint_values[(i+1)*6-5]<<" "<<optimized_joint_values[(i+1)*6-4];
      ss_rpy<<optimized_joint_values[(i+1)*6-3]<<" "<<optimized_joint_values[(i+1)*6-2]<<" "<<optimized_joint_values[(i+1)*6-1];
      joint_xml->FirstChildElement("origin")->SetAttribute("xyz",ss_origin.str());
      joint_xml->FirstChildElement("origin")->SetAttribute("rpy",ss_rpy.str());
    }
  }

}

void UrdfCalibrator::spin()
{
  ros::Rate spin_rate(1000);
  int i=0;
  while(ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_calibrator");
  ros::NodeHandle nh;
  UrdfCalibrator test1(nh);
  return 0;
}
