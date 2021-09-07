#include <cstdlib>
#include <iostream>
#include <thread>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <ros/package.h>


using namespace std::chrono_literals;


#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
int main()
{
    // Init variables
    std::vector<std::string> joint_names_{"elfin_joint1","elfin_joint2","elfin_joint3",
            "elfin_joint4","elfin_joint5","elfin_joint6"};
    std::cout << "Joints: " << std::endl;
    for (std::string x : joint_names_)
            std::cout << x << std::endl;
    
    KDL::Tree 	kdl_tree_;
	KDL::Chain	kdl_chain_;
    unsigned int n_joints_ = joint_names_.size();
    urdf::Model urdf;
    KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
	KDL::JntArray q_, qdot_;

    // Notice that during the init, the q variables are created as KDL::JntArray
    // However, they do not contain any knowledge about the size of the system
    // By calling the variable.data, we can access the underlying Eigen Matrix
    // Eigen::VectorXd::Zero(n_size) creates an empty Eigen vector by the n_size
	q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
	qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
	qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
	q_.data = Eigen::VectorXd::Zero(n_joints_);
	qdot_.data = Eigen::VectorXd::Zero(n_joints_);

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics
    KDL::Jacobian J_;
    KDL::JntArray G_;									// gravity torque vector
    KDL::Vector gravity_;  

    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_; // Frame positions of each link with respect to the world
    KDL::Twist ex_temp_;
    std::string root_name = "world";
    std::string tip_name = "elfin_link6";



    ROS_INFO("Starting Example");
    // URDF Loading
    ROS_INFO("Loading URDF");
    std::string path = ros::package::getPath("kdl_eigen_example");
    if (!urdf.initFile(path+"/elfin.urdf"))
	{
		ROS_ERROR("Failed to parse URDF file");
        return false;
	}else{
        ROS_INFO("URDF Loaded");
    }

    // kdl parser
	if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}else{
        ROS_INFO("KDL Tree constructed from URDF");
    }

    // Get the chain from the world frame to end link
	if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
	{
		ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		ROS_ERROR_STREAM("  The segments are:");
		KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    	KDL::SegmentMap::iterator it;
    	for( it=segment_map.begin(); it != segment_map.end(); it++ )
      		ROS_ERROR_STREAM( "    "<<(*it).first);
    	return false;
	}

    // Define gravity vector (Z down)
    gravity_ = KDL::Vector::Zero();
	gravity_(2) = -9.81;
    G_.resize(n_joints_);	

	// Init inverse dynamics solver
	id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

    // jacobian solver 
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // forward kinematics solver 
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // Create an example joint angles q, and pass it to the forward kinematics solver
    // Lets consider the joints are all at 0 degrees at the moment
    q_(0) = 0;
    q_(1) = 0;
    q_(2) = 0;
    q_(3) = 0;
    q_(4) = 0;
    q_(5) = 0;
    std::cout << std::fixed << std::setprecision(4); // For clean printout
    std::cout << "-----------------------------------------------" << std::endl;
    // We can get the state of the end link using the forward kinematics solver
    // q_ is the KDL::JntArray, which contains the joint state, angle if revolute, position if prismatic
    // x_ is the KDL::Frame, which includes the rotation and position values of the end effector
    // http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Frame.html
    // x_.p is the position with type vector, the x y z coordinates either can be retrieved as x_.p.x(),x_.p.y(),x_.p.z()
    // or as x_.p(0), x_.p(1), x_.p(2)
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position: " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    // X pos: -0.0000 |Y pos: 0.0000 |Z pos: 0.8890
    // Angles (degrees): Roll: -90.0000 |Pitch: -0.0000 |Yaw: -90.0000

    // Rotation can also be extracted from the KDL::Frame, either as euler or quaternion
    // KDL::Frame.M is the KDL::Rotation object
    // http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Rotation.html
    double roll, pitch, yaw;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;

    std::cout << "-----------------------------------------------" << std::endl;

    // Lets apply some rotation to the 5th joint(elfin_joint5), and see the effect at the end effector position
    q_(4) = 90 * D2R; // Radians! Multiplication of degree with D2R results in radians. Notice the index starts from 0
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position for 90 degree rotation to elfin_joint5: " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;
    // X pos: 0.1065 |Y pos: -0.0000 |Z pos: 0.7825
    // Angles (degrees): Roll: -180.0000 |Pitch: -0.0000 |Yaw: -90.0000
    // The 5th joint, is the joint that moves the last part of the robot.
    // The 6th joint is just rotating the end effector orientation, therefore it wont result in the translation difference
    // If you check the URDF, you can find that the link length between link5 and link6 is 0.1065m.
    // As expected, when we rotate the 5th joint 90 degrees, the last part of the robot is rotated parallel to the ground
    // Resulting the end effector to move 0.1065 in the x axis and -0.1065 in the z axis

    // Looks like this
    // ― ―
    // |
    // |
    // |
    // |
    std::cout << "-----------------------------------------------" << std::endl;

    // Similarly, if we rotate it the other way around
    q_(4) = -90 * D2R; 
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position for -90 degree rotation to elfin_joint5: " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;
    // X pos: -0.1065 |Y pos: 0.0000 |Z pos: 0.7825
    // Angles (degrees): Roll: -0.0000 |Pitch: -0.0000 |Yaw: -90.0000
    // We can see that now the end effector is moved to the other way in the x axis

    // Looks like this
    // ― ―
    //   |
    //   |
    //   |
    //   |
    std::cout << "-----------------------------------------------" << std::endl;

    // Lets rotate the elfin_joint3
    q_(2) = 90 * D2R; 
    q_(4) = 0; 
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position for 90 degree rotation to elfin_joint3: " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;
    // X pos: 0.4305 |Y pos: -0.0000 |Z pos: 0.4585
    // Angles (degrees): Roll: -180.0000 |Pitch: -0.0000 |Yaw: -90.0000

    // Similar to previous case, the robot moved into the X direction, and the end effector is closer to the ground
    // since the elfin_joint3 is placed lower. Also, since the motion in elfin_joint3 is earlier in the chain,
    // the translation in the end effector is greater compared to elfin_joint5. 

    // Looks like this
    // ― ― ― ―
    // |   
    // |   

    std::cout << "-----------------------------------------------" << std::endl;


    // With the same rotation in the elfin_joint3, we can rotate the elfin_joint5
    // Now, we are giving -90 degree to elfin_joint5, which will result the end effector to rotate to look up
    q_(2) = 90 * D2R; 
    q_(4) = -90 * D2R; 
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position for -90 degree elfin_joint5, 90 degree elfin_joint3 : " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;
    // X pos: 0.3240 |Y pos: -0.0000 |Z pos: 0.5650
    // Angles (degrees): Roll: -90.0000 |Pitch: -0.0000 |Yaw: -90.0000

    // We can see that now the end effector is moved higher in the z axis, 
    // and the x is reduced by the length between link5 and link6.

    // Looks like this
    //     |
    // ― ― |
    // |   
    // |   
    std::cout << "-----------------------------------------------" << std::endl;


    // Similarly, if we rotate it the other way around
    q_(2) = 90 * D2R; 
    q_(4) = 90 * D2R; 
    fk_pos_solver_->JntToCart(q_, x_);
    std::cout << "End effector link position for -90 degree rotation to elfin_joint5: " << std::endl;
    std::cout << "X pos: " << x_.p.x() << " |";
    std::cout << "Y pos: " << x_.p.y() << " |";
    std::cout << "Z pos: " << x_.p.z() << std::endl;
    x_.M.GetRPY(roll,pitch,yaw);
    std::cout << "Angles (degrees): Roll: " << roll*R2D << " |";
    std::cout << "Pitch: " << pitch*R2D << " |";
    std::cout << "Yaw: " << yaw*R2D << std::endl;
    // X pos: 0.3240 |Y pos: -0.0000 |Z pos: 0.3520
    // Angles (degrees): Roll: 90.0001 |Pitch: 0.0000 |Yaw: -90.0000

    // We can see that now the end effector is moved to look down.
    
    // Looks like this
    // ― ― |
    // |   |
    // |   
    std::cout << "------------------------------------------------" << std::endl;


  return 0;
}