#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"


namespace gazebo
{
  class VelControl : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      // Check if a ros node for gazebo has been initialized
			if (!ros::isInitialized())
			{
			  ROS_WARN("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  ROS_WARN("Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
			  ROS_WARN("If using gazebo as a stand-alone, package, run roscore first");
			  ROS_WARN("Trying to initialize ROS node - 'gazebo_client'\n");

			  // Try creating a new ROS node if one is not created already
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "gazebo_client",
					  ros::init_options::NoSigintHandler);
		  	}
		  	// Initialize the ROS nodehandle and assign it to the class member
		  	this->rosnode = new ros::NodeHandle();

        // Store the pointer to the model
        this->model = _parent;



        // Subscribing and Publishing
		    this->vel_sub = this->rosnode->subscribe("/car/cmd_vel",10,&VelControl::vel_callback,this);
        this->pos_pub = this->rosnode->advertise<geometry_msgs::Pose>("/car/pose",10);


        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VelControl::OnUpdate, this));

            ROS_INFO("Plugin load successful!\n");
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get the world pose
      ignition::math::Pose3<double> pose = this->model->WorldPose();

      // Set the message values
      pos_msg.position.x = pose.Pos()[0];
      pos_msg.position.y = pose.Pos()[1];
      pos_msg.position.z = pose.Pos()[2];

      pos_msg.orientation.w = pose.Rot().W();
      pos_msg.orientation.x = pose.Rot().X();
      pos_msg.orientation.y = pose.Rot().Y();
      pos_msg.orientation.z = pose.Rot().Z();

      // Apply the velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(linear_vel_x, linear_vel_y, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, angular_vel_z));
      // publish the position
      pos_pub.publish(pos_msg);
    }


    public: void vel_callback(const geometry_msgs::Twist& msg)
	  {
        // Quaternion variable
        ignition::math::Quaternion<double> q;
        // Get the world pose
        ignition::math::Pose3<double> pose = this->model->WorldPose();

        // Get the rotation Quaternion
        q = pose.Rot();

        // Get the Yaw value
        double yaw = q.Yaw();

        double vel = -msg.linear.x;

		  	this->linear_vel_x = vel*std::cos(yaw);
        this->linear_vel_y = vel*std::sin(yaw);
        this->angular_vel_z = msg.angular.z;
	  }



    	  //////////////////// Initialize Global Members////////////////////

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS node handle member
    public: ros::NodeHandle* rosnode;

    //global member for the velocity message
    public: double linear_vel_x;
    public: double linear_vel_y;
    public: double angular_vel_z;

    // subsciber for the velocity message
    public: ros::Subscriber vel_sub;

    // publisher for position
    public: ros::Publisher pos_pub;

    // velocity message
    public: geometry_msgs::Pose pos_msg;

  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(VelControl)
}
