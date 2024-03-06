#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelJointControler : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      std::string jointName;

      if (_sdf->HasElement("joint_kp"))
          this->joint_kp = _sdf->Get<double>("joint_kp");
      if (_sdf->HasElement("joint_ki"))
          this->joint_ki = _sdf->Get<double>("joint_ki");
      if (_sdf->HasElement("joint_kd"))
          this->joint_kd = _sdf->Get<double>("joint_kd");
      if (_sdf->HasElement("topicName"))
          this->topicName = _sdf->Get<std::string>("topicName");
      if (_sdf->HasElement("joint_name"))
          jointName = _sdf->Get<std::string>("joint_name");
      if (_sdf->HasElement("namespace_model"))
          this->namespace_model = _sdf->Get<std::string>("namespace_model");
      if (_sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");

      //std::string jointName = this->joint_name;

      // Create a topic name
      std::string model_name = this->model->GetName().c_str();
      std::string joint_velocity = "/" + model_name + this->topicName + "/cmd_joint_velocity";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "set_jointVelocity_rosnode",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      //this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));
      this->rosNode = new ros::NodeHandle(this->namespace_);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelJointControler::OnUpdate, this));

      this->old_secs =ros::Time::now().toSec();

      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
	        const auto &jointController = this->model->GetJointController();
          jointController->Reset();
          jointController->AddJoint(model->GetJoint(jointName.c_str()));

          this->joint_name = model->GetJoint(jointName)->GetScopedName();
          ROS_WARN("Joint Name is ... %s", this->joint_name.c_str());

          jointController->SetVelocityPID(this->joint_name, common::PID(this->joint_kp, this->joint_ki, this->joint_kd));
          jointController->SetVelocityTarget(this->joint_name, 0.0);
      }

      // Freq
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            joint_velocity,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_joint_velocity, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));

      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;

      double max_delta = 0.0;

      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }

      if (delta > max_delta && delta != 0.0)
      {
        this->old_secs = new_secs;

	if(this->activate_pid_control)
        {
          ROS_DEBUG("Update Joint Velocity PID...");
	        const auto &jointController = this->model->GetJointController();
          jointController->SetVelocityTarget(this->joint_name, this->joint_velocity_magn);
        }
        else
        {
          // Apply a small linear velocity to the model.
          ROS_DEBUG("Update Joint Velocity BASIC...");
          this->model->GetJoint(this->joint_name)->SetVelocity(0, this->joint_velocity_magn);
        }

      }

    }


    public: void SetJointVelocity(const double &_freq)
    {
      this->joint_velocity_magn = _freq;
      //ROS_WARN("joint_velocity_magn >> %f", this->joint_velocity_magn);
    }


    public: void OnRosMsg_joint_velocity(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetJointVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Time Memory
    double old_secs;

    // Frequency of earthquake
    double freq_update = 100.0;

    double joint_velocity_magn = 0.0;


    /// \brief A node use for ROS transport
    //private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::NodeHandle* rosNode;

    private: std::string namespace_;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    std::string joint_name = "";
    std::string topicName = "";
    std::string namespace_model = "";
    bool activate_pid_control;

    double joint_kp = 0.1;
    double joint_ki = 0.0;
    double joint_kd = 0.0;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}
