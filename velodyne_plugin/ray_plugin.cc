#include <gazebo/gazebo.hh>
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace gazebo{

    class RayPlugin : public SensorPlugin
    {

        public: RayPlugin(){
            printf("Constructor\n");
        }

        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
            printf("Load\n");

            //Get the parent sensor
            this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
        
            //Make sure the parent sensor is valid
            if(!this->parentSensor){
                gzerr << "RayPlugin requires a RaySensor\n";
                return;
            }

            //Connect to the sensor updat event
            this->newLaserScansConnection = this->parentSensor->ConnectUpdated(std::bind(&RayPlugin::OnNewLaserScans, this));

            //Make sure the parent sensor is active
            this->parentSensor->SetActive(true);

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(_parent->WorldName());

            // Create a topic name
           // std::string topicName = "~/" + _parent->GetName() + "/range";
            std::string topicName = "/velodyne/range";

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                        ros::init_options::NoSigintHandler);
            }

            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            ros::NodeHandle n;
            this->rosPub = n.advertise<std_msgs::Float64>(topicName, 1);
   

        }

        public: void OnNewLaserScans(){

            //Range 0, so we get the range for the first ray (as you can have multiple rays)
            double range = this->parentSensor->Range(0);

            std::cout << "Range: " << range << std::endl;

            std_msgs::Float64 newRange;

            newRange.data = range;

            rosPub.publish(newRange);
        }


        protected: physics::WorldPtr world;

        private: sensors::RaySensorPtr parentSensor;

        private: event::ConnectionPtr newLaserScansConnection;

        private: transport::NodePtr node;

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::Publisher rosPub;



    };

    GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

}