#ifndef OPENHRP_PLUGINS_SCHEDULER_HH
# define OPENHRP_PLUGINS_SCHEDULER_HH
# include <boost/array.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include <ros/ros.h>
# include <ros/time.h>
# include <dynamic_reconfigure/server.h>
# include <tf/transform_broadcaster.h>
# include <std_srvs/Empty.h>

# include <openhrp_msgs/SpawnModel.h>
# include <openhrp_msgs/StartSimulation.h>

# include <openhrp_plugins/SimulationConfig.h>

# include <OpenHRP-3.1/hrpCorba/Controller.hh>
# include <OpenHRP-3.1/hrpCorba/DynamicsSimulator.hh>
# include <OpenHRP-3.1/hrpCorba/ModelLoader.hh>
# include <OpenHRP-3.1/hrpCorba/OnlineViewer.hh>

class SchedulerNode
{
public:
  typedef dynamic_reconfigure::Server<openhrp_plugins::SimulationConfig>
  reconfigureSrv_t;

  explicit SchedulerNode (int argc, char* argv[],
			  ros::NodeHandle& nh,
			  ros::NodeHandle& privateNh);
  ~SchedulerNode ();

  void spin ();

private:
  void
  reconfigureCallback(openhrp_plugins::SimulationConfig& config,
		      uint32_t level);

  bool spawnVrmlModelCallback
  (openhrp_msgs::SpawnModel::Request&,
   openhrp_msgs::SpawnModel::Response&);

  bool startSimulationCallback
  (openhrp_msgs::StartSimulation::Request&,
   openhrp_msgs::StartSimulation::Response&);

  bool pauseSimulationCallback
  (std_srvs::Empty::Request&,
   std_srvs::Empty::Response&);

  bool unpauseSimulationCallback
  (std_srvs::Empty::Request&,
   std_srvs::Empty::Response&);

  void publishModelsData ();

  enum State
    {
      STATE_IDLE,
      STATE_STARTED,
      STATE_PAUSED,
      STATE_MAX
    };

  struct ModelInfo
  {
    openhrp_msgs::SpawnModel::Request meta;
    OpenHRP::BodyInfo_var bodyInfo;

    ros::Publisher jointState;
    ros::Publisher odometry;
  };

  // Simulation state.
  State state_;

  // Node handles.
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  // Tf transform broadcaster.
  tf::TransformBroadcaster transformBroadcaster_;

  // Dynamic reconfiguration.
  boost::recursive_mutex mutex_;
  reconfigureSrv_t reconfigureSrv_;

  // Publishers
  ros::Publisher clock_;

  // Services
  ros::ServiceServer spawnVrmlModel_;
  ros::ServiceServer startSimulation_;

  ros::ServiceServer pauseSimulation_;
  ros::ServiceServer unpauseSimulation_;

  // CORBA
  CORBA::ORB_var orb_;
  CosNaming::NamingContext_var cxt_;
  OpenHRP::OnlineViewer_var onlineViewer_;
  OpenHRP::DynamicsSimulator_var dynamicsSimulator_;
  std::vector<OpenHRP::Controller_var> controllers_;
  std::vector<ModelInfo> models_;
  OpenHRP::WorldState_var worldState_;

  // Dynamic reconfigure.
  boost::array<double, 3> gravity_;

  // Timestep.
  double timeStep_;

  // Integration method.
  OpenHRP::DynamicsSimulator::IntegrateMethod integrationMethod_;
  // Enable sensor simulation?
  OpenHRP::DynamicsSimulator::SensorOption enableSensor_;

  // Current time.
  ros::Time time_;
};

#endif //! OPENHRP_PLUGINS_SCHEDULER_HH
