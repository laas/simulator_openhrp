#ifndef OPENHRP_PLUGINS_SCHEDULER_HH
# define OPENHRP_PLUGINS_SCHEDULER_HH
# include <boost/array.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include <ros/ros.h>
# include <ros/time.h>
# include <dynamic_reconfigure/server.h>

# include <openhrp_msgs/SpawnModel.h>

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
  void spin ();

private:
  void
  reconfigureCallback(openhrp_plugins::SimulationConfig& config,
		      uint32_t level);
  void reconfigure ();

  bool spawnVrmlModelCallback
  (openhrp_msgs::SpawnModel::Request&,
   openhrp_msgs::SpawnModel::Response&);

  // Node handles.
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  // Dynamic reconfiguration.
  boost::recursive_mutex mutex_;
  reconfigureSrv_t reconfigureSrv_;

  // Publishers
  ros::Publisher clock_;

  // Services
  ros::ServiceServer spawnVrmlModel_;
  ros::ServiceServer deleteModel_;

  // CORBA
  CORBA::ORB_var orb_;
  OpenHRP::OnlineViewer_var onlineViewer_;
  OpenHRP::DynamicsSimulator_var dynamicsSimulator_;
  std::vector<OpenHRP::Controller_var> controllers_;
  std::vector<OpenHRP::BodyInfo_var> models_;

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
