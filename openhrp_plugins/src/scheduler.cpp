#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/scope_exit.hpp>

#include <LinearMath/btMatrix3x3.h>

#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <OpenHRP-3.1/hrpModel/ModelLoaderUtil.h>
#include <OpenHRP-3.1/hrpUtil/OnlineViewerUtil.h>

#include "scheduler.hh"

static const char* rootBodyName = "WAIST";

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
  CosNaming::Name ncName;
  ncName.length (1);
  ncName[0].id = CORBA::string_dup (n.c_str ());
  ncName[0].kind = CORBA::string_dup ("");
  X_ptr srv = NULL;
  try
    {
      srv = X::_narrow (cxt->resolve (ncName));
    }
  catch (const CosNaming::NamingContext::NotFound &exc)
    {
      ROS_ERROR_STREAM (n << " not found: ");
      switch (exc.why)
	{
	case CosNaming::NamingContext::missing_node:
	  ROS_ERROR ("Missing Node");
	  break;
	case CosNaming::NamingContext::not_context:
	  ROS_ERROR ("Not Context");
	  break;
	case CosNaming::NamingContext::not_object:
	  ROS_ERROR ("Not Object");
	  break;
	}
      return 0;
    }
  catch (CosNaming::NamingContext::CannotProceed &exc)
    {
      ROS_ERROR_STREAM ("Resolve " << n << " CannotProceed");
    }
  catch(CosNaming::NamingContext::AlreadyBound &exc)
    {
      ROS_ERROR_STREAM ("Resolve " << n << " InvalidName");
    }
  return srv;
}

SchedulerNode::SchedulerNode (int argc, char* argv[],
			      ros::NodeHandle& nh,
			      ros::NodeHandle& privateNh)
  : state_ (STATE_IDLE),
    nodeHandle_ (nh),
    nodeHandlePrivate_ (privateNh),
    transformBroadcaster_ (),
    mutex_ (),
    reconfigureSrv_ (mutex_, nodeHandlePrivate_),
    clock_ (),
    spawnVrmlModel_ (),
    startSimulation_ (),
    pauseSimulation_ (),
    unpauseSimulation_ (),
    orb_ (),
    cxt_ (),
    onlineViewer_ (),
    dynamicsSimulator_ (),
    controllers_ (),
    models_ (),
    worldState_ (),
    gravity_ (),
    timeStep_ (),
    integrationMethod_ (),
    enableSensor_ (),
    time_ (),
    worldFrame_ ("openhrp")
{
  // Dynamic reconfigure.
  reconfigureSrv_t::CallbackType f =
    boost::bind(&SchedulerNode::reconfigureCallback, this, _1, _2);
  reconfigureSrv_.setCallback(f);

  // Fake arguments for CORBA initialization.
  int corbaArgc = 1;
  char* corbaArgv[] = {argv[0]};

  // Initialize CORBA.
  orb_ = CORBA::ORB_init (corbaArgc, corbaArgv);
  CORBA::Object_var poaObj = orb_->resolve_initial_references("RootPOA");
  PortableServer::POA_var rootPOA = PortableServer::POA::_narrow (poaObj);
  PortableServer::POAManager_var manager = rootPOA->the_POAManager ();

  CORBA::Object_var nS = orb_->resolve_initial_references ("NameService");
  cxt_ = CosNaming::NamingContext::_narrow (nS);

  // Initialize publishers.
  clock_ =
    nodeHandle_.advertise<rosgraph_msgs::Clock>
    ("/clock", 1);

  // Initialize services.
  //  Spawn model.
  typedef boost::function<
  bool (openhrp_msgs::SpawnModel::Request&,
	openhrp_msgs::SpawnModel::Response&)>
  spawnModelCallback_t;
  spawnModelCallback_t spawnModelCb =
    boost::bind (&SchedulerNode::spawnVrmlModelCallback,
		 this, _1, _2);
  spawnVrmlModel_ =
    nodeHandle_.advertiseService
    ("spawn_vrml_model", spawnModelCb);

  //  Start simulation.
  typedef boost::function<
  bool (openhrp_msgs::StartSimulation::Request&,
	openhrp_msgs::StartSimulation::Response&)>
  startSimulationCallback_t;
  startSimulationCallback_t startSimulationCb =
    boost::bind (&SchedulerNode::startSimulationCallback,
		 this, _1, _2);
  startSimulation_ =
    nodeHandle_.advertiseService
    ("start_simulation", startSimulationCb);

  //  Pause simulation.
  typedef boost::function<
  bool (std_srvs::Empty::Request&,
	std_srvs::Empty::Response&)>
  pauseSimulationCallback_t;
  pauseSimulationCallback_t pauseSimulationCb =
    boost::bind (&SchedulerNode::pauseSimulationCallback,
		 this, _1, _2);
  pauseSimulation_ =
    nodeHandle_.advertiseService
    ("pause_simulation", pauseSimulationCb);

  //  Unpause simulation.
  typedef boost::function<
  bool (std_srvs::Empty::Request&,
	std_srvs::Empty::Response&)>
  unpauseSimulationCallback_t;
  unpauseSimulationCallback_t unpauseSimulationCb =
    boost::bind (&SchedulerNode::unpauseSimulationCallback,
		 this, _1, _2);
  unpauseSimulation_ =
    nodeHandle_.advertiseService
    ("unpause_simulation", unpauseSimulationCb);


  // Enable simulated time.
  nodeHandle_.setParam ("/use_sim_time", true);
}

SchedulerNode::~SchedulerNode ()
{
  if (!CORBA::is_nil (dynamicsSimulator_))
    dynamicsSimulator_->destroy ();
}


void
SchedulerNode::reconfigureCallback
(openhrp_plugins::SimulationConfig& config, uint32_t level)
{
  mutex_.lock ();
  gravity_[0] = config.gravity_x;
  gravity_[1] = config.gravity_y;
  gravity_[2] = config.gravity_z;
  timeStep_ = config.timestep;

  switch (config.integration_method)
    {
    case 0:
      integrationMethod_ = OpenHRP::DynamicsSimulator::EULER;
      break;
    case 1:
      integrationMethod_ = OpenHRP::DynamicsSimulator::RUNGE_KUTTA;
      break;
    default:
      assert (0 && "should never happen");
    }

  if (config.enable_sensor)
    enableSensor_ = OpenHRP::DynamicsSimulator::ENABLE_SENSOR;
  else
    enableSensor_ = OpenHRP::DynamicsSimulator::DISABLE_SENSOR;
  mutex_.unlock ();
}

bool
SchedulerNode::spawnVrmlModelCallback
(openhrp_msgs::SpawnModel::Request& req,
 openhrp_msgs::SpawnModel::Response& res)
{
  ROS_INFO ("spawning model");

  res.status_message = "";
  res.success = false;

  ModelInfo model;
  model.meta = req;

  boost::format jointStateTopic ("%1%/joint_state");
  jointStateTopic % model.meta.model_name;
  model.jointState =
    nodeHandle_.advertise<sensor_msgs::JointState>
    (jointStateTopic.str (), 1);

  boost::format odometryTopic ("%1%/odometry");
  odometryTopic % model.meta.model_name;
  model.odometry =
    nodeHandle_.advertise<nav_msgs::Odometry>
    (odometryTopic.str (), 1);

  try
    {
      model.bodyInfo =
	hrp::loadBodyInfo (req.model_vrml.c_str (), orb_);
    }
  catch (OpenHRP::ModelLoader::ModelLoaderException& e)
    {
      boost::format fmt ("failed to load model: %1%. Reason: %2%");
      fmt % req.model_vrml % e.description;

      ROS_WARN_STREAM (fmt.str ());
      res.status_message = fmt.str ();
      res.success = false;
      return true;
    }

  if (!model.bodyInfo)
    {
      boost::format fmt ("failed to load model: %1%");
      fmt % req.model_vrml;

      ROS_WARN_STREAM (fmt.str ());
      res.status_message = fmt.str ();
      res.success = false;
      return true;
    }

  models_.push_back (model);
  res.status_message = "";
  res.success = true;
  return true;
}

bool
SchedulerNode::startSimulationCallback
(openhrp_msgs::StartSimulation::Request& req,
 openhrp_msgs::StartSimulation::Response& res)
{
  res.status_message = "";
  res.success = false;

  // Reset time.
  time_.sec = 0;
  time_.nsec = 0;

  // Initialize online viewer.
  onlineViewer_ = hrp::getOnlineViewer(cxt_);


  // Load models into online viewer.
  if (CORBA::is_nil (onlineViewer_))
    ROS_WARN_STREAM ("Online Viewer not found");
  else
    {
      BOOST_FOREACH (const ModelInfo& model, models_)
	{
	  onlineViewer_->load (model.meta.model_name.c_str (),
			       model.meta.model_vrml.c_str ());
	}
      onlineViewer_->clearLog ();
    }

  // Retrieve the dynamics simulator factory.
  OpenHRP::DynamicsSimulatorFactory_var
    dynamicsSimulatorFactory =
    checkCorbaServer <OpenHRP::DynamicsSimulatorFactory,
    OpenHRP::DynamicsSimulatorFactory_var>
    ("DynamicsSimulatorFactory", cxt_);

  if (CORBA::is_nil(dynamicsSimulatorFactory))
    {
      res.status_message = "DynamicsSimulatorFactory not found";
      res.success = false;
    }

  // Create the dynamics simulator.
  dynamicsSimulator_ = dynamicsSimulatorFactory->create();

  if (CORBA::is_nil (dynamicsSimulator_))
    {
      res.status_message = "failed to initialize dynamics simulator";
      res.success = false;
      return true;
    }

  // Register models into the dynamics simulator.
  BOOST_FOREACH (const ModelInfo& model, models_)
      dynamicsSimulator_->registerCharacter
	(model.meta.model_name.c_str (),
	 model.bodyInfo);

  // Setup the simulation.
  dynamicsSimulator_->init
    (timeStep_, integrationMethod_, enableSensor_);

  // Setting the gravity vector.
  OpenHRP::DblSequence3 g;
  g.length (3);
  g[0] = gravity_[0];
  g[1] = gravity_[1];
  g[2] = gravity_[2];
  dynamicsSimulator_->setGVector (g);

  // Place the models into the simulation.
  BOOST_FOREACH (const ModelInfo& model, models_)
    {
      // Set position.
      OpenHRP::DblSequence trans;
      trans.length(12);

      trans[0] = model.meta.initial_pose.position.x;
      trans[1] = model.meta.initial_pose.position.y;
      trans[2] = model.meta.initial_pose.position.z;

      btQuaternion quaternion
	(model.meta.initial_pose.orientation.x,
	 model.meta.initial_pose.orientation.y,
	 model.meta.initial_pose.orientation.z,
	 model.meta.initial_pose.orientation.w);
      btMatrix3x3 rotation (quaternion);

      for (unsigned i = 0; i < 3; ++i)
	for (unsigned j = 0; j < 3; ++j)
	  trans[3 + (i * 3) + j] = rotation[i][j];

      dynamicsSimulator_->setCharacterLinkData
	(model.meta.model_name.c_str (), rootBodyName,
	 OpenHRP::DynamicsSimulator::ABS_TRANSFORM, trans);

      //FIXME: set joint angles.
    }

  // Prepare for first iteration.
  dynamicsSimulator_->initSimulation();

  // Load collision pairs.

  // spring damper values not used now.
  OpenHRP::DblSequence6 K, C;
  K.length (0);
  C.length (0);
  double statFric = 0.5;
  double slipFric = 0.5;
  double culling_thresh = 0.01;

  BOOST_FOREACH (const ModelInfo& model1, models_)
    {
      BOOST_FOREACH (const ModelInfo& model2, models_)
	{
	  if (model1.bodyInfo == model2.bodyInfo)
	    continue;

	  dynamicsSimulator_->registerCollisionCheckPair
	    (model1.meta.model_name.c_str (), "",
	     model2.meta.model_name.c_str (), "",
	     statFric, slipFric, K, C, culling_thresh, 0.0);
	}
    }
  dynamicsSimulator_->calcWorldForwardKinematics();

  // Load controllers.


  BOOST_SCOPE_EXIT ((&res) (&dynamicsSimulator_))
    {
      if (!res.success)
	{
	  dynamicsSimulator_->destroy ();
	  dynamicsSimulator_ = 0;
	}
    } BOOST_SCOPE_EXIT_END;

  res.status_message = "";
  res.success = true;
  state_ = STATE_STARTED;
  return true;
}

bool
SchedulerNode::pauseSimulationCallback
(std_srvs::Empty::Request& req,
 std_srvs::Empty::Response& res)
{
  if (!state_ == STATE_IDLE)
    return false;
  state_ = STATE_PAUSED;
  return true;
}

bool
SchedulerNode::unpauseSimulationCallback
(std_srvs::Empty::Request& req,
 std_srvs::Empty::Response& res)
{
  if (!state_ == STATE_IDLE)
    return false;
  state_ = STATE_STARTED;
  return true;
}

namespace
{
  static void copy (std::vector<double>& dst,
		    const OpenHRP::DblSequence_var& src)
  {
    dst.resize (src->length());
    for (unsigned i = 0; i < src->length (); ++i)
      dst[i] = src.in ()[i];
  }
} // end of anonymous namespace.

void
SchedulerNode::publishModelsData ()
{
  BOOST_FOREACH (const ModelInfo& model, models_)
    {
      // Retrieve data.
      OpenHRP::DblSequence_var q;
      OpenHRP::DblSequence_var dq;
      OpenHRP::DblSequence_var tau;
      OpenHRP::DblSequence_var position;
      OpenHRP::DblSequence_var velocity;
      OpenHRP::DblSequence_var acceleration;

      try
	{
	  dynamicsSimulator_->getCharacterAllLinkData
	    (model.meta.model_name.c_str (),
	     OpenHRP::DynamicsSimulator::JOINT_VALUE, q);

	  dynamicsSimulator_->getCharacterAllLinkData
	    (model.meta.model_name.c_str (),
	     OpenHRP::DynamicsSimulator::JOINT_VELOCITY, dq);

	  dynamicsSimulator_->getCharacterAllLinkData
	    (model.meta.model_name.c_str (),
	     OpenHRP::DynamicsSimulator::JOINT_TORQUE, tau);

	  dynamicsSimulator_->getCharacterLinkData
	    (model.meta.model_name.c_str (),
	     rootBodyName,
	     OpenHRP::DynamicsSimulator::ABS_TRANSFORM, position);

	  dynamicsSimulator_->getCharacterLinkData
	    (model.meta.model_name.c_str (),
	     rootBodyName,
	     OpenHRP::DynamicsSimulator::ABS_VELOCITY, velocity);

	  dynamicsSimulator_->getCharacterLinkData
	    (model.meta.model_name.c_str (),
	     rootBodyName,
	     OpenHRP::DynamicsSimulator::ABS_ACCELERATION, acceleration);
	}
      catch (...)
	{} //FIXME:

      // Joint state.
      sensor_msgs::JointStatePtr jointState
	(new sensor_msgs::JointState);
      jointState->header.seq = 0;
      jointState->header.stamp = time_;
      jointState->header.frame_id = "";


      OpenHRP::LinkInfoSequence_var links = model.bodyInfo->links ();
      jointState->name.resize (links->length ());
      for (unsigned i = 0; i < links->length (); ++i)
	jointState->name[i] = links[i].name;

      //jointState->name
      copy (jointState->position, q);
      copy (jointState->velocity, dq);
      copy (jointState->effort, tau);
      model.jointState.publish (jointState);


      // Odometry.
      nav_msgs::OdometryPtr odometry
	(new nav_msgs::Odometry);

      odometry->header.seq = 0;
      odometry->header.stamp = time_;
      odometry->header.frame_id = worldFrame_;
      odometry->child_frame_id = model.meta.model_name;
      odometry->pose.pose.position.x = 0.;
      odometry->pose.pose.position.y = 0.;
      odometry->pose.pose.position.z = 0.;
      odometry->pose.pose.orientation.x = 0.;
      odometry->pose.pose.orientation.y = 0.;
      odometry->pose.pose.orientation.z = 0.;
      odometry->pose.pose.orientation.w = 1.;
      for (unsigned i = 0; i < 6; ++i)
	for (unsigned j = 0; j < 6; ++j)
	  odometry->pose.covariance[(i * 6) + j] =
	    (i == j) ? 1. : 0.;

      odometry->twist.twist.linear.x = 0.;
      odometry->twist.twist.linear.y = 0.;
      odometry->twist.twist.linear.z = 0.;
      odometry->twist.twist.angular.x = 0.;
      odometry->twist.twist.angular.y = 0.;
      odometry->twist.twist.angular.z = 0.;
      for (unsigned i = 0; i < 6; ++i)
	for (unsigned j = 0; j < 6; ++j)
	  odometry->twist.covariance[(i * 6) + j] =
	    (i == j) ? 1. : 0.;
      model.odometry.publish (odometry);

      // Tf.
      geometry_msgs::TransformStamped transform;
      transform.header.seq = 0;
      transform.header.stamp = time_;
      transform.header.frame_id = worldFrame_;
      transform.child_frame_id = model.meta.model_name;

      transform.transform.translation.x = 0.;
      transform.transform.translation.y = 0.;
      transform.transform.translation.z = 0.;

      transform.transform.rotation.x = 0.;
      transform.transform.rotation.y = 0.;
      transform.transform.rotation.z = 0.;
      transform.transform.rotation.w = 1.;

      transformBroadcaster_.sendTransform (transform);
    }
}

void
SchedulerNode::spin ()
{
  ROS_INFO ("scheduler started");

  ros::Rate rate (100);
  while (ros::ok ())
    {
      if (state_ == STATE_STARTED)
	{
	  time_ += ros::Duration (timeStep_);
	  dynamicsSimulator_->stepSimulation ();

	  dynamicsSimulator_->getWorldState (worldState_);
	  if (!CORBA::is_nil (onlineViewer_))
	    onlineViewer_->update (worldState_);
	  publishModelsData ();
	}

      // Publish simulation results.
      rosgraph_msgs::ClockPtr clock (new rosgraph_msgs::Clock);
      clock->clock = time_;
      clock_.publish (clock);

      ros::spinOnce ();

      //rate.sleep ();
    }
}


int main (int argc, char* argv[])
{
  ros::init (argc, argv, "scheduler");

  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate ("~");

  SchedulerNode node (argc, argv, nh, nhPrivate);
  node.spin ();
}
