#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/scope_exit.hpp>

#include <LinearMath/btMatrix3x3.h>

#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>

#include <OpenHRP-3.1/hrpModel/ModelLoaderUtil.h>
#include <OpenHRP-3.1/hrpUtil/OnlineViewerUtil.h>

#include "scheduler.hh"

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].id = CORBA::string_dup(n.c_str());
  ncName[0].kind = CORBA::string_dup("");
  X_ptr srv = NULL;
  try {
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
    mutex_ (),
    reconfigureSrv_ (mutex_, nodeHandlePrivate_),
    clock_ (),
    spawnVrmlModel_ (),
    startSimulation_ (),
    orb_ (),
    cxt_ (),
    onlineViewer_ (),
    dynamicsSimulator_ (),
    controllers_ (),
    gravity_ (),
    timeStep_ (),
    integrationMethod_ (),
    enableSensor_ ()
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

  OpenHRP::BodyInfo_var model =
    hrp::loadBodyInfo (req.model_vrml.c_str (), orb_);
  if (!model)
    {
      boost::format fmt ("failed to load model: %1%");
      fmt % req.model_vrml;

      ROS_WARN_STREAM (fmt.str ());
      res.status_message = fmt.str ();
      res.success = false;
      return true;
    }
  models_.push_back (std::make_pair (req, model));

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
  typedef std::pair<openhrp_msgs::SpawnModel::Request,
    OpenHRP::BodyInfo_var> model_t;

  if (CORBA::is_nil (onlineViewer_))
    ROS_WARN_STREAM ("Online Viewer not found");
  else
    {
      BOOST_FOREACH (const model_t& model, models_)
	{
	  onlineViewer_->load (model.first.model_name.c_str (),
			       model.first.model_vrml.c_str ());
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
  BOOST_FOREACH (const model_t& model, models_)
      dynamicsSimulator_->registerCharacter
	(model.first.model_name.c_str (),
	 model.second);

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
  BOOST_FOREACH (const model_t& model, models_)
    {
      // Set position.
      OpenHRP::DblSequence trans;
      trans.length(12);

      trans[0] = model.first.initial_pose.position.x;
      trans[1] = model.first.initial_pose.position.y;
      trans[2] = model.first.initial_pose.position.z;

      btQuaternion quaternion
	(model.first.initial_pose.orientation.x,
	 model.first.initial_pose.orientation.y,
	 model.first.initial_pose.orientation.z,
	 model.first.initial_pose.orientation.w);
      btMatrix3x3 rotation (quaternion);

      for (unsigned i = 0; i < 3; ++i)
	for (unsigned j = 0; j < 3; ++j)
	  trans[3 + (i * 3) + j] = rotation[i][j];

      dynamicsSimulator_->setCharacterLinkData
	(model.first.model_name.c_str (), "WAIST",
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

  BOOST_FOREACH (const model_t& model1, models_)
    {
      BOOST_FOREACH (const model_t& model2, models_)
	{
	  if (model1.second == model2.second)
	    continue;

	  dynamicsSimulator_->registerCollisionCheckPair
	    (model1.first.model_name.c_str (), "",
	     model2.first.model_name.c_str (), "",
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

void
SchedulerNode::spin ()
{
  ROS_INFO ("scheduler started");

  OpenHRP::WorldState_var state;

  ros::Rate rate (100);
  while (ros::ok ())
    {
      if (state_ == STATE_STARTED)
	{
	  time_ += ros::Duration (timeStep_);
	  dynamicsSimulator_->stepSimulation ();

	  dynamicsSimulator_->getWorldState (state);
	  if (!CORBA::is_nil (onlineViewer_))
	    onlineViewer_->update (state);
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
