#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/function.hpp>

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
    srv = X::_narrow(cxt->resolve(ncName));
  } catch(const CosNaming::NamingContext::NotFound &exc) {
    std::cerr << n << " not found: ";
    switch(exc.why) {
    case CosNaming::NamingContext::missing_node:
      std::cerr << "Missing Node" << std::endl;
    case CosNaming::NamingContext::not_context:
      std::cerr << "Not Context" << std::endl;
      break;
    case CosNaming::NamingContext::not_object:
      std::cerr << "Not Object" << std::endl;
      break;
    }
    return (X_ptr)NULL;
  } catch(CosNaming::NamingContext::CannotProceed &exc) {
    std::cerr << "Resolve " << n << " CannotProceed" << std::endl;
  } catch(CosNaming::NamingContext::AlreadyBound &exc) {
    std::cerr << "Resolve " << n << " InvalidName" << std::endl;
  }
  return srv;
}

SchedulerNode::SchedulerNode (int argc, char* argv[],
			      ros::NodeHandle& nh,
			      ros::NodeHandle& privateNh)
  : nodeHandle_ (nh),
    nodeHandlePrivate_ (privateNh),
    mutex_ (),
    reconfigureSrv_ (mutex_, nodeHandlePrivate_),
    clock_ (),
    spawnVrmlModel_ (),
    deleteModel_ (),
    orb_ (),
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

  CosNaming::NamingContext_var cxt;
  CORBA::Object_var nS = orb_->resolve_initial_references ("NameService");
  cxt = CosNaming::NamingContext::_narrow (nS);

  // Initialize online viewer.
  onlineViewer_ = hrp::getOnlineViewer(orb_);

  // Initialize dynamics simulator.
  OpenHRP::DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
  dynamicsSimulatorFactory =
    checkCorbaServer <OpenHRP::DynamicsSimulatorFactory,
    OpenHRP::DynamicsSimulatorFactory_var>
    ("DynamicsSimulatorFactory", cxt);

  if (CORBA::is_nil(dynamicsSimulatorFactory)) {
    std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
  }

  dynamicsSimulator_ = dynamicsSimulatorFactory->create();

  reconfigure ();

  // Initialize publishers.
  clock_ =
    nodeHandle_.advertise<rosgraph_msgs::Clock>
    ("/clock", 1);

  // Initialize services.
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

  // Enable simulated time.
  nodeHandle_.setParam ("/use_sim_time", true);
}

void
SchedulerNode::reconfigureCallback
(openhrp_plugins::SimulationConfig& config, uint32_t level)
{
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

  reconfigure ();
}

void
SchedulerNode::reconfigure ()
{
  if (CORBA::is_nil (dynamicsSimulator_))
    return;

  // Setting the gravity vector.
  OpenHRP::DblSequence3 g;
  g.length (3);
  g[0] = gravity_[0];
  g[1] = gravity_[1];
  g[2] = gravity_[2];
  dynamicsSimulator_->setGVector (g);

  // If required, restart the simulation.
  dynamicsSimulator_->init
    (timeStep_, integrationMethod_, enableSensor_);

  dynamicsSimulator_->initSimulation();
}

bool
SchedulerNode::spawnVrmlModelCallback
(openhrp_msgs::SpawnModel::Request& req,
 openhrp_msgs::SpawnModel::Response& res)
{
  OpenHRP::BodyInfo_var model =
    hrp::loadBodyInfo (req.model_vrml.c_str (), orb_);

  // Load model into online viewer.
  onlineViewer_->load (req.model_name.c_str (),
		       req.model_vrml.c_str ());
  onlineViewer_->clearLog ();

  dynamicsSimulator_->registerCharacter
    (req.model_name.c_str (), model);


  // Set model initial position / posture.
  OpenHRP::DblSequence trans;
  trans.length(12);

  trans[0] = req.initial_pose.position.x;
  trans[1] = req.initial_pose.position.y;
  trans[2] = req.initial_pose.position.z;

  btQuaternion quaternion
    (req.initial_pose.orientation.x,
     req.initial_pose.orientation.y,
     req.initial_pose.orientation.z,
     req.initial_pose.orientation.w);
  btMatrix3x3 rotation (quaternion);

  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      trans[3 + (i * 3) + j] = rotation[i][j];

  dynamicsSimulator_->setCharacterLinkData
    (req.model_name.c_str (), "WAIST",
     OpenHRP::DynamicsSimulator::ABS_TRANSFORM, trans);

  models_.push_back (model);
  return true;
}

void
SchedulerNode::spin ()
{
  ROS_INFO ("scheduler started");

  ros::Rate rate (1. / timeStep_);
  rosgraph_msgs::Clock clock;
  while (ros::ok ())
    {
      time_ += ros::Duration (timeStep_);

      //dynamicsSimulator_->stepSimulation ();
      //dynamicsSimulator_->getWorldState (state);

      // Publish simulation results.
      clock.clock = time_;
      clock_.publish (clock);

      ros::spinOnce ();
      rate.sleep ();
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
