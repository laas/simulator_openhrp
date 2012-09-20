#include <ros/ros.h>
#include <ros/console.h>

#include <OpenHRP-3.1/hrpCorba/ModelLoader.hh>
#include <OpenHRP-3.1/hrpCorba/DynamicsSimulator.hh>


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

class SchedulerNode
{
public:
  explicit SchedulerNode (int argc, char* argv[]);
  void spin ();

private:
  CORBA::ORB_var orb_;
};

SchedulerNode::SchedulerNode (int argc, char* argv[])
{
  // Fake arguments for CORBA inialization.
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

  // Initialize dynamics simulator.
  OpenHRP::DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
  dynamicsSimulatorFactory =
    checkCorbaServer <OpenHRP::DynamicsSimulatorFactory,
    OpenHRP::DynamicsSimulatorFactory_var>
    ("DynamicsSimulatorFactory", cxt);
  
  if (CORBA::is_nil(dynamicsSimulatorFactory)) {
    std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
  }
  
  OpenHRP::DynamicsSimulator_var dynamicsSimulator =
    dynamicsSimulatorFactory->create();
}

void
SchedulerNode::spin ()
{
  ROS_INFO ("scheduler started");
  ros::spin ();
}


int main (int argc, char* argv[])
{
  ros::init (argc, argv, "scheduler");

  SchedulerNode node (argc, argv);
  node.spin ();
}
