#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Bool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/GazeboSpawnerDiagnostics.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* class Tester //{ */

class Tester {

public:
  Tester();

  bool test();

private:
  ros::NodeHandle                    nh_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>     sh_estim_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>  sh_spawner_diagnostics_;
  mrs_lib::SubscribeHandler<std_msgs::Bool>                      sh_can_takeoff;

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard_;

  mrs_lib::ServiceClientHandler<mrs_msgs::String> sch_spawn_;
};

//}

/* Tester() //{ */

Tester::Tester() {

  // | ------------------ initialize test node ------------------ |

  nh_ = ros::NodeHandle("~");

  ROS_INFO("[%s]: ROS node initialized", ros::this_node::getName().c_str());

  ros::Time::waitForValid();

  spinner_ = std::make_shared<ros::AsyncSpinner>(4);
  spinner_->start();

  std::string uav_name = "uav1";

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Test";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "/" + uav_name + "/control_manager/diagnostics");
  sh_estim_manager_diag_   = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "/" + uav_name + "/estimation_manager/diagnostics");
  sh_can_takeoff           = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "/" + uav_name + "/automatic_start/can_takeoff");
  sh_spawner_diagnostics_  = mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>(shopts, "/mrs_drone_spawner/diagnostics");

  ROS_INFO("[%s]: subscribers initialized", ros::this_node::getName().c_str());

  // | --------------------- service clients -------------------- |

  sch_arming_   = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "/" + uav_name + "/hw_api/arming");
  sch_offboard_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + uav_name + "/hw_api/offboard");
  sch_spawn_    = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/mrs_drone_spawner/spawn");

  ROS_INFO("[%s]: service client initialized", ros::this_node::getName().c_str());
}

//}

/* test() //{ */

bool Tester::test() {

  // | ------------ wait for the spawner diagnostics ------------ |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for spawner diagnostics", ros::this_node::getName().c_str());

    if (sh_spawner_diagnostics_.hasMsg()) {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  // | -------------------------- spawn ------------------------- |

  ROS_INFO("[%s]: spawning the drone", ros::this_node::getName().c_str());

  {
    mrs_msgs::String spawn;
    spawn.request.value = "1 --x500 --enable-rangefinder";

    {
      bool service_call = sch_spawn_.call(spawn);

      if (!service_call || !spawn.response.success) {
        ROS_ERROR("[%s]: spawning failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  // | ------------------- wait for the spawn ------------------- |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the spawn", ros::this_node::getName().c_str());

    if (!sh_spawner_diagnostics_.getMsg()->processing) {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  ROS_INFO("[%s]: The UAV is ready", ros::this_node::getName().c_str());

  ros::Duration(1.0).sleep();

  // | ---------------- wait for ready to takeoff --------------- |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", ros::this_node::getName().c_str());

    if (sh_control_manager_diag_.hasMsg() && sh_estim_manager_diag_.hasMsg()) {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  ROS_INFO("[%s]: MRS UAV System is ready", ros::this_node::getName().c_str());

  ros::Duration(1.0).sleep();

  // | ---------------------- arm the drone --------------------- |

  ROS_INFO("[%s]: arming the edrone", ros::this_node::getName().c_str());

  {
    std_srvs::SetBool arming;
    arming.request.data = true;

    {
      bool service_call = sch_arming_.call(arming);

      if (!service_call || !arming.response.success) {
        ROS_ERROR("[%s]: arming service call failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  ros::Duration(0.2).sleep();

  // | -------------------- trigger offboard -------------------- |

  ROS_INFO("[%s]: triggering offboard", ros::this_node::getName().c_str());

  {
    std_srvs::Trigger trigger;

    {
      bool service_call = sch_offboard_.call(trigger);

      if (!service_call || !trigger.response.success) {
        ROS_ERROR("[%s]: offboard service call failed", ros::this_node::getName().c_str());
        return false;
      }
    }
  }

  // | -------------- wait for the takeoff finished ------------- |

  while (ros::ok()) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the takeoff to finish", ros::this_node::getName().c_str());

    if (sh_control_manager_diag_.getMsg()->flying_normally) {

      ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

      return true;
    }

    ros::Duration(0.01).sleep();
  }

  return false;
}

//}

std::shared_ptr<Tester> tester_;

TEST(TESTSuite, takeoff) {

  bool result = tester_->test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "takeoff_test");

  tester_ = std::make_shared<Tester>();

  testing::InitGoogleTest(&argc, argv);

  Tester tester;

  return RUN_ALL_TESTS();
}
