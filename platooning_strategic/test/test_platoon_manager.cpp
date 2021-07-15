#include "platoon_manager.hpp"
#include "platoon_strategic.hpp"
#include "platoon_config.h"
// #include "platoon_strategic.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/CARMAUtils.h>
// #include "TestHelpers.h"

using namespace platoon_strategic;

// TEST(PlatoonManagerTest, test_construct)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;

// }

// TEST(PlatoonManagerTest, test_ecef_encode)
// {
//     ros::Time::init();

//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     cav_msgs::LocationECEF ecef_point_test;
//     ecef_point_test.ecef_x = 1.0;
//     ecef_point_test.ecef_y = 2.0;
//     ecef_point_test.ecef_z = 3.0;
//     plugin.pose_ecef_point_ = ecef_point_test;
//     plugin.run_leader_waiting();
//     // plugin.pm_.current_platoon_state = PlatoonState::LEADER;

// }

// TEST(PlatoonManagerTest, test_ecef_decode)
// {
//     ros::Time::init();

//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    
//     cav_msgs::MobilityOperation msg;
//     std::string strategyParams("INFO|REAR:1,LENGTH:2,SPEED:3,SIZE:4,DTD:5,ECEFX:6,ECEFY:7,ECEFZ:8");

//     msg.strategy_params = strategyParams;

//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;
//     plugin.pm_.current_plan.valid == false;

//     plugin.mob_op_cb(msg);

// }

// TEST(PlatoonManagerTest, test_split)
// {
//     cav_msgs::MobilityOperation msg;
//     std::string strategyParams("INFO|REAR:1,LENGTH:2,SPEED:3,SIZE:4");
//     std::vector<std::string> inputsParams;
//     boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));
//     std::vector<std::string> rearVehicleBsmId_parsed;
//     boost::algorithm::split(rearVehicleBsmId_parsed, inputsParams[0], boost::is_any_of(":"));
//     std::string rearVehicleBsmId = rearVehicleBsmId_parsed[1];
//     std::cout << "rearVehicleBsmId: " << rearVehicleBsmId << std::endl;

//     std::vector<std::string> rearVehicleDtd_parsed;
//     boost::algorithm::split(rearVehicleDtd_parsed, inputsParams[1], boost::is_any_of(":"));
//     double rearVehicleDtd = std::stod(rearVehicleDtd_parsed[1]);
//     std::cout << "rearVehicleDtd: " << rearVehicleDtd << std::endl;
// }


// TEST(PlatoonManagerTest, test_states)
// {
//     ros::Time::init();

//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;
//     plugin.pm_.current_downtrack_didtance_ = 20;

//     cav_msgs::MobilityRequest request;
//     request.plan_type.type = 3;
//     request.strategy_params = "SIZE:1,SPEED:0,DTD:11.5599";

//     plugin.mob_req_cb(request);

//     EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::LEADERWAITING);


// }

// TEST(PlatoonManagerTest, test_compose)
// {
//     std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%";
//     double cmdSpeed = 1;
//     double current_speed = 2;
//     double current_downtrack = 4;
//     boost::format fmter(OPERATION_STATUS_PARAMS);
//     fmter %cmdSpeed;
//     fmter %current_downtrack;
//     fmter %current_speed;
//     std::string statusParams = fmter.str();

//     std::cout << "statusParams: " << statusParams << std::endl;
// }

// TEST(PlatoonManagerTest, test1)
// {
//     std::vector<platoon_strategic::PlatoonMember> cur_pl;


//     platoon_strategic::PlatoonManager pm;
//     pm.platoon = cur_pl;

    
//     pm.isFollower = false;
//     pm.platoonSize = 1;
//     pm.leaderID = "0";
//     pm.currentPlatoonID = "a";

//     std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

//     pm.memberUpdates("1", "1", "1", params);

//     EXPECT_EQ(1, pm.platoon.size());

    
// }

// TEST(PlatoonStrategicPlugin, test_req_cb1)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;

//     cav_msgs::MobilityRequest request;
//     request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
//     request.strategy_params = "SIZE:2,SPEED:22,DTD:22";
//     plugin.mob_req_cb(request);
   
// }

// TEST(PlatoonStrategicPlugin, mob_resp_cb)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;

//     plugin.onSpin();
   
// }

// TEST(PlatoonStrategicPlugin, platoon_info_pub)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;

//     cav_msgs::PlatooningInfo info_msg1 = plugin.compose_platoon_info_msg();
//     EXPECT_EQ(info_msg1.leader_id, "default_id");

//     plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;
//     plugin.pm_.isFollower = true;
//     PlatoonMember member = PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
//     std::vector<PlatoonMember> cur_pl;
//     cur_pl.push_back(member);
//     plugin.pm_.platoon = cur_pl;

    
//     cav_msgs::PlatooningInfo info_msg2 = plugin.compose_platoon_info_msg();
//     EXPECT_EQ(info_msg2.leader_id, "1");
   
// }

// TEST(PlatoonStrategicPlugin, test_follower)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
//     EXPECT_EQ(plugin.pm_.isFollower, false);

//     cav_msgs::MobilityResponse resp;
//     resp.header.plan_id = "resp";
//     resp.is_accepted = true;
//     plugin.mob_resp_cb(resp);
//     EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::FOLLOWER);
//     EXPECT_EQ(plugin.pm_.isFollower, true);
// }

// TEST(PlatoonStrategicPlugin, test_get_leader)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;

//     PlatoonMember member = PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
//     std::vector<PlatoonMember> cur_pl;
//     cur_pl.push_back(member);

//     plugin.pm_.platoon = cur_pl;

//     EXPECT_EQ(plugin.pm_.platoon.size(), 1);

//     PlatoonMember member1 = plugin.pm_.platoon[0];

//     plugin.pm_.isFollower = true;
//     PlatoonMember platoon_leader = plugin.pm_.getLeader();

//     EXPECT_EQ(member1.staticId, "1");

//     EXPECT_EQ(platoon_leader.staticId, "1");
   
// }

// TEST(PlatoonStrategicPlugin, test_req_cb2)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADERWAITING;

//     cav_msgs::MobilityRequest request;
//     request.header.sender_id = "";
//     request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
//     request.strategy_params = "SIZE:2,SPEED:22,DTD:22";

//     // MobilityRequestResponse res = plugin.handle_mob_req(request);
//     // EXPECT_EQ(res, MobilityRequestResponse::ACK);
//     plugin.mob_req_cb(request);
//     EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::LEADER);
// }

// TEST(PlatoonStrategicPlugin, test_run_candidate)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
//     plugin.pm_.isFollower = true;
//     plugin.onSpin();

// }

// TEST(PlatoonStrategicPlugin, test_mob_op_cb_candidate)
// {
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
//     EXPECT_EQ(0, plugin.pm_.platoon.size());

//     plugin.pm_.isFollower = true;
//     cav_msgs::MobilityOperation op_msg;
//     op_msg.header.sender_id = "test-id";
//     op_msg.header.plan_id = "test_plan";
//     op_msg.strategy_params = "STATUS|CMDSPEED:1.0,DTD:22.0,SPEED:2.0";
//     plugin.mob_op_cb(op_msg);
//     EXPECT_EQ(1, plugin.pm_.platoon.size());
//     PlatoonMember platoon_leader = plugin.pm_.getLeader();
//     EXPECT_EQ("test-id", platoon_leader.staticId);
//     EXPECT_EQ(op_msg.header.sender_id, platoon_leader.staticId);
//     EXPECT_EQ(22, platoon_leader.vehiclePosition);
//     EXPECT_EQ(1, platoon_leader.commandSpeed);

// }

// TEST(PlatoonManagerTest, test2)
// {
//     platoon_strategic::PlatoonMember* member = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
//     std::vector<platoon_strategic::PlatoonMember> cur_pl;

//     cur_pl.push_back(*member);

//     platoon_strategic::PlatoonManager pm;
//     pm.platoon = cur_pl;

//     pm.isFollower = true;
//     pm.platoonSize = 1;
//     pm.leaderID = "0";
//     pm.currentPlatoonID = "a";

//     std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

//     ros::Time::init();

//     pm.updatesOrAddMemberInfo("2", "2", 2.0, 1.0, 2.5);
//     // updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed)

//     EXPECT_EQ(2, pm.platoon.size());
//     EXPECT_EQ("1", pm.platoon[0].staticId);

// }


// TEST(PlatoonManagerTest, test3)
// {
//     platoon_strategic::PlatoonMember* member1 = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
//     platoon_strategic::PlatoonMember* member2 = new platoon_strategic::PlatoonMember("2", "2", 2.0, 2.1, 0.2, 200);
//     std::vector<platoon_strategic::PlatoonMember> cur_pl;

//     cur_pl.push_back(*member1);
//     cur_pl.push_back(*member2);

//     platoon_strategic::PlatoonManager pm;
//     pm.platoon = cur_pl;

//     pm.isFollower = false;
//     pm.platoonSize = 2;
//     pm.leaderID = "0";
//     pm.currentPlatoonID = "a";


//     ros::Time::init();

//     int res = pm.getTotalPlatooningSize();

//     EXPECT_EQ(3, res);

// }

// TEST(PlatoonManagerTest, test4)
// {
//     platoon_strategic::PlatoonMember* member1 = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
//     platoon_strategic::PlatoonMember* member2 = new platoon_strategic::PlatoonMember("2", "2", 2.0, 2.1, 0.2, 200);
//     std::vector<platoon_strategic::PlatoonMember> cur_pl;

//     cur_pl.push_back(*member1);
//     cur_pl.push_back(*member2);

//     platoon_strategic::PlatoonManager pm;
//     pm.platoon = cur_pl;

//     pm.isFollower = true;
//     pm.platoonSize = 2;
//     pm.leaderID = "0";
//     pm.currentPlatoonID = "a";


//     ros::Time::init();

//     int res = pm.allPredecessorFollowing();

//     EXPECT_EQ(0, res);

// }


