/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "route_following_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/geometry/LineString.h>
#include <string>


namespace route_following_plugin
{

    TEST(RouteFollowingPluginTest, testFindLaneletIndexFromPath)
    {
        RouteFollowingPlugin rfp;
        lanelet::ConstLanelets lls;
        lanelet::Lanelet ll;
        ll.setId(15);
        lls.push_back(ll);
        lanelet::routing::LaneletPath path(lls);
        EXPECT_EQ(0, rfp.findLaneletIndexFromPath(15, path));
        EXPECT_EQ(-1, rfp.findLaneletIndexFromPath(5, path));
    }

    TEST(RouteFollowingPluginTest, testComposeManeuverMessage)
    {
        RouteFollowingPlugin rfp;
        auto msg = rfp.composeManeuverMessage(1.0, 10.0, 0.9, RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS, 2, ros::Time(0, 0));
        EXPECT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, msg.type);
        EXPECT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, msg.lane_following_maneuver.parameters.neogition_type);
        EXPECT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN, msg.lane_following_maneuver.parameters.presence_vector);
        EXPECT_EQ("InLaneCruisingPlugin", msg.lane_following_maneuver.parameters.planning_tactical_plugin);
        EXPECT_EQ("RouteFollowingPlugin", msg.lane_following_maneuver.parameters.planning_strategic_plugin);
        EXPECT_NEAR(1.0, msg.lane_following_maneuver.start_dist, 0.01);
        EXPECT_NEAR(0.9, msg.lane_following_maneuver.start_speed, 0.01);
        EXPECT_EQ(ros::Time(0), msg.lane_following_maneuver.start_time);
        EXPECT_NEAR(10.0, msg.lane_following_maneuver.end_dist, 0.01);
        EXPECT_NEAR(25 / 2.237, msg.lane_following_maneuver.end_speed, 0.01);
        EXPECT_TRUE(msg.lane_following_maneuver.end_time - ros::Time(1.49) < ros::Duration(0.01));
        EXPECT_EQ("2", msg.lane_following_maneuver.lane_id);
    }

    TEST(RouteFollowingPluginTest, testIdentifyLaneChange)
    {
        RouteFollowingPlugin rfp;
        auto relations = lanelet::routing::LaneletRelations();
        EXPECT_FALSE(rfp.identifyLaneChange(relations, 1));
        lanelet::routing::LaneletRelation relation;
        relation.relationType = lanelet::routing::RelationType::Successor;
        relations.push_back(relation);
        EXPECT_TRUE(rfp.identifyLaneChange(relations, 0));
    }

    TEST(RouteFollowingPlugin,DISABLED_TestAssociateSpeedLimit)
    {
        //Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_=25;
        options.lane_width_=3.7;
        options.speed_limit_=carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_=carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map=carma_wm::test::buildGuidanceTestMap(options.lane_width_,options.lane_length_);

        //set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210,1213},cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules=lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        //Compute and print shortest path
        lanelet::Lanelet start_lanelet=map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet=map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);
 
        cmw.get()->setConfigSpeedLimit(30.0);

        RouteFollowingPlugin worker;
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker.wm_=cmw;

        //Define current position and velocity
        worker.pose_msg_.pose.position.x=5.55;
        worker.pose_msg_.pose.position.y=12.5;
        worker.pose_msg_.pose.position.z=0.0;
        
        worker.pose_msg_.pose.orientation.x=0.0;
        worker.pose_msg_.pose.orientation.y=0.0;
        worker.pose_msg_.pose.orientation.z=0.0;
        worker.pose_msg_.pose.orientation.w=0.0;
        //define twist
        worker.current_speed_=10.0;
        
       //Define plan for request and response
        //PlanManeuversRequest
        cav_srvs::PlanManeuvers plan;
        cav_srvs::PlanManeuversRequest pplan;
        
        cav_msgs::ManeuverPlan plan_req1;
        plan_req1.header;
        plan_req1.maneuver_plan_id;
        plan_req1.planning_start_time;
        plan_req1.planning_completion_time;
        //cav_msgs::Maneuver RouteFollowingPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time)
        plan_req1.maneuvers.push_back(worker.composeManeuverMessage(0,0,0,0,0,ros::Time(0)));
        pplan.prior_plan=plan_req1;
        plan.request=pplan;
        //PlanManeuversResponse 
        cav_srvs::PlanManeuversResponse newplan;
        for(auto i=0;i<plan_req1.maneuvers.size();i++) newplan.new_plan.maneuvers.push_back(plan_req1.maneuvers[i]);

        plan.response=newplan;
        
        //RouteFollowing plan maneuver callback
        ros::Time::init();  
        if(worker.plan_maneuver_cb(plan.request,plan.response)){    
            //check target speeds in updated response
            lanelet::Velocity limit=30_mph;
            ASSERT_EQ(plan.response.new_plan.maneuvers[0].lane_following_maneuver.end_speed,0);
            for(auto i=1;i<plan.response.new_plan.maneuvers.size();i++){
                ASSERT_EQ(plan.response.new_plan.maneuvers[i].lane_following_maneuver.end_speed, limit.value()) ;
            }
        }
        else{
            EXPECT_TRUE(false);
        } 
        

    }

    TEST(RouteFollowingPlugin,TestAssociateSpeedLimitusingosm)
    {
        // File to process. Path is relative to test folder
        std::string file = "../resource/map/town01_vector_map_1.osm";
        lanelet::Id start_id=101;
        lanelet::Id end_id=111;
        /***
         * VAVLID PATHs (consists of lanenet ids): (This is also the shortest path because certain Lanelets missing)
         * 159->160->164->136->135->137->144->121; 
         * 159->160->164->136->135->137->144->118;
         * 168->170->111
         * 159->161->168->170->111
         * 167->169->168->170->111
         * 115->146->140->139->143->167->169->168->170->111 
         * 141->139->143->167->169->168->170->111 
         * 127->146->140->139->143->167->169->168->170->111 
         * 101->100->104->167->169->168->170->111 (a counter cLock circle) 
         * **/
        // Write new map to file
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        // Parse geo reference info from the original lanelet map (.osm)
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        RouteFollowingPlugin worker;
        //get position on map
        auto llt=map.get()->laneletLayer.get(101);
        lanelet::LineString3d left_bound=llt.leftBound();
        lanelet::LineString3d right_bound=llt.rightBound();
        geometry_msgs::PoseStamped left;
        geometry_msgs::PoseStamped right;
        for(lanelet::Point3d& p : left_bound)
        {
            left.pose.position.x=p.x();
            left.pose.position.y=p.y();
            left.pose.position.z=p.z();

        }
        for(lanelet::Point3d& p : right_bound)
        {
            right.pose.position.x=p.x();
            right.pose.position.y=p.y();
            right.pose.position.z=p.z();
        }
        worker.pose_msg_.pose.position.x=(left.pose.position.x+right.pose.position.x)/2;
        worker.pose_msg_.pose.position.y=(left.pose.position.y+right.pose.position.y)/2;
        worker.pose_msg_.pose.position.z=(left.pose.position.z+right.pose.position.z)/2;

        worker.pose_msg_.pose.orientation.x=0.0;
        worker.pose_msg_.pose.orientation.y=0.0;
        worker.pose_msg_.pose.orientation.z=0.0;
        worker.pose_msg_.pose.orientation.w=0.0;

        //define twist
        worker.current_speed_=0.0;

        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker.wm_=cmw;
        
        //Define plan for request and response
        //PlanManeuversRequest
        cav_srvs::PlanManeuvers plan;
        cav_srvs::PlanManeuversRequest pplan;
        
        cav_msgs::ManeuverPlan plan_req1;
        plan_req1.header;
        plan_req1.maneuver_plan_id;
        plan_req1.planning_start_time;
        plan_req1.planning_completion_time;
        
        plan_req1.maneuvers.push_back(worker.composeManeuverMessage(0,0,0,0,0,ros::Time(0)));
        pplan.prior_plan=plan_req1;
        plan.request=pplan;
        //PlanManeuversResponse 
        cav_srvs::PlanManeuversResponse newplan;
        for(auto i=0;i<plan_req1.maneuvers.size();i++) newplan.new_plan.maneuvers.push_back(plan_req1.maneuvers[i]);

        plan.response=newplan;
    
        ros::Time::init();  //initializing ros time to use ros::Time::now()
        if(worker.plan_maneuver_cb(plan.request,plan.response)){    
            //check target speeds in updated response
            lanelet::Velocity limit=25_mph;
            ASSERT_EQ(plan.response.new_plan.maneuvers[0].lane_following_maneuver.end_speed,0);
            //std::cout<<"Maneuver size:"<<
            for(auto i=1;i<plan.response.new_plan.maneuvers.size();i++){
                //std::cout<<"maneuver:"<<i<<" "<< plan.response.new_plan.maneuvers[i].lane_following_maneuver.end_speed<<std::endl;
                ASSERT_EQ(plan.response.new_plan.maneuvers[i].lane_following_maneuver.end_speed, limit.value()) ;
            }
        }
        else{
            EXPECT_TRUE(false);
        }
        //Test findSpeedLimit function
        lanelet::BasicPoint2d current_loc(worker.pose_msg_.pose.position.x, worker.pose_msg_.pose.position.y);
        auto current_lanelets= lanelet::geometry::findNearest(worker.wm_->getMap()->laneletLayer, current_loc, 10); 
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
        double speed=worker.findSpeedLimit(current_lanelet);
        if(speed < 11.176)
        {
            ASSERT_EQ(speed, worker.config_limit);
        }
        else ASSERT_EQ(speed,11.176);                                                                            
    }

    

}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}


