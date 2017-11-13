#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <src/talker.h>

std::shared_ptr<ros::NodeHandle> nh;


TEST(TESTSuite, testChangeStringService) {
  ros::NodeHandle n;
  ros::ServiceClient client =
      nh->serviceClient<beginner_tutorials::changeString>("changeString");
  beginner_tutorials::changeString srv;
  srv.request.str1 = "hello";
  client.call(srv);
  EXPECT_STREQ("hello", srv.response.str2);
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "changeStringServiceClient");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


