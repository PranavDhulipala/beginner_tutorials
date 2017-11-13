#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/service.h"
#include "std_msgs/String.h"

struct Helper {
  Helper()
      : count(0) {
  }

  void cb(const std_msgs::String::ConstPtr& msg) {
    ++count;
  }
  uint32_t count;
};

/**
 * @brief test to check the existence of publisher and subscriber
 */
TEST(TestSuite, talkerTest) {
  ros::NodeHandle nh;
  Helper h;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber chatter_sub = nh.subscribe("chatter", 1000, &Helper::cb, &h);
  EXPECT_EQ(chatter_pub.getNumSubscribers(), 1U);
  EXPECT_EQ(chatter_sub.getNumPublishers(), 1U);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

