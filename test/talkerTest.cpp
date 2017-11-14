/**
 *  @file    talkerTest.cpp
 *  @author  Pranav Dhulipala
 *  @copyright  MIT License (c) 2017 Pranav Dhulipala
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 *  @brief test node
 *
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/service.h"
#include "std_msgs/String.h"
/**
 * @brief contains the callback method used to count
 */
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

