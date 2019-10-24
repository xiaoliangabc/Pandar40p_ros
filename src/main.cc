/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <ros/ros.h>
#include <arpa/inet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pandar40p_sdk/pandar40p_sdk.h"

class Pandar40PClient {
 public:
  Pandar40PClient(ros::NodeHandle node, ros::NodeHandle nh) {
    // default configure
    std::string ip = std::string("192.168.20.51");
    int lidarRecvPort = 2368;
    int gpsRecvPort = 10110;
    int startAngle = 135;
    std::string lidarTopic = "/pandar40p/sensor/pandar40p/hesai40/PointCloud2";
    int timezone = 8;
    std::string frameId = std::string("hesai40");

    //  parse nodehandle param
    bool ret = parseParameter(nh, &ip, &lidarRecvPort, &gpsRecvPort, \
            &startAngle, &lidarTopic, &timezone, &frameId);
    if (!ret) {
        ROS_INFO("Parse parameters failed, please check parameters above.");
        return;
    }

    // advertise
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>(lidarTopic, 10);

    psdk = new Pandar40PSDK(ip, lidarRecvPort, gpsRecvPort,
            boost::bind(&Pandar40PClient::lidarCallback, this, _1, _2),
            NULL, startAngle * 100, timezone, frameId);
    psdk->Start();
  }

  bool parseParameter(ros::NodeHandle nh, std::string* ip, int* lidarRecvPort, \
                      int* gpsRecvPort, int* startAngle, \
                      std::string* lidarTopic, int* timezone, \
                      std::string* frameId) {
    if (nh.hasParam("pandar40p_ip")) {
      nh.getParam("pandar40p_ip", *ip);
    }
    if (nh.hasParam("lidar_recv_port")) {
      nh.getParam("lidar_recv_port", *lidarRecvPort);
    }
    if (nh.hasParam("gps_recv_port")) {
      nh.getParam("gps_recv_port", *gpsRecvPort);
    }
    if (nh.hasParam("start_angle")) {
      nh.getParam("start_angle", *startAngle);
    }
    if (nh.hasParam("lidar_topic")) {
      nh.getParam("lidar_topic", *lidarTopic);
    }
    if (nh.hasParam("timezone")) {
      nh.getParam("timezone", *timezone);
    }
    if (nh.hasParam("frame_id")) {
      nh.getParam("frame_id", *frameId);
    }

    std::cout << "Configs: pandar40pIP: " << *ip \
        << ", lidarRecvPort: " << *lidarRecvPort \
        << ", gpsRecvPort: " << *gpsRecvPort \
        << ", startAngle: " << *startAngle \
        << ", lidarTopic: " << *lidarTopic \
        << ", frameId: " << *frameId << std::endl;

    // check
    struct sockaddr_in sa;
    return checkPort(*lidarRecvPort) && checkPort(*gpsRecvPort)
        && (*startAngle >= 0) && (*startAngle < 360)
        && (1 == inet_pton(AF_INET, ip->c_str(), &(sa.sin_addr)));
  }

  bool checkPort(int port) {
    return (port > 0) && (port < 65535);
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }

  ~Pandar40PClient() {
    if (NULL != psdk) {
      delete(psdk);
    }
  }

 private:
  ros::Publisher lidarPublisher;
  Pandar40PSDK *psdk;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pandar40p_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  Pandar40PClient pandar40pClient(node, nh);

  ros::spin();
  return 0;
}
