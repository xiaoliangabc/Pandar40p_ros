/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
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

#include "pandar40p_sdk/pandar40p_sdk.h"

double pandar40pToSysTimeGap = 0;
int gpsTimestamp = 0;

void gpsCallback(int timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  gpsTimestamp = timestamp;
  pandar40pToSysTimeGap =
      static_cast<double>(ts.tv_sec) + \
      (static_cast<double>(ts.tv_usec) / 1000000.0) - \
      static_cast<double>(timestamp);
  printf("gps: %d, gap: %f\n", timestamp, pandar40pToSysTimeGap);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  printf("lidar: %lf with frame id : %s \n", timestamp, \
         cld->header.frame_id.c_str());
}

int main(int argc, char** argv) {
  Pandar40PSDK pandar40p(std::string("192.168.1.201"),
                  2368, 10110, lidarCallback, gpsCallback,
                  13500, 0, std::string("hesai40"));

  pandar40p.Start();

  sleep(1);

  printf("show lidar correction contents: %s\n", pandar40p.GetLidarCalibration().c_str());

  while (true) {
    sleep(100);
  }

  return 0;
}
