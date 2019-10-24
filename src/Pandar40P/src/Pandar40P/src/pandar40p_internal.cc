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

#include <sstream>

#include "src/input.h"
#include "src/pandar40p_internal.h"

namespace apollo {
namespace drivers {
namespace hesai {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float pandar40p_elev_angle_map[] = {
    15.0f,  11.0f,  8.0f,   5.0f,   3.0f,   2.0f,   1.67f,  1.33f,
    1.0f,   0.67f,  0.33f,  0.0f,   -0.33f, -0.67f, -1.0f,  -1.33f,
    -1.66f, -2.0f,  -2.33f, -2.67f, -3.0f,  -3.33f, -3.67f, -4.0f,
    -4.33f, -4.67f, -5.0f,  -5.33f, -5.67f, -6.0f,  -7.0f,  -8.0f,
    -9.0f,  -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float pandar40p_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f,  -5.208f,
    -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,
    -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f,
    3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

Pandar40P_Internal::Pandar40P_Internal(
    std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    std::string frame_id) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  start_angle_ = start_angle;

  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
    cos_lookup_table_[rotIndex] = cosf(rotation);
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }

  input_.reset(new Input(lidar_port, gps_port));

  pcl_callback_ = pcl_callback;
  gps_callback_ = gps_callback;

  last_azimuth_ = 0;

  // init the block time offset, us
  blockOffset_[9] = 55.56f * 0.0f + 28.58f;
  blockOffset_[8] = 55.56f * 1.0f + 28.58f;
  blockOffset_[7] = 55.56f * 2.0f + 28.58f;
  blockOffset_[6] = 55.56f * 3.0f + 28.58f;
  blockOffset_[5] = 55.56f * 4.0f + 28.58f;
  blockOffset_[4] = 55.56f * 5.0f + 28.58f;
  blockOffset_[3] = 55.56f * 6.0f + 28.58f;
  blockOffset_[2] = 55.56f * 7.0f + 28.58f;
  blockOffset_[1] = 55.56f * 8.0f + 28.58f;
  blockOffset_[0] = 55.56f * 9.0f + 28.58f;

  // init the laser shot time offset, us
  laserOffset_[3] = 3.62f;
  laserOffset_[39] = 3.62f;
  laserOffset_[35] = 4.92f;
  laserOffset_[27] = 6.23f;
  laserOffset_[11] = 8.19f;
  laserOffset_[15] = 8.19f;
  laserOffset_[31] = 9.5f;
  laserOffset_[23] = 11.47f;
  laserOffset_[28] = 12.77f;
  laserOffset_[16] = 14.74f;
  laserOffset_[2] = 16.04f;
  laserOffset_[38] = 16.04f;
  laserOffset_[34] = 17.35f;
  laserOffset_[24] = 18.65f;
  laserOffset_[8] = 20.62f;
  laserOffset_[12] = 20.62f;
  laserOffset_[30] = 21.92f;
  laserOffset_[20] = 23.89f;
  laserOffset_[25] = 25.19f;
  laserOffset_[13] = 27.16f;
  laserOffset_[1] = 28.47f;
  laserOffset_[37] = 28.47f;
  laserOffset_[33] = 29.77f;
  laserOffset_[5] = 31.74f;
  laserOffset_[21] = 31.7447f;
  laserOffset_[9] = 33.71f;
  laserOffset_[29] = 35.01f;
  laserOffset_[17] = 36.98f;
  laserOffset_[22] = 38.95f;
  laserOffset_[10] = 40.91f;
  laserOffset_[0] = 42.22f;
  laserOffset_[36] = 42.22f;
  laserOffset_[32] = 43.52f;
  laserOffset_[4] = 45.49f;
  laserOffset_[18] = 45.49f;
  laserOffset_[6] = 47.46f;
  laserOffset_[26] = 48.76f;
  laserOffset_[14] = 50.73f;
  laserOffset_[19] = 52.7f;
  laserOffset_[9] = 54.67f;

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the laser offset */
    elev_angle_map_[i] = pandar40p_elev_angle_map[i];
    horizatal_azimuth_offset_map_[i] =
        pandar40p_horizatal_azimuth_offset_map[i];
  }

  frame_id_ = frame_id;
  tz_second_ = tz * 3600;
}

Pandar40P_Internal::~Pandar40P_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int Pandar40P_Internal::LoadCorrectionFile(std::string correction_content) {
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  double azimuthOffset[LASER_COUNT];
  double elev_angle[LASER_COUNT];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    if (lineCounter++ >= LASER_COUNT) break;

    int lineId = 0;
    double elev, azimuth;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (lineId != lineCounter) {
      return -1;
    }

    elev_angle[lineId - 1] = elev;
    azimuthOffset[lineId - 1] = azimuth;
  }

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the laser offset */
    elev_angle_map_[i] = elev_angle[i];
    horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }

  return 0;
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void Pandar40P_Internal::ResetStartAngle(uint16_t start_angle) {
  start_angle_ = start_angle;
}

int Pandar40P_Internal::Start() {
  Stop();
  enable_lidar_recv_thr_ = true;
  enable_lidar_process_thr_ = true;
  lidar_process_thr_ = new boost::thread(
      boost::bind(&Pandar40P_Internal::ProcessLiarPacket, this));
  lidar_recv_thr_ =
      new boost::thread(boost::bind(&Pandar40P_Internal::RecvTask, this));
}

void Pandar40P_Internal::Stop() {
  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }
  return;
}

void Pandar40P_Internal::RecvTask() {
  int ret = 0;
  while (enable_lidar_recv_thr_) {
    PandarPacket pkt;
    int rc = input_->getPacket(&pkt);
    if (rc == -1) {
      continue;
    }

    if (pkt.size == GPS_PACKET_SIZE) {
      PandarGPS gpsMsg;
      ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
      if (ret == 0) {
        ProcessGps(gpsMsg);
      }
      continue;
    }

    PushLiDARData(pkt);
  }
}

void Pandar40P_Internal::ProcessLiarPacket() {
  double lastTimestamp = 0.0f;
  struct timespec ts;
  int ret = 0;

  boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());

  while (enable_lidar_process_thr_) {
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      std::cout << "get time error" << std::endl;
    }

    ts.tv_sec += 1;
    if (sem_timedwait(&lidar_sem_, &ts) == -1) {
      continue;
    }

    pthread_mutex_lock(&lidar_lock_);
    PandarPacket packet = lidar_packets_.front();
    lidar_packets_.pop_front();
    pthread_mutex_unlock(&lidar_lock_);

    if (packet.size == PACKET_SIZE) {
      Pandar40PPacket pkt;
      ret = ParseRawData(&pkt, packet.data, packet.size);
      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
        int azimuthGap = 0; /* To do */
        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) +
                       (36000 - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) -
                       static_cast<int>(last_azimuth_);
        }

        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            azimuthGap < 600 /* 6 degree*/) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && outMsg->points.size() > 0) {
              pcl_callback_(outMsg, outMsg->points[0].timestamp);
              outMsg.reset(new PPointCloud());
            }
          }
        }
        CalcPointXYZIT(&pkt, i, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
      }
    } else {
      continue;
    }

    outMsg->header.frame_id = frame_id_;
    outMsg->height = 1;
  }
}

void Pandar40P_Internal::PushLiDARData(PandarPacket packet) {
  pthread_mutex_lock(&lidar_lock_);
  lidar_packets_.push_back(packet);
  sem_post(&lidar_sem_);
  pthread_mutex_unlock(&lidar_lock_);
}

void Pandar40P_Internal::ProcessGps(const PandarGPS &gpsMsg) {
  struct tm t;
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;

  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;

  // UTC's month start from 1, but mktime only accept month from 0.
  t.tm_mon = gpsMsg.month - 1;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  t.tm_year = gpsMsg.year + 100;
  t.tm_isdst = 0;

  if (gps_callback_) {
    gps_callback_(static_cast<double>(mktime(&t) + tz_second_));
  }
}

int Pandar40P_Internal::ParseRawData(Pandar40PPacket *packet,
                                     const uint8_t *buf, const int len) {
  if (len != PACKET_SIZE) {
    std::cout << "packet size mismatch Pandar40P_Internal " << len << ","
              << PACKET_SIZE << std::endl;
    return -1;
  }

  int index = 0;
  // 10 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    Pandar40PBlock &block = packet->blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x units
    for (int j = 0; j < LASER_COUNT; j++) {
      Pandar40PUnit &unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      // distance is M.
      unit.distance =
          (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      // TODO(Philip.Pi): Filtering wrong data for LiDAR.
      if ((unit.distance == 0x010101 && unit.intensity == 0x0101) ||
          unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }
      index += RAW_MEASURE_SIZE;
    }
  }
  index += RESERVE_SIZE;  // skip reserved bytes

  index += REVOLUTION_SIZE;

  packet->usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  packet->usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet->echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  // parse the UTC Time.

  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet->t.tm_year = (buf[index + 0] & 0xff) + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  packet->t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet->t.tm_mday = buf[index + 2] & 0xff;
  packet->t.tm_hour = buf[index + 3] & 0xff;
  packet->t.tm_min = buf[index + 4] & 0xff;
  packet->t.tm_sec = buf[index + 5] & 0xff;
  packet->t.tm_isdst = 0;

  return 0;
}

int Pandar40P_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf,
                                 const int size) {
  if (size != GPS_PACKET_SIZE) {
    return -1;
  }
  int index = 0;
  packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  packet->year =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  packet->month =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  packet->day =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  packet->second =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  packet->minute =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  packet->hour =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  packet->fineTime =
      (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
#ifdef DEBUG
  if (packet->year != 18) {
    printf("error gps\n");
    char str[128];
    int fd = open("/var/tmp/error_gps.txt", O_RDWR | O_CREAT, 0666);
    lseek(fd, 0, SEEK_END);
    int i = 0;
    for (i = 0; i < 512; i++) {
      snprintf(str, "%02x ", recvbuf[i], 127);
      write(fd, str, strlen(str));
    }
    write(fd, "\n", 1);
    close(fd);
  }
#endif
  return 0;
}

void Pandar40P_Internal::CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                                        boost::shared_ptr<PPointCloud> cld) {
  Pandar40PBlock *block = &pkt->blocks[blockid];

  double unix_second = static_cast<double>(mktime(&pkt->t) + tz_second_);

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the units in a block */
    Pandar40PUnit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.5 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance =
        unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
    point.x = static_cast<float>(
        xyDistance *
        sinf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.y = static_cast<float>(
        xyDistance *
        cosf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance *
                                 sinf(degreeToRadian(elev_angle_map_[i])));

    // point.x = static_cast<float>(xyDistance *
    // sin_lookup_table_[xylookup_id]);
    // point.y = static_cast<float>(xyDistance *
    // cos_lookup_table_[xylookup_id]);
    // point.z = static_cast<float>(unit.distance *
    // sin_lookup_table_[zlookup_id]);

    point.intensity = unit.intensity;

    point.timestamp =
        unix_second + (static_cast<double>(pkt->usec)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp =
          point.timestamp -
          (static_cast<double>(blockOffset_[blockid / 2] + laserOffset_[i]) /
           1000000.0f);
    } else {
      point.timestamp =
          point.timestamp -
          (static_cast<double>(blockOffset_[blockid] + laserOffset_[i]) /
           1000000.0f);
    }

    point.ring = i;

    cld->push_back(point);
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
