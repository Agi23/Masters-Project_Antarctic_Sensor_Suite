//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <time.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include "lvx_file.h"
#include "third_party/rapidxml/rapidxml.hpp"
#include "third_party/rapidxml/rapidxml_utils.hpp"

#define WRITE_BUFFER_LEN 1024 * 1024
#define MAGIC_CODE       (0xac0ea767)
#define RAW_POINT_NUM     100
#define SINGLE_POINT_NUM  96
#define DUAL_POINT_NUM    48
#define TRIPLE_POINT_NUM  30
#define IMU_POINT_NUM     1
#define M_PI             3.14159265358979323846

namespace {
constexpr float kMmToMeter = 0.001f;
constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

inline void SphericalToCartesian(uint32_t depth_mm, uint16_t theta_centideg, uint16_t phi_centideg, PcdPoint &point) {
  const float radius = static_cast<float>(depth_mm) * kMmToMeter;
  const float theta = static_cast<float>(theta_centideg) * 0.01f * kDegToRad;
  const float phi = static_cast<float>(phi_centideg) * 0.01f * kDegToRad;
  const float xy = radius * std::sin(theta);
  point.x = xy * std::cos(phi);
  point.y = xy * std::sin(phi);
  point.z = radius * std::cos(theta);
}

inline void AppendCartesianPoint(int32_t x_mm, int32_t y_mm, int32_t z_mm, uint8_t reflectivity,
                                 std::vector<PcdPoint> &points) {
  PcdPoint p;
  p.x = static_cast<float>(x_mm) * kMmToMeter;
  p.y = static_cast<float>(y_mm) * kMmToMeter;
  p.z = static_cast<float>(z_mm) * kMmToMeter;
  p.intensity = static_cast<float>(reflectivity);
  points.push_back(p);
}

void AppendPointsFromPacket(const LvxBasePackDetail &packet, std::vector<PcdPoint> &points) {
  switch (packet.data_type) {
    case PointDataType::kCartesian: {
      auto *raw = reinterpret_cast<const LivoxRawPoint *>(packet.raw_point);
      for (int i = 0; i < RAW_POINT_NUM; ++i) {
        AppendCartesianPoint(raw[i].x, raw[i].y, raw[i].z, raw[i].reflectivity, points);
      }
      break;
    }
    case PointDataType::kSpherical: {
      auto *raw = reinterpret_cast<const LivoxSpherPoint *>(packet.raw_point);
      for (int i = 0; i < RAW_POINT_NUM; ++i) {
        PcdPoint p;
        SphericalToCartesian(raw[i].depth, raw[i].theta, raw[i].phi, p);
        p.intensity = static_cast<float>(raw[i].reflectivity);
        points.push_back(p);
      }
      break;
    }
    case PointDataType::kExtendCartesian: {
      auto *raw = reinterpret_cast<const LivoxExtendRawPoint *>(packet.raw_point);
      for (int i = 0; i < SINGLE_POINT_NUM; ++i) {
        AppendCartesianPoint(raw[i].x, raw[i].y, raw[i].z, raw[i].reflectivity, points);
      }
      break;
    }
    case PointDataType::kExtendSpherical: {
      auto *raw = reinterpret_cast<const LivoxExtendSpherPoint *>(packet.raw_point);
      for (int i = 0; i < SINGLE_POINT_NUM; ++i) {
        PcdPoint p;
        SphericalToCartesian(raw[i].depth, raw[i].theta, raw[i].phi, p);
        p.intensity = static_cast<float>(raw[i].reflectivity);
        points.push_back(p);
      }
      break;
    }
    case PointDataType::kDualExtendCartesian: {
      auto *raw = reinterpret_cast<const LivoxDualExtendRawPoint *>(packet.raw_point);
      for (int i = 0; i < DUAL_POINT_NUM; ++i) {
        AppendCartesianPoint(raw[i].x1, raw[i].y1, raw[i].z1, raw[i].reflectivity1, points);
        AppendCartesianPoint(raw[i].x2, raw[i].y2, raw[i].z2, raw[i].reflectivity2, points);
      }
      break;
    }
    case PointDataType::kDualExtendSpherical: {
      auto *raw = reinterpret_cast<const LivoxDualExtendSpherPoint *>(packet.raw_point);
      for (int i = 0; i < DUAL_POINT_NUM; ++i) {
        PcdPoint p1;
        SphericalToCartesian(raw[i].depth1, raw[i].theta, raw[i].phi, p1);
        p1.intensity = static_cast<float>(raw[i].reflectivity1);
        points.push_back(p1);
        PcdPoint p2;
        SphericalToCartesian(raw[i].depth2, raw[i].theta, raw[i].phi, p2);
        p2.intensity = static_cast<float>(raw[i].reflectivity2);
        points.push_back(p2);
      }
      break;
    }
    case PointDataType::kTripleExtendCartesian: {
      auto *raw = reinterpret_cast<const LivoxTripleExtendRawPoint *>(packet.raw_point);
      for (int i = 0; i < TRIPLE_POINT_NUM; ++i) {
        AppendCartesianPoint(raw[i].x1, raw[i].y1, raw[i].z1, raw[i].reflectivity1, points);
        AppendCartesianPoint(raw[i].x2, raw[i].y2, raw[i].z2, raw[i].reflectivity2, points);
        AppendCartesianPoint(raw[i].x3, raw[i].y3, raw[i].z3, raw[i].reflectivity3, points);
      }
      break;
    }
    case PointDataType::kTripleExtendSpherical: {
      auto *raw = reinterpret_cast<const LivoxTripleExtendSpherPoint *>(packet.raw_point);
      for (int i = 0; i < TRIPLE_POINT_NUM; ++i) {
        PcdPoint p1;
        SphericalToCartesian(raw[i].depth1, raw[i].theta, raw[i].phi, p1);
        p1.intensity = static_cast<float>(raw[i].reflectivity1);
        points.push_back(p1);
        PcdPoint p2;
        SphericalToCartesian(raw[i].depth2, raw[i].theta, raw[i].phi, p2);
        p2.intensity = static_cast<float>(raw[i].reflectivity2);
        points.push_back(p2);
        PcdPoint p3;
        SphericalToCartesian(raw[i].depth3, raw[i].theta, raw[i].phi, p3);
        p3.intensity = static_cast<float>(raw[i].reflectivity3);
        points.push_back(p3);
      }
      break;
    }
    case PointDataType::kImu:
    default:
      break;
  }
}
}  // namespace

LvxFileHandle::LvxFileHandle()
    : cur_frame_index_(0),
      cur_offset_(0),
      frame_duration_(kDefaultFrameDurationTime),
      pcd_base_name_(),
      pcd_frame_index_(0) {
}

bool LvxFileHandle::InitLvxFile() {
  time_t curtime = time(nullptr);
  char filename[30] = { 0 };

  tm* local_time = localtime(&curtime);
  strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx", local_time);
  lvx_file_.open(filename, std::ios::out | std::ios::binary);

  if (!lvx_file_.is_open()) {
    return false;
  }
  return true;
}

bool LvxFileHandle::InitPcdFile() {
  time_t curtime = time(nullptr);
  char basename[30] = {0};
  tm *local_time = localtime(&curtime);
  strftime(basename, sizeof(basename), "%Y-%m-%d_%H-%M-%S", local_time);
  pcd_base_name_ = basename;
  pcd_frame_index_ = 0;
  return true;
}

void LvxFileHandle::InitLvxFileHeader() {
  LvxFilePublicHeader lvx_file_public_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = "livox_tech";
  memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());

  lvx_file_public_header.version[0] = 1;
  lvx_file_public_header.version[1] = 1;
  lvx_file_public_header.version[2] = 0;
  lvx_file_public_header.version[3] = 0;

  lvx_file_public_header.magic_code = MAGIC_CODE;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
  cur_offset_ += sizeof(LvxFilePublicHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  LvxFilePrivateHeader lvx_file_private_header = { 0 };
  lvx_file_private_header.frame_duration = frame_duration_;
  lvx_file_private_header.device_count = device_count;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
  cur_offset_ += sizeof(LvxFilePrivateHeader);

  for (int i = 0; i < device_count; i++) {
    memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    cur_offset_ += sizeof(LvxDeviceInfo);
  }

  lvx_file_.write((char *)write_buffer.get(), cur_offset_);
}

void LvxFileHandle::SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {
  uint64_t cur_pos = 0;
  FrameHeader frame_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + sizeof(FrameHeader);
  auto iterator = point_packet_list_temp.begin();
  for (; iterator != point_packet_list_temp.end(); iterator++) {
    frame_header.next_offset += iterator->pack_size;
  }

  frame_header.frame_index = cur_frame_index_;

  memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(FrameHeader));
  cur_pos += sizeof(FrameHeader);

  auto iter = point_packet_list_temp.begin();
  for (; iter != point_packet_list_temp.end(); iter++) {
    if (cur_pos + iter->pack_size >= WRITE_BUFFER_LEN) {
      lvx_file_.write((char*)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
      cur_pos += iter->pack_size;
    }
    else {
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
      cur_pos += iter->pack_size;
    }
  }
  lvx_file_.write((char*)write_buffer.get(), cur_pos);

  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileHandle::SaveFrameToPcdFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {
  std::vector<PcdPoint> points;
  points.reserve(point_packet_list_temp.size() * RAW_POINT_NUM);

  for (const auto &packet : point_packet_list_temp) {
    AppendPointsFromPacket(packet, points);
  }

  std::ostringstream filename;
  filename << pcd_base_name_ << "_frame_" << pcd_frame_index_ << ".pcd";
  std::ofstream pcd_file(filename.str(), std::ios::out);
  if (!pcd_file.is_open()) {
    return;
  }

  pcd_file << "# .PCD v0.7 - Point Cloud Data file format\n";
  pcd_file << "VERSION 0.7\n";
  pcd_file << "FIELDS x y z intensity\n";
  pcd_file << "SIZE 4 4 4 4\n";
  pcd_file << "TYPE F F F F\n";
  pcd_file << "COUNT 1 1 1 1\n";
  pcd_file << "WIDTH " << points.size() << "\n";
  pcd_file << "HEIGHT 1\n";
  pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
  pcd_file << "POINTS " << points.size() << "\n";
  pcd_file << "DATA ascii\n";

  for (const auto &point : points) {
    pcd_file << point.x << " " << point.y << " " << point.z << " " << point.intensity << "\n";
  }
  pcd_frame_index_++;
}

void LvxFileHandle::CloseLvxFile() {
  if (lvx_file_.is_open())
    lvx_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet) {
  packet.version = data->version;
  packet.port_id = data->slot;
  packet.lidar_index = data->id;
  packet.rsvd = data->rsvd;
  packet.error_code = data->err_code;
  packet.timestamp_type = data->timestamp_type;
  packet.data_type = data->data_type;
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
  switch (packet.data_type) {
    case PointDataType::kCartesian:
       packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - \
          sizeof(packet.pack_size) + RAW_POINT_NUM*sizeof(LivoxRawPoint);
      memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxRawPoint));
      break;
    case PointDataType::kSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +RAW_POINT_NUM*sizeof(LivoxSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, RAW_POINT_NUM*sizeof(LivoxSpherPoint));
      break;
    case PointDataType::kExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +SINGLE_POINT_NUM*sizeof(LivoxExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, SINGLE_POINT_NUM*sizeof(LivoxExtendRawPoint));
      break;
    case PointDataType::kExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +SINGLE_POINT_NUM*sizeof(LivoxExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, SINGLE_POINT_NUM*sizeof(LivoxExtendSpherPoint));
      break;
    case PointDataType::kDualExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +DUAL_POINT_NUM*sizeof(LivoxDualExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, DUAL_POINT_NUM*sizeof(LivoxDualExtendRawPoint));
      break;
    case PointDataType::kDualExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +DUAL_POINT_NUM*sizeof(LivoxDualExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, DUAL_POINT_NUM*sizeof(LivoxDualExtendSpherPoint));
      break;
    case PointDataType::kImu :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +IMU_POINT_NUM*sizeof(LivoxImuPoint);
      memcpy(packet.raw_point,(void *)data->data, IMU_POINT_NUM*sizeof(LivoxImuPoint));
      break;
    case PointDataType::kTripleExtendCartesian :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendRawPoint);
      memcpy(packet.raw_point,(void *)data->data, TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendRawPoint));
      break;
    case PointDataType::kTripleExtendSpherical :
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point)- sizeof(packet.pack_size) +TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendSpherPoint);
      memcpy(packet.raw_point,(void *)data->data, TRIPLE_POINT_NUM*sizeof(LivoxTripleExtendSpherPoint));
      break;
    default:
      break;
  }
}

void ParseExtrinsicXml(DeviceItem &item, LvxDeviceInfo &info) {
  rapidxml::file<> extrinsic_param("extrinsic.xml");
  rapidxml::xml_document<> doc;
  doc.parse<0>(extrinsic_param.data());
  rapidxml::xml_node<>* root = doc.first_node();
  if ("Livox" == (std::string)root->name()) {
    for (rapidxml::xml_node<>* device = root->first_node(); device; device = device->next_sibling()) {
      if ("Device" == (std::string)device->name() && (strncmp(item.info.broadcast_code, device->value(), kBroadcastCodeSize) == 0)) {
        memcpy(info.lidar_broadcast_code, device->value(), kBroadcastCodeSize);
        memset(info.hub_broadcast_code, 0, kBroadcastCodeSize);
        info.device_type = item.info.type;
        info.device_index = item.handle;
        for (rapidxml::xml_attribute<>* param = device->first_attribute(); param; param = param->next_attribute()) {
          if ("roll" == (std::string)param->name()) info.roll = static_cast<float>(atof(param->value()));
          if ("pitch" == (std::string)param->name()) info.pitch = static_cast<float>(atof(param->value()));
          if ("yaw" == (std::string)param->name()) info.yaw = static_cast<float>(atof(param->value()));
          if ("x" == (std::string)param->name()) info.x = static_cast<float>(atof(param->value()));
          if ("y" == (std::string)param->name()) info.y = static_cast<float>(atof(param->value()));
          if ("z" == (std::string)param->name()) info.z = static_cast<float>(atof(param->value()));
        }
      }
    }
  }
}
