// Copyright 2019 Justin Zhang
#include <chrono>
#include "pcl_octree.hpp"

typedef std::chrono::milliseconds TimeUnit;

PclOctree::PclOctree(const pcl::PointCloud<PointOct>::Ptr cloud_in,
                     pcl::io::compression_Profiles_e compression_profile) {
  cloud_src_ = cloud_in;
  cloud_encoder_ = new pcl::io::OctreePointCloudCompression<PointOct>
                   (compression_profile, true);
  cloud_decoder_ = new pcl::io::OctreePointCloudCompression<PointOct> ();
}

std::stringstream PclOctree::encodeCloud() {
  std::stringstream compressed_data;
  cloud_encoder_->encodePointCloud(cloud_src_, compressed_data);
  compressed_data_ << compressed_data.rdbuf();
  return compressed_data;
}

pcl::PointCloud<PointOct>::Ptr PclOctree::decodeCloud() {
  pcl::PointCloud<PointOct>::Ptr cloud_out(new pcl::PointCloud<PointOct> ());
  cloud_decoder_->decodePointCloud(compressed_data_, cloud_out);
  return cloud_out;
}

main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(
                 new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<PointOct>::Ptr cloud(new pcl::PointCloud<PointOct>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud_i) == -1) {
    std::cout << "Couldn't read file: " << argv[1] << std::endl;
    return (-1);
  }
  pcl::copyPointCloud(*cloud_i, *cloud);
  pcl::io::savePCDFileASCII("../pcd/cloud_src.pcd", *cloud);
  PclOctree pcl_octree_low(cloud,
                          pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR);
  PclOctree pcl_octree_mid(cloud,
                          pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR);
  PclOctree pcl_octree_high(cloud,
                          pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR);
  std::cout << "\n********** Encode Low Res PointCloud **********" << std::endl;
  TimeUnit ms_a = std::chrono::duration_cast<TimeUnit>(
                  std::chrono::system_clock::now().time_since_epoch());
  pcl_octree_low.encodeCloud();
  TimeUnit ms_b = std::chrono::duration_cast<TimeUnit>(
                  std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Encode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;
  std::cout << "\n********** Encode Mid Res PointCloud **********" << std::endl;
  ms_a = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  pcl_octree_mid.encodeCloud();
  ms_b = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Encode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;
  std::cout << "\n********** Encode High Res PointCloud **********" << std::endl;
  ms_a = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  pcl_octree_high.encodeCloud();
  ms_b = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Encode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;

  // Decode
  std::cout << "\n********** Decode Low Res PointCloud **********" << std::endl;
  ms_a = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  auto decoded_pc = pcl_octree_low.decodeCloud();
  ms_b = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Decode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;
  pcl::io::savePCDFileASCII("../pcd/low_decode_cloud.pcd", *decoded_pc);

  std::cout << "\n********** Decode Mid Res PointCloud **********" << std::endl;
  ms_a = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  decoded_pc = pcl_octree_mid.decodeCloud();
  ms_b = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Decode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;
  pcl::io::savePCDFileASCII("../pcd/mid_decode_cloud.pcd", *decoded_pc);

  std::cout << "\n********** Decode High Res PointCloud **********" << std::endl;
  ms_a = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  decoded_pc = pcl_octree_high.decodeCloud();
  ms_b = std::chrono::duration_cast<TimeUnit>(
                std::chrono::system_clock::now().time_since_epoch());
  std::cout << "Decode Time: " << (ms_b - ms_a).count() << " ms" << std::endl;
  pcl::io::savePCDFileASCII("../pcd/high_decode_cloud.pcd", *decoded_pc);
  return (0);
}
