// Copyright 2019 Justin Zhang
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <sstream>

typedef pcl::PointXYZ PointOct;

class PclOctree {
 public:
  pcl::io::OctreePointCloudCompression<PointOct>* cloud_encoder_;
  pcl::io::OctreePointCloudCompression<PointOct>* cloud_decoder_;
  pcl::PointCloud<PointOct>::Ptr cloud_src_;
  std::stringstream compressed_data_;
  PclOctree(const pcl::PointCloud<PointOct>::Ptr cloud_in,
            pcl::io::compression_Profiles_e compression_profile);
  std::stringstream encodeCloud();
  pcl::PointCloud<PointOct>::Ptr decodeCloud();
};
