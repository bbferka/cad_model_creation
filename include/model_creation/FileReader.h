#ifndef __FILEREADER_H__
#define __FILEREADER_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PCD
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::string f_name;
  PCD() : cloud (new pcl::PointCloud<pcl::PointXYZRGB>) {};
};

 class FileReader {
 public:
     std::vector<PCD, Eigen::aligned_allocator<PCD> > pcd_vector;
     std::string path;
      FileReader(std::string f_path);
      ~FileReader();
      void loadData() ;
      std::string size;

};

#endif
