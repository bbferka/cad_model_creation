
#include<iostream>
#include<model_creation/FileReader.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

FileReader::FileReader(std::string f_path){
   this->path=f_path;
}
FileReader::~FileReader()
{

}
bool sortByName(const PCD &lhs, const PCD &rhs) { return lhs.f_name < rhs.f_name; }

void FileReader::loadData(){
    std::cout <<"loading data from: " <<FileReader::path <<std::endl;
    //std::string extension (".pcd");
    std::vector< std::string > filenames;
      boost::filesystem3::path dir_path(FileReader::path); //path to directory where are stored our .pcd files
      boost::filesystem3::directory_iterator end_it; // end iterator
      // loop through each file in the directory
      for(boost::filesystem3::directory_iterator it(dir_path); it != end_it; ++it) {
        // if it's not a directory and its extension is .pcd [2]
        if( !boost::filesystem3::is_directory(it->status()) && boost::filesystem3::extension(it->path()) == ".pcd" ) {
          // store filename for later use
          filenames.push_back( it->path().string() );

          PCD m;
          m.f_name = it->path().string();
          pcl::io::loadPCDFile (it->path().string(), *m.cloud);
          //remove NAN points from the cloud
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
          FileReader::pcd_vector.push_back(m);

          //std::cout<< it->path().string() <<std::endl;
        }

      }
      std::cout<<"Size of pcd vector:" <<FileReader::pcd_vector.size()<<std::endl;
      std::sort(FileReader::pcd_vector.begin(),FileReader::pcd_vector.end(),sortByName);
      for(int i=0;i<FileReader::pcd_vector.size();i++)
      {
          PCD temp =FileReader::pcd_vector[i];
          std::cout<<temp.f_name<<std::endl;
      }

}


