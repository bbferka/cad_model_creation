#include<iostream>
#include <string>
#include<math.h>
#include<pcl/common/transforms.h>
#include<pcl/io/file_io.h>
#include<pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <fstream>
#include <opencv/cv.h>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Core>

#include <eigen3/Eigen/src/Core/util/Constants.h>
std::string model_filename_;
std::string scene_filename_;
bool use_extern_matrix_ , use_inverse_;
typedef pcl::PointXYZRGBA PointType;

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " input_cloud.pcd [out_name.pcd] [Options]" << std::endl << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     --mat:     YAML file with transformation matrix." << std::endl;
    std::cout << "     -i:     Use inverse." << std::endl;
}

cv::Mat readFromXMLFile(std::string filePath)throw(cv::Exception)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    cv::Mat MCamera;
    fs["transfomation_matrix"] >> MCamera;
    if (MCamera.cols!=4 || MCamera.rows!=4)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
    return MCamera;

}

int main (int argc, char** argv)
{
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }


    //Model & scene filenames
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () < 1)
    {
        std::cout << "Filenames missing.\n";
        showHelp (argv[0]);
        exit (-1);
    }
    if (filenames.size () == 2)
    {
        model_filename_ = argv[filenames[0]];
        scene_filename_ = argv[filenames[1]];
    }
    else
    {
        model_filename_ = argv[filenames[0]];
        scene_filename_ = "default.pcd";
    }


    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr out (new pcl::PointCloud<PointType> ());

    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        showHelp (argv[0]);
        return (-1);
    }

    if (pcl::console::find_switch (argc, argv, "--mat"))
    {
        use_extern_matrix_ = true;
    }
    if (pcl::console::find_switch (argc, argv, "-i"))
    {
        use_inverse_ = true;
    }
    Eigen::Matrix4d transformata_origine,aux;
    if(!use_extern_matrix_)
    {
        double param,angle,trans;
        trans = 0.7815; //TODO : Load values from planne_model.xml
        param = 136.997;
        angle = (param*M_PI/180); //transfor degree to radian
        transformata_origine <<
                1.,         0. ,           0. ,     -0.02861,
                0. ,    cos (angle),  -sin(angle),  0.04224,
                0. ,    sin (angle),   cos(angle), trans,
                0. ,        0. ,           0. ,      1. ;
    }
    else
    {
        std::string path_to_matrix;
        pcl::console::parse_argument (argc, argv, "--mat", path_to_matrix);
        std::cout<<"Using external file: "<<path_to_matrix<<std::endl;
        cv::Mat_<double> a = readFromXMLFile(path_to_matrix);
        cv::cv2eigen(a,transformata_origine);
    }
    if(use_inverse_)
    {
        aux=transformata_origine.inverse();
        transformata_origine=aux;
    }

    pcl::transformPointCloud(*model,*out,transformata_origine);
    pcl::io::savePCDFileASCII(scene_filename_,*out);

    return 0;

}
