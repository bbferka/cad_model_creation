#include<iostream>
#include <string>
#include<model_creation/FileReader.h>
#include<model_creation/Estimate_Transformation.h>
#include<printf.h>
#include<pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<math.h>
#include<pcl/common/transforms.h>
#include<pcl/io/file_io.h>
#include<pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_representation.h>
#include <pcl/surface/mls.h>
#include <cstdio>
#include <ctime>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
#include <opencv/cv.h>
#include <pcl/console/parse.h>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_representation.h>
#include <thread>
#include <chrono>
#include <future>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#define PATH_TO_PLANE  "/home/stefan/Documents/Licenta_Image/result_plane_p_0.pcd"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

std::string path_to_seg_folder_;
std::string path_to_transformation_matrix_;
std::string path_to_config_file_;
float object_height_,icp_treeshold_,downsample_threshold_,smothing_radius_,outliner_radius_;
bool is_object_upside_down_;
int icp_nr_iterations_,outliner_neighbors_;
std::clock_t start;
double duration;

namespace patch
{
template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
}


pcl::PointCloud<pcl::PointXYZRGBNormal> normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
    std::cout<<"Start estimating normals:"<<std::endl;
   // start = std::clock();
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    tree->setInputCloud (cloud_filtered);
    n.setInputCloud (cloud_filtered);
    n.setSearchMethod (tree);
    n.setRadiusSearch(0.01); //0.005
    //n.setKSearch (100);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    //pcl::io::savePCDFile ("cloud_with_normals.pcd", *cloud_with_normals,true);
    //duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End estimating normals, pcd size:"<<(*cloud_with_normals).points.size()<<" , in "<<duration<<" sec."<<std::endl;

    return *cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZRGB> downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c) {
    std::cout<<"Start downsampling pointCloud with size:"<<(*cloud_c).points.size()<<" threshold:"<<downsample_threshold_<<std::endl;
    //start = std::clock();
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (downsample_threshold_, downsample_threshold_, downsample_threshold_);
    sor.setInputCloud (cloud_c);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_downsampled_ptr);
    //pcl::io::savePCDFile ("Concatenat2.pcd", *cloud_downsampled_ptr,true);
    //duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End downsampling, pcd size:"<<(*cloud_downsampled_ptr).points.size()<<" , in "<<duration<<" sec."<<std::endl;

    return *cloud_downsampled_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB> outliner_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr){
    std::cout<<"Start removing outliners, before pointCloud had size:"<<(*cloud_downsampled_ptr).points.size()<<std::endl;
    //start = std::clock();
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
    //    sor2.setInputCloud (cloud_downsampled_ptr);
    //    sor2.setMeanK (50);
    //    sor2.setStddevMulThresh (6);
    //    sor2.filter (*cloud_filtered);
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud_downsampled_ptr);
    outrem.setRadiusSearch(outliner_radius_);
    outrem.setMinNeighborsInRadius (outliner_neighbors_);
    // apply filter
    outrem.filter (*cloud_filtered);

    //pcl::io::savePCDFile ("Filtered.pcd", *cloud_filtered,true);
    //duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End filtering, pcd size:"<<(*cloud_filtered).points.size()<<" , in "<<duration<<" sec."<<std::endl;

    return *cloud_filtered;
}

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform){
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    src = cloud_src;
    tgt = cloud_tgt;
    //
    // Align
    pcl::IterativeClosestPoint<PointT,PointT > reg;
    //reg.setTransformationEpsilon (1e-6); //1e-6
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (icp_treeshold_); //LL=0.05
    reg.setTransformationEpsilon (1e-6); //LL=1e-6

    //      reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
    reg.setInputSource (src);
    reg.setInputTarget (tgt);

    Eigen::Matrix4f prev;
    //PointCloud::Ptr reg_result = src;
    //reg.setMaximumIterations (5);
    reg.align (*src);
    for (int i = 1; i <icp_nr_iterations_; ++i) //LL<350 //30Jul-30
    {
        if (reg.hasConverged()) {
            PCL_INFO ("Iteration Nr. %d.\n", i);
           reg.align (*src);

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (fabs ((reg.getLastIncrementalTransformation () - prev).sum()) < reg.getTransformationEpsilon ())
                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.0002);

            prev = reg.getLastIncrementalTransformation ();
            //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
        }
        else i=200; //LL=350
    }

    //add the source to the transformed target
    *output =*src+*tgt;

}

//void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
//{
//  //
//  // Downsample for consistency and speed
//  // \note enable this for large datasets
//  PointCloud::Ptr src (new PointCloud);
//  PointCloud::Ptr tgt (new PointCloud);

//    src = cloud_src;
//    tgt = cloud_tgt;



//  // Compute surface normals and curvature
//  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
//  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

// *points_with_normals_src=normal_estimation(src);
// *points_with_normals_tgt=normal_estimation(tgt);

//  // Instantiate our custom point representation (defined above) ...
//    MyPointRepresentation point_representation;
//    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
//    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
//    point_representation.setRescaleValues (alpha);

//  //
//  // Align
//  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
//  reg.setTransformationEpsilon (1e-6);
//  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
//  // Note: adjust this based on the size of your datasets
//  reg.setMaxCorrespondenceDistance (Icp_treeshold_); //0.005
//  // Set the point representation
//  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

//  reg.setInputSource (points_with_normals_src);
//  reg.setInputTarget (points_with_normals_tgt);
//  //
//  // Run the same optimization in a loop and visualize the results
//  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
//  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
//  reg.setMaximumIterations (1);
//  for (int i = 0; i < 50; ++i)
//  {
//    PCL_INFO ("Iteration Nr. %d.\n", i);

//    // save cloud for visualization purpose
//    points_with_normals_src = reg_result;

//    // Estimate
//    reg.setInputSource (points_with_normals_src);
//    reg.align (*reg_result);

//        //accumulate transformation between each Iteration
//    Ti = reg.getFinalTransformation () * Ti;

//        //if the difference between this transformation and the previous one
//        //is smaller than the threshold, refine the process by reducing
//        //the maximal correspondence distance
//    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
//      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.0002);

//    prev = reg.getLastIncrementalTransformation ();

//  }

//    //
//  // Get the transformation from target to source
//  targetToSource = Ti.inverse();
//  // Transform target back in source frame

//  pcl::transformPointCloud (*cloud_tgt, *tgt, targetToSource);

//  //add the source to the transformed target
//  *output =*cloud_src+*tgt;

//  //final_transform = targetToSource;
// }

pcl::PointCloud<pcl::PointXYZRGBNormal> smoothing_for_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
    std::cout<<"Start smoothing"<<std::endl;
    start = std::clock();
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls ;

    mls.setComputeNormals (true);
    // Set parametersr
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (0.003);
    //    mls.setPolynomialOrder(2);
    //    mls.setSqrGaussParam(0.0025);
    // Reconstruct
    mls.process (*mls_points);
    // Save output
    pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End smoothing, pcd size:"<<(*mls_points).points.size()<<" , in "<<duration<<" sec."<<std::endl;

    return *mls_points;
}

pcl::PointCloud<pcl::PointXYZRGB> smoothing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
    std::cout<<"Start smoothing"<<std::endl;
    start = std::clock();
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls ;

    mls.setComputeNormals (false);
    // Set parametersr
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (smothing_radius_);
    mls.setPolynomialOrder(2);
    mls.setSqrGaussParam(0.005);
    // Reconstruct
    mls.process (*mls_points);
    // Save output
    pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End smoothing, pcd size:"<<(*mls_points).points.size()<<" , in "<<duration<<" sec."<<std::endl;

    return *mls_points;
}

cv::Mat readFromXMLFile_M(std::string filePath)throw(cv::Exception)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    cv::Mat MCamera;
    fs["transfomation_matrix"] >> MCamera;
    if (MCamera.cols!=4 || MCamera.rows!=4)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
    return MCamera;

}
//pcl::PointCloud<pcl::PointXYZRGB> registration (FileReader *fl) {
//    std::cout<<"Starting registration"<<std::endl;
//    start = std::clock();
//    Eigen::Matrix4d transformata_origine ,rot_z,transformata_finala,translatie,rotatie_flip,rot_z_correct,translatie2;
//    double param,angle,trans;


//    //    trans = 0.795; //TODO : Load values from planne_model.xml
//    //   // param = 131.115; //pentru modelele mai vechi
//    //     param = 133.5;

//    trans = 0.79; //TODO : Load values from planne_model.xml
//    param = 133.9;

//    angle = (param*M_PI/180); //transfor degree to radian
//    // rot(x) + trans(y):
//    //    transformata_origine <<   1,         0 ,           0 ,      0,
//    //                              0,    cos (angle),  -sin(angle),  0,
//    //                              0,    sin (angle),   cos(angle), trans,
//    //                              0 ,         0 ,          0 ,     1 ;
//    //    transformata_origine <<   0.2754769, 0.95628476, 0.098142214,   0.0347831,
//    //                              0.66575259, -0.11613788, -0.73707902,  0.0355636,
//    //                              -0.69345939, 0.26838666, -0.6686424,   0.793487 ,
//    //                                0 ,            0 ,          0 ,          1 ;

//    //    Estimate_Transformation * estimator = new Estimate_Transformation(PATH_TO_PLANE);
//    //   transformata_origine = estimator->get_transformation();

//    //std::string path_to_matrix="/home/stefan/camera_calib_out/test_param.yml";
//    std::string path_to_matrix=path_to_transformation_matrix_;
//    //pcl::console::parse_argument (argc, argv, "--mat", path_to_matrix);
//    std::cout<<"Using external file: "<<path_to_matrix<<std::endl;
//    cv::Mat_<double> a = readFromXMLFile_M(path_to_matrix);
//    cv::cv2eigen(a,transformata_origine);

//    //    translatie <<  1,  0,  0 ,  -0.215194,
//    //                   0,  1,  0 ,  -0.192863 ,
//    //                   0 , 0 , 1 ,  0.965344,
//    //                   0 , 0 , 0 ,  1 ;

//    translatie <<  1,  0,  0 , 0,
//            0,  1,  0 , 0 ,
//            0 , 0 , 1 , -object_height_/2,
//            0 , 0 , 0 , 1 ;
//    translatie2 <<  1,  0,  0 , 0,
//            0,  1,  0 , 0 ,
//            0 , 0 , 1 , object_height_/2,
//            0 , 0 , 0 , 1 ;

//    param = -180;
//    angle = (param*M_PI/180); //transfor degree to radian
//    rotatie_flip<< cos (angle),      0 ,  sin (angle),    0 ,
//            0,            1 ,          0 ,     0 ,
//            -sin (angle),     0,   cos(angle),     0 ,
//            0 ,           0 ,          0 ,     1 ;


//    //rot(z) with 15 degree:
//    param = 15;
//    angle = (param*M_PI/180); //transform degree to radian
//    rot_z << cos (angle),  -sin(angle),  0 , 0 ,
//            sin (angle),   cos(angle),  0 , 0 ,
//            0 ,         0 ,       1 , 0 ,
//            0 ,         0 ,       0 , 1 ;

//    param = -180;
//    angle = (param*M_PI/180); //transform degree to radian
//    rot_z_correct << cos (angle),  -sin(angle),  0 , 0 ,
//            sin (angle),   cos(angle),  0 , 0 ,
//            0 ,         0 ,       1 , 0 ,
//            0 ,         0 ,       0 , 1 ;

//    PCD temp =fl->pcd_vector[0];
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_vector_transform[23];
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, cloud_transformed;

//    cloud_in=temp.cloud;
//    cloud_transformed=cloud_in;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ), cloud_before(new pcl::PointCloud<pcl::PointXYZRGB> ), cloud_final (new pcl::PointCloud<pcl::PointXYZRGB> ),result (new pcl::PointCloud<pcl::PointXYZRGB> );

//    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

//    for(int i=0; i<12;i++)
//    {
//        temp =fl->pcd_vector[i];
//        cloud_in=temp.cloud;
//        if(i==0)
//        {
//            //transformata_finala=transformata_origine;
//            transformata_finala=transformata_origine.inverse();
//            //transformata_finala=translatie.inverse();
//            pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
//            if(is_object_upside_down_){
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rotatie_flip);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie2);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rot_z_correct);
//            }
//        }
//        else

//        {

//            transformata_finala=rot_z.inverse()*transformata_finala;
//            pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
//            if(is_object_upside_down_)
//            {
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rotatie_flip);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie2);
//                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rot_z_correct);
//            }
//        }
//        //        pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
//        pcl::io::savePCDFile ("transformed"+patch::to_string(i)+".pcd", *cloud_transformed,true);
//        //        // pcd_vector_transform[i]=cloud_transformed;
//        *cloud_current=downsample(cloud_transformed);
//        //cloud_current=cloud_transformed;
//        *cloud_current=outliner_removal(cloud_current);
//         *cloud_current=smoothing(cloud_current);

//        if(i==0)
//            *cloud_final=*cloud_current;
//        if(i>0)
//        {
//            *cloud_before=*cloud_final;
//            //          pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
//            //            *cloud_final=*cloud_current + *cloud_before;
//            //             pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB> );
//            //           pairAlign (cloud_current, cloud_before, cloud_final, pairTransform);
//            pairAlign ( cloud_before,cloud_current, cloud_final, pairTransform); //(src,tgt,tmp)
//            //transform current pair into the global transform
//            // pcl::transformPointCloud (*temp, *result, GlobalTransform);
//            //update the global transform
//            // GlobalTransform = pairTransform * GlobalTransform;

//            *cloud_final=downsample(cloud_final);
//            //*cloud_final=outliner_removal(cloud_final);
//            //*cloud_final=smoothing(cloud_final);
//            pcl::io::savePCDFile ("inter"+patch::to_string(i)+".pcd", *cloud_final,true);
//        }

//        //*cloud_current=outliner_removal(cloud_current);
//        // boost::filesystem3::copy_file("transformetgtd"+patch::to_string(i)+".pcd",fl->path+"/transform/transformed"+patch::to_string(i)+".pcd",boost::filesystem::copy_option::overwrite_if_exists);
//    }

//    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//    std::cout<<"End registration in: "<<duration<<" sec."<<std::endl;
//    //*cloud_final=outliner_removal(cloud_final);
//    *cloud_final=smoothing(cloud_final);
//    pcl::io::savePCDFile ("Concatenat.pcd", *cloud_final,true);

//    return *cloud_final;
//}

pcl::PointCloud<pcl::PointXYZRGB> registration2 (FileReader *fl,int start_frame,int end_frame,bool backward) {
    std::cout<<"Starting registration"<<std::endl;
    start = std::clock();
    Eigen::Matrix4d transformata_origine ,rot_z,transformata_finala,translatie,rotatie_flip,rot_z_st,translatie2,rot_z_correct;
    double param,angle,trans;

    //std::string path_to_matrix="/home/stefan/camera_calib_out/test_param.yml";
    std::string path_to_matrix=path_to_transformation_matrix_;
    //pcl::console::parse_argument (argc, argv, "--mat", path_to_matrix);
    std::cout<<"Using external file: "<<path_to_matrix<<std::endl;
    cv::Mat_<double> a = readFromXMLFile_M(path_to_matrix);
    cv::cv2eigen(a,transformata_origine);
//    Estimate_Transformation * estimator = new Estimate_Transformation(PATH_TO_PLANE);
//    transformata_origine = estimator->get_transformation();
    translatie <<  1,  0,  0 , 0,
            0,  1,  0 , 0 ,
            0 , 0 , 1 , -object_height_/2,
            0 , 0 , 0 , 1 ;
    translatie2 <<  1,  0,  0 , 0,
            0,  1,  0 , 0 ,
            0 , 0 , 1 , object_height_/2,
            0 , 0 , 0 , 1 ;

    param = -180;
    angle = (param*M_PI/180); //transfor degree to radian
    rotatie_flip<< cos (angle),      0 ,  sin (angle),    0 ,
            0,            1 ,          0 ,     0 ,
            -sin (angle),     0,   cos(angle),     0 ,
            0 ,           0 ,          0 ,     1 ;


    //rot(z) with 15 degree:
    param = 15;
    angle = (param*M_PI/180); //transform degree to radian
    rot_z << cos (angle),  -sin(angle),  0 , 0 ,
            sin (angle),   cos(angle),  0 , 0 ,
            0 ,         0 ,       1 , 0 ,
            0 ,         0 ,       0 , 1 ;

    param = start_frame*15;
    angle = (param*M_PI/180); //transform degree to radian
    rot_z_st << cos (angle),  -sin(angle),  0 , 0 ,
            sin (angle),   cos(angle),  0 , 0 ,
            0 ,         0 ,       1 , 0 ,
            0 ,         0 ,       0 , 1 ;
    param = 180;
    angle = (param*M_PI/180); //transform degree to radian
    rot_z_correct << cos (angle),  -sin(angle),  0 , 0 ,
            sin (angle),   cos(angle),  0 , 0 ,
            0 ,         0 ,       1 , 0 ,
            0 ,         0 ,       0 , 1 ;
    PCD temp =fl->pcd_vector[start_frame];
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_vector_transform[23];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, cloud_transformed;

    cloud_in=temp.cloud;
    cloud_transformed=cloud_in;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ), cloud_previous(new pcl::PointCloud<pcl::PointXYZRGB> ), cloud_final (new pcl::PointCloud<pcl::PointXYZRGB> ),result (new pcl::PointCloud<pcl::PointXYZRGB> );
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
 pcl::PassThrough<pcl::PointXYZRGB> pass;
    for(int i=start_frame; i<end_frame&&i<fl->pcd_vector.size() ;i+=1)
    {
        temp =fl->pcd_vector[i];
        cloud_in=temp.cloud;
        if(i==start_frame)
        {
            //transformata_finala=transformata_origine;
            transformata_origine *=rot_z_st;
            transformata_finala=transformata_origine.inverse();
            //transformata_finala=translatie.inverse();
            pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
            if(is_object_upside_down_){
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rotatie_flip);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie2);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rot_z_correct);
//                pass.setInputCloud (cloud_transformed);
//                pass.setFilterFieldName ("z");
//                pass.setFilterLimits (0.0, object_height_/2);
//                //pass.setFilterLimitsNegative (true);
//                pass.filter (*cloud_transformed);

            }
        }
        else
        {
            transformata_finala=rot_z.inverse()*transformata_finala;
            pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
            if(is_object_upside_down_)
            {
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rotatie_flip);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,translatie2);
                pcl::transformPointCloud(*cloud_transformed,*cloud_transformed,rot_z_correct);
//                pass.setInputCloud (cloud_transformed);
//                pass.setFilterFieldName ("z");
//                pass.setFilterLimits (0.0, object_height_/2);
//                //pass.setFilterLimitsNegative (true);
//                pass.filter (*cloud_transformed);
            }
        }
        //        pcl::transformPointCloud(*cloud_in,*cloud_transformed,transformata_finala);
        pcl::io::savePCDFile ("transformed"+patch::to_string(i)+".pcd", *cloud_transformed,true);
        //        // pcd_vector_transform[i]=cloud_transformed;
        *cloud_current=downsample(cloud_transformed);
        //cloud_current=cloud_transformed;
        *cloud_current=outliner_removal(cloud_current);
        *cloud_current=smoothing(cloud_current);
        if(i==start_frame)
            *cloud_final=*cloud_current;
        if(i>start_frame)
        {
            *cloud_previous=*cloud_final;
            //          pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
            //            *cloud_final=*cloud_current + *cloud_before;
            //             pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB> );
            //           pairAlign (cloud_current, cloud_before, cloud_final, pairTransform);
            pairAlign ( cloud_previous,cloud_current, cloud_final, pairTransform); //(src,tgt,tmp)
            //transform current pair into the global transform
            // pcl::transformPointCloud (*temp, *result, GlobalTransform);
            //update the global transform
            // GlobalTransform = pairTransform * GlobalTransform;

            *cloud_final=downsample(cloud_final);
            //*cloud_final=outliner_removal(cloud_final);
            *cloud_final=smoothing(cloud_final);
            pcl::io::savePCDFile ("inter"+patch::to_string(i)+".pcd", *cloud_final,true);
        }

        //*cloud_current=outliner_removal(cloud_current);
        // boost::filesystem3::copy_file("transformetgtd"+patch::to_string(i)+".pcd",fl->path+"/transform/transformed"+patch::to_string(i)+".pcd",boost::filesystem::copy_option::overwrite_if_exists);
    }


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End registration in: "<<duration<<" sec."<<std::endl;
    //*cloud_final=outliner_removal(cloud_final);
    *cloud_final=smoothing(cloud_final);
    pcl::io::savePCDFile ("Concatenat.pcd", *cloud_final,true);

    return *cloud_final;
}

void create_mesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_final){
    std::cout<<"Starting creating mesh:"<<std::endl;
    start = std::clock();
    // Create search tree
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_final);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (1000);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_final);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    pcl::io::saveVTKFile ("mesh.vtk", triangles);
    pcl::io::savePolygonFileSTL("smth.stl",triangles);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End creating mesh in: "<<duration<<" sec."<<std::endl;
}

void normals_viewer(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_to_view){
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_to_view);     //after smoothing
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void readFromXMLFile(std::string filePath)throw(cv::Exception)
{
    std::cout<<"try reading from config file: "<<filePath<<std::endl;


    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << filePath << "\"" << endl;
        exit(-1);
    }

    //    cv::Mat MCamera;
    //    fs["transfomation_matrix"] >> MCamera;
    //    if (MCamera.cols!=4 || MCamera.rows!=4)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);

    fs["Input_seg"] >> path_to_seg_folder_;
    fs["Input_matrix"] >> path_to_transformation_matrix_;
    fs["Height"] >> object_height_;
    fs["Is_UPDOWN"] >> is_object_upside_down_;
    fs["Icp_treeshold"] >> icp_treeshold_;
    fs["Icp_nr_iterations"] >> icp_nr_iterations_;
    fs["Downsample_threshold"] >>downsample_threshold_;
    fs["Smothing_radius"] >> smothing_radius_;
    fs["Outliner_radius"] >> outliner_radius_;
    fs["Outliner_neighbors"] >> outliner_neighbors_;
}
void showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*             Model Creation - Usage Guide              *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: 1st version" << filename << "path_to_config_file" << std::endl << std::endl;
    std::cout << "Usage: 2nd version" << filename << " path_to_seg path_to_transf_matrix [Options]" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                      Show this help." << std::endl;
    std::cout << "     --path_seg [] :          Path to segmented pcd file of object." << std::endl;
    std::cout << "     --path_mat []:           Path to .yml where is stored the transformation matrix." << std::endl;
    std::cout << "     --height [] :            Height of object, used when we create the down part of object, with upside_down flag." << std::endl;
    std::cout << "     -u :                     Use this flag when provide pcd files with object turn up." << std::endl;
    std::cout << "     --config :               Path to .yml file which contain the parameters described up." << std::endl;
}

void parseCommandLine (int argc, char *argv[])
{

    if (argc<2) {
        showHelp(argv[0]);
        exit(0);
    }
    //Show help if -h flag is used
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }

    bool using_external_file;
    if (pcl::console::parse_argument (argc, argv, "--config", path_to_config_file_) != -1)
    {
        using_external_file=true;
        readFromXMLFile(path_to_config_file_);

    }
    if(!using_external_file){

        if (argc<=4) {
            showHelp(argv[0]);
            std::cout<<"Not enough arguments for 2nd version of call !"<<std::endl;
            exit(0);
        }
        //Model & scene filenames
        std::vector<int>filenames;

        filenames = pcl::console::parse_file_extension_argument (argc, argv, ".yml");
        if (filenames.size () == 1)
        {
            std::cout << "Using external config file.\n";
            path_to_config_file_ = argv[filenames[0]];
        }
        else
        {
            showHelp(argv[0]);
            std::cout<<"Not a valid .yml transformation matrix file !"<<std::endl;
            exit(-1);
        }

        //General parameters
        if(pcl::console::parse_argument (argc, argv, "--path_seg", path_to_seg_folder_) == -1 ||  pcl::console::parse_argument (argc, argv, "--path_mat", path_to_transformation_matrix_) == -1)
        {
            showHelp(argv[0]);
            std::cout<<"Input arguments are not correct !"<<std::endl;
            exit(-1);
        }

        pcl::console::parse_argument (argc, argv, "--height", object_height_);        //Program behavior
        if (pcl::console::find_switch (argc, argv, "-u"))
        {
            is_object_upside_down_= true;
        }
    }

}

int main(int argc, char *argv[]) {

    //-------- Load data from files ---------------------------------------------------------------------------------------------
    parseCommandLine (argc, argv);

    FileReader *fl = new FileReader(path_to_seg_folder_);
    //FileReader *fl = new FileReader("/home/stefan/camera_calib_out");

    std::cout << fl->path <<std::endl;
    fl->loadData();
    //---------------------------------------------------------------------------------------------------------------------------

    //-------- Start registration,apply transformation --------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c (new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c2 (new pcl::PointCloud<pcl::PointXYZRGB> );
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_co (new pcl::PointCloud<pcl::PointXYZRGB> );
    //*cloud_c=registration(fl);
    //std::thread first (registration,fl);
//    std::thread first (registration2,fl,0,12);
//    std::thread second (registration2,fl,12,23);
// //    std::thread third (registration2,fl,12,18);
// //    std::thread fourth (registration2,fl,18,23);
//    first.join();
//    second.join();
      // //    third.join();
      // //    fourth.join();

    std::future<pcl::PointCloud<pcl::PointXYZRGB>> ret = std::async(std::launch::async,registration2,fl,0,23,false);
//    std::future<pcl::PointCloud<pcl::PointXYZRGB>> ret2 = std::async(std::launch::async,registration2,fl,12,22,true);
//    Eigen::Matrix4f cv;
//    *cloud_c = ret.get();
//    *cloud_c2 = ret2.get();
//    std::cout<<"Finish ! first:"<<cloud_c->size() << " sec:"<<cloud_c2->size()<<std::endl<<std::endl<<std::endl;
//    pairAlign(cloud_c,cloud_c2,cloud_co,cv);
//    pcl::io::savePCDFileASCII("out.pcd",*cloud_co);

    //---------------------------------------------------------------------------------------------------------------------------

    //-------- Downsampling resulted cloud --------------------------------------------------------------------------------------
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    *cloud_downsampled_ptr=downsample(cloud_c);
    //---------------------------------------------------------------------------------------------------------------------------

    //-------- Outliner removal -------------------------------------------------------------------------------------------------
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //    *cloud_filtered=outliner_removal(cloud_downsampled_ptr);
    //---------------------------------------------------------------------------------------------------------------------------

    //-------- Smoothing and normal estimation based on polynomial reconstruction -----------------------------------------------
    //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //    *mls_points=smoothing_for_mesh(cloud_c);
    //---------------------------------------------------------------------------------------------------------------------------

    //-------- Normal estimation: -----------------------------------------------------------------------------------------------
    //        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //        *cloud_with_normals=normal_estimation(cloud_filtered);
    //---------------------------------------------------------------------------------------------------------------------------

    //-------CREATE MESH from resulted filterd PointCloud -----------------------------------------------------------------------
    //    create_mesh(mls_points);
    //---------------------------------------------------------------------------------------------------------------------------

    //----- Normal visualization ------------------------------------------------------------------------------------------------
    //        normals_viewer(cloud_with_normals);
    //---------------------------------------------------------------------------------------------------------------------------

    return 0;
}


