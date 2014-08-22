#include<iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h> //hydro
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/filesystem.hpp>
#include <pcl/filters/filter.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include<pcl/common/centroid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_line.h>
#include<pcl/sample_consensus/sac_model_circle.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/common/transforms.h>
#include <model_creation/Estimate_Transformation.h>
#include<pcl/filters/radius_outlier_removal.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
std::string path;
Eigen::Matrix4d transformata_finala;

Estimate_Transformation::Estimate_Transformation(std::string f_path){
     //  std::cout<<"Path= "<<f_path<<std::endl;
    path=f_path; // path to plane
}
Estimate_Transformation::~Estimate_Transformation()
{
}

Eigen::Vector4d calculate_centroid(PointCloud::Ptr cluster_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls ;
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cluster_cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f); //0.0015f toate cele 3
    vg.filter (*cloud_filtered);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.05);
    outrem.setMinNeighborsInRadius (50);
    // apply filter
    outrem.filter (*cloud_filtered);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    mls.setComputeNormals (false);
    // Set parametersr
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (0.1);
    mls.setPolynomialOrder(2);
    mls.setSqrGaussParam(0.005);
    // Reconstruct
    mls.process (*mls_points);

    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(*mls_points, xyz_centroid);

    return xyz_centroid;
}


Eigen::Vector3d estimate_plane_normals(PointCloud::Ptr cloud_f)
{
    PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_filtered=cloud_f;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    //std::cout<<"Plane Coefficients:"<<std::endl<<*coefficients<<std::endl;
    Eigen::Vector3d plane_normal_vector ;
    for(int i=0;i<3;i++)
        plane_normal_vector(i) = -coefficients->values[i];
    //std::cout<<"Plane Coefficients:"<<std::endl<<plane_normal_vector<<std::endl;

    return plane_normal_vector;
}


Eigen::Vector3d calculate_one_object_axe (PointCloud cloud_cluster)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_aux=cloud_cluster;
    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3d covariance_matrix,result;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    //pcl::compute3DCentroid(*cloud_aux, xyz_centroid);
      xyz_centroid=calculate_centroid(cloud_aux);
    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix(*cloud_aux, xyz_centroid, covariance_matrix);
    Eigen::EigenSolver <Eigen::Matrix3d> es(covariance_matrix);
    //std::cout<<std::endl<<"Eigen vectors:"<<std::endl<<es.pseudoEigenvectors()<<std::endl;
    result=es.pseudoEigenvectors();
    //    std::cout<<std::endl<<"Eigen values:"<<std::endl<<es.eigenvalues()<<std::endl;
    Eigen::VectorXcd eivals = es.eigenvalues();
    //std::cout<<std::endl<<"Eigen values:"<<std::endl<<eivals<<std::endl;

    int j=0;
    double max;
    //max = eivals(0,0);
    max = eivals(0).real();
    for(int i=1;i<3;i++)
    {
        if(max < eivals(i).real())
            j=i;
    }
    Eigen::Vector3d vector_1 ;
    for(int i=0;i<3;i++)
        vector_1(i)=result(i,j);

    return vector_1;

}

Eigen::Matrix4d calculate_transformation(Eigen::Vector3d x_or_y_axe_vector, Eigen::Vector3d z_axe_vector, Eigen::Vector4d xyz_centroid)
{
    transformata_finala=Eigen::MatrixXd::Identity(4,4);
    Eigen::Vector3d axa3;
    axa3(0)=x_or_y_axe_vector(1)*z_axe_vector(2)-z_axe_vector(1)*x_or_y_axe_vector(2);//  a_nou = b1*c2 - b2*c1;
    axa3(1)=z_axe_vector(0)*x_or_y_axe_vector(2)-x_or_y_axe_vector(0)*z_axe_vector(2);//  b_nou = a2*c1 - a1*c2;
    axa3(2)=x_or_y_axe_vector(0)*z_axe_vector(1)-x_or_y_axe_vector(1)*z_axe_vector(0);//  c_nou = a1*b2 - b1*a2;
    for(int i=0;i<3;i++)
    {
        transformata_finala(i,0) =x_or_y_axe_vector(i);
        transformata_finala(i,1) =axa3(i);
        transformata_finala(i,2) =-z_axe_vector(i);
    }
    for(int i=0;i<3;i++)
        transformata_finala(i,3)=xyz_centroid(i);

    std::cout<<std::endl<<"Transformation Matrix:"<<std::endl<<transformata_finala<<std::endl;
    return transformata_finala;
}



Eigen::Matrix4d Estimate_Transformation::get_transformation()
{
    PointCloud::Ptr cloud (new PointCloud);
    pcl::io::loadPCDFile(path,*cloud);  // "/home/stefan/result_plane_p_0.pcd"
    Eigen::Vector3d z_axe,x_or_y_axe;
    Eigen::Vector4d translation;
\
    z_axe=estimate_plane_normals(cloud);
     x_or_y_axe = calculate_one_object_axe(*cloud);
    translation = calculate_centroid(cloud);
    Eigen::Matrix4d transformata =  calculate_transformation(x_or_y_axe,z_axe ,translation);

    return transformata;

}
