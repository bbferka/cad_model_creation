

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
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr cloud (new PointCloud);
Eigen::Vector4d calculate_centroid(PointCloud cluster_cloud);

Eigen::Matrix4d transformata_finala;

Eigen::Vector3d calculate_one_object_axe (PointCloud::Ptr cloud_cluster);
Eigen::Vector3d estimate_plane_normals(PointCloud::Ptr cloud_f);
Eigen::Matrix4d calculate_transformation(Eigen::Vector3d first_vector, Eigen::Vector3d second_vector,Eigen::Vector4d);


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
    PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_f_aux (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_filtered=cloud_f;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);


    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    //pcl::compute3DCentroid(*cloud_plane, xyz_centroid);
    // Estimate the XYZ centroid
    //pcl::compute3DCentroid(*cloud_f, xyz_centroid);
    xyz_centroid=calculate_centroid(cloud_f);
    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.001;
    pcl::PointXYZRGB p_x,p_y,p_z;
    p_x.r=0;
    p_x.g=0;
    p_x.b=255;
    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<200;i++)
    {
        p_x.x =x0 + coefficients->values[0]*l;
        p_x.y =y0 + coefficients->values[1]*l;
        p_x.z =z0 + coefficients->values[2]*l;

        l+=0.001;
        axe->push_back(p_x);
    }

    std::cout<<"axe size"<<axe->size()<<std::endl;

    pcl::visualization::PCLVisualizer viewer ("Plane Normal Vector");
//    int v1(0); int v2(1);
//    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//    //viewer.addCoordinateSystem(0.1,0);
//    viewer.addPointCloud (cloud_f, "cloud",v1);
    viewer.addPointCloud (cloud_f, "plane");
    viewer.addPointCloud (axe, "a");
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }

    //std::cout<<"Plane Coefficients:"<<std::endl<<*coefficients<<std::endl;
    Eigen::Vector3d plane_normal_vector ;
    for(int i=0;i<3;i++)
        plane_normal_vector(i) = coefficients->values[i];
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

    //std::cout<<std::endl<<"Centroid"<<std::endl << xyz_centroid << std::endl;

    //std::cout<<std::endl<<"Covariance Matrix"<<std::endl << covariance_matrix << std::endl;

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
    Eigen::Vector3d vector_1,vector_2,vector_3 ;
//    for(int i=0;i<3;i++) // good version
//        {
//            vector_1(i)=result(i,j);
//        }
    for(int i=0;i<3;i++) //test version
        {
            vector_1(i)=-result(i,0);
            vector_2(i)=-result(i,1);
            vector_3(i)=-result(i,2);
        }

    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.001;
    pcl::PointXYZRGB p_x ,p_x_m, p_x_m2;
    p_x.r=255;
    p_x.g=0;
    p_x.b=0;

    p_x_m.r=0;
    p_x_m.g=255;
    p_x_m.b=0;

    p_x_m2.r=0;
    p_x_m2.g=0;
    p_x_m2.b=255;
    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<300;i++)
    {
//        p_x.x =x0 + vector_1(0)*l;
//        p_x.y =y0 + vector_1(1)*l;
//        p_x.z =z0 + vector_1(2)*l;

//        p_x_m.x =x0 - vector_1(0)*l;
//        p_x_m.y =y0 - vector_1(1)*l;
//        p_x_m.z =z0 - vector_1(2)*l;
        p_x.x =x0 + vector_1(0)*l;
        p_x.y =y0 + vector_1(1)*l;
        p_x.z =z0 + vector_1(2)*l;

//        p_x_m.x =x0 - vector_2(0)*l;
//        p_x_m.y =y0 - vector_2(1)*l;
//        p_x_m.z =z0 - vector_2(2)*l;

//        p_x_m2.x =x0 - vector_3(0)*l;
//        p_x_m2.y =y0 - vector_3(1)*l;
//        p_x_m2.z =z0 - vector_3(2)*l;

        l+=0.001;
        axe->push_back(p_x);
//        axe->push_back(p_x_m);
//     axe->push_back(p_x_m2);
//        if(i==99)

//        {
//            std::cerr<<"red xyz: "<<p_x.x<<" "<<p_x.y<<" "<<p_x.z<<std::endl;
//            std::cerr<<"green xyz: "<<p_x_m.x<<" "<<p_x_m.y<<" "<<p_x_m.z<<std::endl;

//            if(p_x_m.y<p_x.y)
//                vector_1*=-1;
//        }

    }

//    //    //std::cout<<"axe size"<<axe->size()<<std::endl;


    pcl::visualization::PCLVisualizer viewer ("Object CLuster Axe");
//    int v1(0); int v2(1);
//    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//    viewer.addPointCloud (cloud_aux, "cloud",v1);
    viewer.addPointCloud (cloud, "cluster");
    viewer.addPointCloud (axe, "a");
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
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
        transformata_finala(i,0) =axa3(i);
        transformata_finala(i,1) =x_or_y_axe_vector(i);
        transformata_finala(i,2) =z_axe_vector(i);
    }

    for(int i=0;i<3;i++)
        transformata_finala(i,3)=xyz_centroid(i);

    std::cout<<std::endl<<"Transformation Matrix:"<<std::endl<<transformata_finala<<std::endl;

    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.005;
    pcl::PointXYZRGB p_x,p_y,p_z;
    p_x.r=255;
    p_x.g=0;
    p_x.b=0;

    p_y.r=0;
    p_y.g=255;
    p_y.b=0;

    p_z.r=0;
    p_z.g=0;
    p_z.b=255;

    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<100;i++)
    {

        //axa x red
        p_x.x =x0 + transformata_finala(0,0)*l;
        p_x.y =y0 + transformata_finala(1,0)*l;
        p_x.z =z0 + transformata_finala(2,0)*l;

        //axa y green
        p_y.x =x0 + transformata_finala(0,1)*l;
        p_y.y =y0 + transformata_finala(1,1)*l;
        p_y.z =z0 + transformata_finala(2,1)*l;

        //axa z blue
        p_z.x =x0 + transformata_finala(0,2)*l;
        p_z.y =y0 + transformata_finala(1,2)*l;
        p_z.z =z0 + transformata_finala(2,2)*l;

        axe->push_back(p_x);
        axe->push_back(p_y);
        axe->push_back(p_z);

        l+=0.005;
    }

    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    int v1(0); int v2(1);
    //    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    //    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    //viewer.addCoordinateSystem(0.1,0);
    // viewer.addPointCloud (cloud_f, "cloud",v1);
    viewer.addPointCloud (cloud, "plane",0);
    viewer.addPointCloud (axe, "a",0);
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
    return transformata_finala;
}



int main(int argc, char ** argv)
{
    pcl::io::loadPCDFile("/home/stefan/Documents/Licenta_Image/result_plane_p_0.pcd",*cloud);
    Eigen::Vector3d z_axe,x_or_y_axe;
    Eigen::Vector4d translation;

    z_axe=estimate_plane_normals(cloud);
    translation = calculate_centroid(cloud);
    //pcl::compute3DCentroid(*cloud, translation);
    x_or_y_axe = calculate_one_object_axe(*cloud);
    //calculate_transformation(x_or_y_axe,z_axe ,translation);
    PointCloud::Ptr cloud_transformed (new PointCloud());
//    PointCloud::Ptr rr ;
//    rr.reset(new PointCloud());
//    *rr= *cloud;
//    pcl::io::savePCDFileASCII("rr.pcd",*rr);
    Eigen::Matrix4d transformata =  calculate_transformation(x_or_y_axe,z_axe ,translation);
    Eigen::Matrix4d aux = transformata.inverse();
    //std::cout<<translation[0]<<std::endl;
    pcl::transformPointCloud(*cloud,*cloud_transformed,aux);


    pcl::visualization::PCLVisualizer viewer ("Final Transformed");
//    int v1(0); int v2(1);
//    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//    viewer.addPointCloud (cloud, "cloud",v1);
    viewer.addPointCloud (cloud_transformed, "cluster");
    viewer.addCoordinateSystem(0.2,0);
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
    return 0;
}
