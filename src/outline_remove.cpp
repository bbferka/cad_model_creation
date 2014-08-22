/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/io/file_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <thread>
#include <chrono>
#include <future>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
    //our visualizer
    pcl::visualization::PCLVisualizer *p;
    //its left and right viewports
    int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};


pcl::PointCloud<pcl::PointXYZRGB> downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c) {
    std::clock_t start;
    double duration;
    std::cout<<"Start downsampling pointCloud with size:"<<(*cloud_c).points.size()<<std::endl;
    start = std::clock();
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.0008f, 0.0008f, 0.0008f);
    sor.setInputCloud (cloud_c);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_downsampled_ptr);
    pcl::io::savePCDFile ("Concatenat2.pcd", *cloud_downsampled_ptr,true);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"End downsampling, pcd size:"<<(*cloud_downsampled_ptr).points.size()<<" in "<<duration<<" seconds"<<std::endl;
    return *cloud_downsampled_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB> outliner_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr){
    std::cout<<"Start removing outliners, before pointCloud had size:"<<(*cloud_downsampled_ptr).points.size()<<std::endl;
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
          // build the filter
          outrem.setInputCloud(cloud_downsampled_ptr);
          outrem.setRadiusSearch(0.01);
          outrem.setMinNeighborsInRadius (350);
          // apply filter
          outrem.filter (*cloud_filtered);
          outrem.setNegative(true);
          outrem.filter(*cloud_filtered2);

//      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
//    sor2.setInputCloud (cloud_downsampled_ptr);
//    sor2.setMeanK (2000);
//    sor2.setStddevMulThresh (3.5);
//    sor2.filter (*cloud_filtered);
//    pcl::io::savePCDFile ("Filtered.pcd", *cloud_filtered,true);
//    std::cout<<"End filtering, pcd size:"<<(*cloud_filtered).points.size()<<std::endl;
//    sor2.setNegative(true);
//    sor2.filter(*cloud_filtered2);
//    pcl::io::savePCDFile ("Out.pcd", *cloud_filtered2,true);

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two verticaly separated viewports
    int v1(0); int v2(1);int v3(2);
    viewer.createViewPort (0.0, 0.0, 0.33, 1.0, v1);
    viewer.createViewPort (0.33, 0.0, 0.6, 1.0, v2);
    viewer.createViewPort (0.66, 0.0, 1.0, 1.0, v3);
    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0-bckgr_gray_level;

    viewer.addPointCloud (cloud_downsampled_ptr, "cloud_in_v1", v1);
viewer.addText("Original point cloud\n", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addPointCloud (cloud_filtered, "cloud_in_v2", v2);
viewer.addText("Inliers", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
    viewer.addPointCloud (cloud_filtered2, "cloud_in_v3", v3);
viewer.addText("Outliers", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_3", v3);

    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
    return *cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBNormal> normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
    std::cout<<"Start estimating normals:"<<std::endl;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    tree->setInputCloud (cloud_filtered);
    n.setInputCloud (cloud_filtered);
    n.setSearchMethod (tree);
    n.setRadiusSearch(0.02); //0.005
    //n.setKSearch (100);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
    std::cout<<"End estimating normals, pcd size:"<<(*cloud_with_normals).points.size()<<std::endl;

    return *cloud_with_normals;
}


pcl::PointCloud<pcl::PointXYZRGB> smoothing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
    std::cout<<"Start smoothing"<<std::endl;
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls ;

    mls.setComputeNormals (false);
    // Set parameters
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialFit (false);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (0.01);
    mls.setPolynomialOrder(3);
    mls.setSqrGaussParam(0.01);
    // Reconstruct
    mls.process (*mls_points);
    // Save output
    pcl::io::savePCDFile ("smoth.pcd", *mls_points);
    std::cout<<"End smoothing, pcd size:"<<(*mls_points).points.size()<<std::endl;

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("Smoothing");
    // Create two verticaly separated viewports
    int v1(0); int v2(1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1, 1.0, v2);

    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0-bckgr_gray_level;

    viewer.addPointCloud (cloud_filtered, "cloud_in_v1", v1);
viewer.addText("Original point cloud", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addPointCloud (mls_points, "cloud_in_v2", v2);
viewer.addText("Smoothed point cloud", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
}
    return *mls_points;
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

void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
        pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spinOnce();
}


//////////////////////////////////////////////////////////////////////////////////
///** \brief Display source and target on the second viewport of the visualizer
// *
// */
//void showCloudsRight(const  PointCloud::Ptr cloud_target)
//{
//  p->removePointCloud ("target");

// PointCloudColorHandlerCustom<PointT> tgt_color_handler (cloud_target, 255, 0, 0);
// if (!tgt_color_handler.isCapable ())
//     PCL_WARN ("Cannot create curvature color handler!");
//  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
//      p->spin ();

//}

void showCloudsRight(const  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target)
{
  p->removePointCloud ("target");

 PointCloudColorHandlerGenericField<pcl::PointXYZRGBNormal> tgt_color_handler (cloud_target, "curvature");
 if (!tgt_color_handler.isCapable ())
     PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
      p->spin ();

}

int main (int argc, char** argv)
{
    // Load data
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData (argc, argv, data);

    // Check user input
    if (data.empty ())
    {
      PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
      PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
      return (-1);
    }
    PCL_INFO ("Loaded %d datasets.", (int)data.size ());
    std::cout<<"Loaded:"<<data.size()<<" files"<<std::endl;

    // Create a PCLVisualizer object

//    p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
//    p->createViewPort (0.0, 0, 0.5, 1.0,3 vp_1);
//    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

    PointCloud::Ptr result (new PointCloud), source;
    for (size_t i = 0; i < data.size (); i++)
    {
      source = data[i].cloud;

      // Add visualization data
      //showCloudsLeft(source);
    *result=downsample(source);
      //*result=*source;
      *result=outliner_removal(result);
       //*result=outliner_removal(result);
       //*result=outliner_removal(result);
     *result=smoothing(result);
      //showCloudsRight(result);
//      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//      *cloud_with_normals=smoothing(result);
     // showCloudsRight(cloud_with_normals);
      //*cloud_with_normals=normal_estimation(result);
    //  normals_viewer(cloud_with_normals);
      std::cout<<"end";
          //save aligned pair, transformed into the first cloud's frame
      std::stringstream ss;
      ss << i << ".pcd";
      pcl::io::savePCDFile (ss.str (), *result, true);

    }
    return 0;

}
