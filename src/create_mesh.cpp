#include <iostream>
#include <pcl/io/file_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/console/parse.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

pcl::PointCloud<pcl::PointXYZRGB> downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c) {
    std::cout<<"Start downsampling pointCloud with size:"<<(*cloud_c).points.size()<<std::endl;

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.001f, 0.001, 0.001f);
    sor.setInputCloud (cloud_c);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_downsampled_ptr);
    return *cloud_downsampled_ptr;
}

void create_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final){

    //*cloud_final=downsample(cloud_final);

    std::cout<<"Start estimating normals, cloud size:"<<cloud_final->size()<<std::endl;
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls ;

    mls.setComputeNormals (true);
    // Set parametersr
    mls.setInputCloud (cloud_final);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (0.01);
//    mls.setPolynomialOrder(2);
//    mls.setSqrGaussParam(0.0025);
    // Reconstruct
    mls.process (*mls_points);

    std::cout<<"Start creating mesh:"<<std::endl;
    // Create search tree
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (mls_points);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.008);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (5000);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (mls_points);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    //std::vector<int> parts = gp3.getPartIDs();
    //std::vector<int> states = gp3.getPointStates();
    pcl::io::saveVTKFile ("mesh.vtk", triangles);


    pcl::PolygonMesh mesh_vtk;
     pcl::io::loadPolygonFileVTK("mesh.vtk",mesh_vtk);
    pcl::io::savePolygonFileSTL("smth.stl",mesh_vtk);
}

int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud (new  pcl::PointCloud<pcl::PointXYZRGB> );
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size() !=1)
    {
        std::cout << "Filenames missing.\n";
        //showHelp (argv[0]);
        exit (-1);
    }

    if (pcl::io::loadPCDFile (argv[filenames[0]], *m_cloud) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        //showHelp (argv[0]);
        return (-1);
    }
    //pcl::io::loadPCDFile (argv[filenames], *m_cloud);
    create_mesh(m_cloud);
//    pcl::PolygonMesh mesh_vtk;
//     pcl::io::loadPolygonFileVTK("mesh.vtk",mesh_vtk);
//    pcl::io::savePolygonFileSTL("output.stl",mesh_vtk);
//    pcl::io::savePolygonFilePLY("ceva.ply",mesh_vtk);
    return 0;

}
