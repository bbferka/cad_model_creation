
#include <iostream>
#include <iomanip>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<math.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
char tasta;
Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Identity();

void printMatix4f(const Eigen::Matrix4f & matrix) {

    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
//    if (( event.getKeySym() == "space" ||event.getKeySym() == "k" || ) && event.keyDown())
//        next_iteration = true;
    if(event.keyDown()){
    switch ( event.getKeyCode() )   {
    case  'z' :
        tasta='z';
        next_iteration=true;
        break;

    case 'Z' :
        tasta='Z';
        next_iteration=true;
        break;

    case  'v' :
        tasta='v';
        next_iteration=true;
        break;

    case 	'V' :
        tasta='V';
        next_iteration=true;
        break;

    case 	'y' :
        tasta='y';
        next_iteration=true;
        break;

    case 	'Y' :
        tasta='Y';
        next_iteration=true;
        break;

    case   'M'   :
        tasta='M';
        next_iteration=true;
        break;

    case    'm' :
        tasta='m';
        next_iteration=true;
        break;

    case    'b' :
        tasta='b';
        next_iteration=true;
        break;

    case    'B' :
        tasta='B';
        next_iteration=true;
        break;

    case    'n' :
        tasta='n';
        next_iteration=true;
        break;

    case    'N' :
        tasta='N';
        next_iteration=true;
        break;

    case    'a' :
        tasta='a';
        next_iteration=true;
        break;
    case    'A' :
        tasta='A';
        next_iteration=true;
        break;

    case    'd' :
        tasta='d';
        next_iteration=true;
        break;
    case    'D' :
        tasta='D';
        next_iteration=true;
        break;

    case    'i' :
        tasta='i';
        next_iteration=true;
        break;
    case    'I' :
        tasta='I';
        next_iteration=true;
        break;

    case    'k' :
        tasta='k';
        next_iteration=true;
        break;
    case    'K' :
        tasta='K';
        next_iteration=true;
        break;
   case 0x20 :
        tasta='S';
        next_iteration=true;

       default:
        break;
        }
    }
}

void rot_x(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_transform ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed , double param , bool inverse){
    double angle;
    Eigen::Matrix4f rot_x;
    angle = (param*M_PI/180);
    rot_x <<   1,         0 ,           0 ,      0 ,
               0,    cos (angle),  -sin(angle),  0 ,
               0,    sin (angle),   cos(angle),  0 ,
               0 ,        0 ,           0 ,      1 ;
    if(inverse==true)
       global_transformation=global_transformation*rot_x.inverse();
    else
      global_transformation=global_transformation*rot_x;

    pcl::transformPointCloud(*cloud_to_transform,*cloud_transformed,global_transformation);
}



void rot_y(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_transform ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed , double param , bool inverse){
    double angle;
    Eigen::Matrix4f rot_y,transformata;
    angle = (param*M_PI/180);
    rot_y<< cos(angle),      0 ,  sin(angle),     0 ,
                0,           1 ,          0 ,     0 ,
            -sin(angle),     0,   cos(angle),     0 ,
                0 ,          0 ,          0 ,     1 ;
    if(inverse==true)
        global_transformation=global_transformation*rot_y.inverse();
    else
         global_transformation=global_transformation*rot_y;

    pcl::transformPointCloud(*cloud_to_transform,*cloud_transformed,global_transformation);
}

void rot_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_transform ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed , double param , bool inverse){
    double angle;
    Eigen::Matrix4f rot_z;
    angle = (param*M_PI/180);
    rot_z << cos (angle),  -sin(angle),  0 , 0 ,
             sin (angle),   cos(angle),  0 , 0 ,
                   0 ,         0 ,       1 , 0 ,
                   0 ,         0 ,       0 , 1 ;
    if(inverse==true)
         global_transformation=global_transformation*rot_z.inverse();
    else
         global_transformation=global_transformation*rot_z;

     pcl::transformPointCloud(*cloud_to_transform,*cloud_transformed,global_transformation);
}

void translation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_transform ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed , double param , char axe, bool inverse){
    double xd,yd,zd;
    Eigen::Matrix4f trans,transformata;
    switch(axe)
    {
    case 'X' :
        xd=param;
        break;
    case 'Y' :
        yd=param;
        break;
    case 'Z' :
        zd=param;
        break;
    default :
        xd=0;yd=0;zd=0;
        break;
    }

    trans <<  1 , 0 , 0 , xd ,
              0 , 1 , 0 , yd ,
              0 , 0 , 1 , zd ,
              0 , 0 , 0 , 1 ;
    if(inverse==true)
         global_transformation=global_transformation*trans.inverse();
    else
         global_transformation=global_transformation*trans;

     pcl::transformPointCloud(*cloud_to_transform,*cloud_transformed,global_transformation);
}

int main (int argc, char* argv[])
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in 	(new PointCloudT); // Original point cloud
    PointCloudT::Ptr cloud_tr	(new PointCloudT); // Transformed point cloud

    pcl::io::loadPCDFile (argv[1], *cloud_in);
    *cloud_tr=*cloud_in;
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two verticaly separated viewports
    int v1(0); int v2(1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0-bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl);
//    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.addPointCloud (cloud_in, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
//    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
  viewer.addPointCloud (cloud_tr, "cloud_tr_v2", v2);

    // ICP aligned point cloud is red
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
// //    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
// viewer.addPointCloud (cloud_icp, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText("Original point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);

    viewer.addText("Transformed point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

//    std::stringstream ss; ss << iterations;
//    std::string iterations_cnt = "ICP iterations = " + ss.str();
    double z_angle=0, x_angle=0 , y_angle=0,x_tra=0,y_tra=0,z_tra=0;
    double z_angle_incr=0, x_angle_incr=0 , y_angle_incr=0,x_tra_incr=0,y_tra_incr=0,z_tra_incr=0,angle_incr=1,trans_incr=0.01,scale_ang=0.1,scale_trans=0.01;
    std::stringstream ss;
    ss.str (""); ss << x_angle;
    std::string degree_cnt = "X Degree = " + ss.str();
    viewer.addText(degree_cnt, 10, 100, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_x_cnt", v2);

    ss.str (""); ss << y_angle;
    degree_cnt = "Y Degree = " + ss.str();
    viewer.addText(degree_cnt, 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_y_cnt", v2);

    ss.str (""); ss << z_angle;
     degree_cnt = "Z Degree = " + ss.str();
    viewer.addText(degree_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_z_cnt", v2);

    ss.str (""); ss << x_tra;
     degree_cnt = "X trans = " + ss.str();
    viewer.addText(degree_cnt, 380, 105, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_x_cnt", v2);

    ss.str (""); ss << y_tra;
     degree_cnt = "Y trans = " + ss.str();
    viewer.addText(degree_cnt, 380, 85, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_y_cnt", v2);

    ss.str (""); ss << z_tra;
     degree_cnt = "Z trans = " + ss.str();
    viewer.addText(degree_cnt, 380, 65, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_z_cnt", v2);

    ss.str (""); ss << angle_incr;
     degree_cnt = "Angle increment = " + ss.str();
    viewer.addText(degree_cnt, 150, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_cnt", v2);

    ss.str (""); ss << trans_incr;
     degree_cnt = "Tramslation increment = " + ss.str();
    viewer.addText(degree_cnt, 500, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_cnt", v2);

    ss.str (""); ss << scale_ang;
     degree_cnt = "-> scale = " + ss.str();
    viewer.addText(degree_cnt, 150, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_scale_cnt", v2);

    ss.str (""); ss << scale_trans;
     degree_cnt = "-> scale = " + ss.str();
    viewer.addText(degree_cnt, 500, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_scale_cnt", v2);

    // Set background color
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    viewer.addCoordinateSystem(0.2, 0);

    // Set camera position and orientation
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024); // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);


//Eigen::Matrix4f prev;
    // Display the visualiser
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration) {
           viewer.removePointCloud ("cloud_in_v2", v2);
           switch(tasta)
           {
           case 'v' :
           {
            x_angle += angle_incr;
            x_angle_incr=angle_incr;
            rot_x(cloud_in,cloud_tr,x_angle_incr,false);
            ss.str (""); ss << x_angle;
            std::string degree_cnt = "X Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 100, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_x_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'V' :
           {
            x_angle -= angle_incr;
            x_angle_incr=-angle_incr;
            rot_x(cloud_in,cloud_tr,x_angle_incr,false);
            ss.str (""); ss << x_angle;
            std::string degree_cnt = "X Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 100, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_x_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'y' :
           {
            y_angle+=angle_incr;
            y_angle_incr=angle_incr;
            rot_y(cloud_in,cloud_tr,y_angle_incr,false);
            ss.str (""); ss << y_angle;
            std::string degree_cnt = "Y Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_y_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'Y' :
           {
            y_angle-=angle_incr;
            y_angle_incr=-angle_incr;
            rot_y(cloud_in,cloud_tr,y_angle_incr,false);
            ss.str (""); ss << y_angle;
            std::string degree_cnt = "Y Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_y_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'z' :
           {
            z_angle+=angle_incr;
            z_angle_incr=angle_incr;
            rot_z(cloud_in,cloud_tr,z_angle_incr,false);
            ss.str (""); ss << z_angle;
            std::string degree_cnt = "Z Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_z_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'Z' :
           {
            z_angle-=angle_incr;
            z_angle_incr=-angle_incr;
            rot_z(cloud_in,cloud_tr,z_angle_incr,false);
            ss.str (""); ss << z_angle;
            std::string degree_cnt = "Z Degree = " + ss.str();
                viewer.updateText (degree_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "degree_z_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'b' :
           {
            x_tra+=trans_incr;
            x_tra_incr=trans_incr;
            translation(cloud_in,cloud_tr,x_tra_incr,'X',false);
            ss.str (""); ss << x_tra;
            std::string degree_cnt = "X trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 105, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_x_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'B' :
           {
            x_tra-=trans_incr;
            x_tra_incr=-trans_incr;
            translation(cloud_in,cloud_tr,x_tra_incr,'X',false);
            ss.str (""); ss << x_tra;
            std::string degree_cnt = "X trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 105, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_x_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'n' :
           {
            y_tra+=trans_incr;
            y_tra_incr=trans_incr;
            translation(cloud_in,cloud_tr,y_tra_incr,'Y',false);
            ss.str (""); ss << y_tra;
            std::string degree_cnt = "Y trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 85, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_y_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'N' :
           {
            y_tra-=trans_incr;
            y_tra_incr=-trans_incr;
            translation(cloud_in,cloud_tr,y_tra_incr,'Y',false);
            ss.str (""); ss << y_tra;
            std::string degree_cnt = "Y trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 85, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_y_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }

           case 'm' :
           {
            z_tra+=trans_incr;
            z_tra_incr=trans_incr;
            translation(cloud_in,cloud_tr,z_tra_incr,'Z',false);
            ss.str (""); ss << z_tra;
            std::string degree_cnt = "Z trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 65, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_z_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'M' :
           {
            z_tra-=trans_incr;
            z_tra_incr=-trans_incr;
            translation(cloud_in,cloud_tr,z_tra_incr,'Z',false);
            ss.str (""); ss << z_tra;
            std::string degree_cnt = "Z trans = " + ss.str();
                viewer.updateText (degree_cnt, 380, 65, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_z_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
                viewer.updatePointCloud (cloud_tr,"cloud_tr_v2");
                break;
           }
           case 'a' :
           {
            angle_incr+=scale_ang;
            ss.str (""); ss << angle_incr;
            std::string degree_cnt = "Angle increment = " + ss.str();
                viewer.updateText (degree_cnt, 150, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_cnt");
                break;
           }
           case 'A' :
           {
            if(angle_incr>(0+scale_ang))
                angle_incr-=scale_ang;
            ss.str (""); ss << angle_incr;
            std::string degree_cnt = "Angle increment = " + ss.str();
                viewer.updateText (degree_cnt, 150, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_cnt");
                break;
           }
           case 'd' :
           {
            trans_incr+=scale_trans;
            ss.str (""); ss << trans_incr;
            std::string degree_cnt = "Translation increment = " + ss.str();
                viewer.updateText (degree_cnt, 500, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_cnt");
                break;
           }
           case 'D' :
           {
               if(trans_incr>(0+scale_trans))
            trans_incr-=scale_trans;
            ss.str (""); ss << trans_incr;
            std::string degree_cnt = "Translation increment = " + ss.str();
                viewer.updateText (degree_cnt, 500, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_cnt");
                break;
           }
           case 'i' :
           {
            scale_ang+=0.01;
            ss.str (""); ss << scale_ang;
            std::string degree_cnt = "-> scale = " + ss.str();
                viewer.updateText (degree_cnt, 150, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_scale_cnt");
                break;
           }
           case 'I' :
           {
            if(scale_ang>(0+0.01))
                scale_ang-=0.01;
            ss.str (""); ss << scale_ang;
            std::string degree_cnt = "-> scale = " + ss.str();
                viewer.updateText (degree_cnt, 150, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "angle_incr_scale_cnt");
                break;
           }
           case 'k' :
           {
            scale_trans+=0.0001;
            ss.str (""); ss << scale_trans;
            std::string degree_cnt = "-> scale = " + ss.str();
                viewer.updateText (degree_cnt, 500, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_scale_cnt");
                break;
           }
           case 'K' :
           {
            if(scale_trans>(0+0.0001))
                scale_trans-=0.0001;
            ss.str (""); ss << scale_trans;
            std::string degree_cnt = "-> scale = " + ss.str();
                viewer.updateText (degree_cnt, 500, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "trans_incr_scale_cnt");
                break;
           }
           case 'S':
           {
               global_transformation = Eigen::Matrix4f::Identity();
               *cloud_in=*cloud_tr;
               viewer.resetCamera();

               break;
           }
           default :
               break;
        }


      }
        next_iteration = false;
        //tasta ='[';
    }
    //save aligned pair, transformed into the first cloud's frame
//std::stringstream ss;
ss << "lol.pcd";
//*cloud_icp+=*cloud_in;
//pcl::io::savePCDFile (ss.str (), *cloud_tr, true);

 return 0;
}
