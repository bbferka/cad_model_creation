/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/


#include <iostream>
#include <model_creation/aruco_include/aruco.h>
#include "model_creation/aruco_include/cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include  "opencv2/calib3d/calib3d.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <chrono>
using namespace cv;
using namespace aruco;
cv::Mat color;
bool find_cloud_,end_of_prog;
std::string path_image_,path_yml,path_image_out;
float marker_size_;
int marker_id_,argc_;
void write_paramsS(cv::Mat cameraMatrix)
{
    FileStorage fs("test_param.yml", FileStorage::WRITE);
    fs << "transfomation_matrix" << cameraMatrix;
    fs.release();

}

int gen_o()
{
    aruco::CameraParameters CamParam;
    MarkerDetector MDetector;
    vector<Marker> Markers;
    //read the input image
    cv::Mat InImage;
    //read camera parameters if specifed
    if (argc_>=3) {
        CamParam.readFromXMLFile(path_yml);
        //resizes the parameters to fit the size of the input image
    }
    while(!end_of_prog)
        if(find_cloud_)
            try
        {

            while(true){
                InImage=color;
                CamParam.resize( InImage.size());
                MDetector.detect(InImage,Markers,CamParam,marker_size_);
                //show also the internal image resulting from the threshold operation
                for (unsigned int i=0;i<Markers.size();i++)
                {
                    if(Markers[i].id==marker_id_)
                    {CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
                    CvDrawingUtils::draw3dAxis(InImage,Markers[i],CamParam);}
                    Markers[i].draw(InImage,Scalar(0,0,255),2);
                }
                cv::imshow("CenterDetect",InImage);
                if(waitKey(200) >= 0) break;
                //cv::waitKey(0);//wait for key to be pressed
            }

            //CvMat * transformata =cvCreateMat(3,3,CV_32F);
            CvMat rotatia;
            //cv::Mat * rezult = new cv::Mat();
            cv::Mat dest,m;
            //for each marker, draw info and its boundaries in the image
            for (unsigned int i=0;i<Markers.size();i++) {
                cout<<std::endl<<Markers[i]<<endl;
                if(Markers[i].id==marker_id_)
                {
                    end_of_prog=true;
                    rotatia=Markers[i].Rvec;
                    cv::Rodrigues(Markers[i].Rvec,dest);
                    std::cout<<"cv::Rodrigues rezult:"<<std::endl<<dest<<std::endl;
                    m=dest;
                    cv::hconcat(m, Markers[i].Tvec, m);
                    cv::Mat linie = cv::Mat::zeros(1, 4, CV_32FC1);
                    linie.ptr<float>(0)[3]=1;
                    cv::vconcat(m,linie,m);
                    write_paramsS(m);

                }
                CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
                CvDrawingUtils::draw3dAxis(InImage,Markers[i],CamParam);
                Markers[i].draw(InImage,Scalar(0,0,255),2);
            }

            if (argc_>=6) cv::imwrite(path_image_out,InImage);

            //std::this_thread::sleep_for (std::chrono::seconds(4));
            //return 0;

        } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}

void cloud_cb (const sensor_msgs::Image::Ptr  input)
{
    cv_bridge::CvImageConstPtr pCvColor = cv_bridge::toCvShare(input, sensor_msgs::image_encodings::BGR8);
    //cv::Mat color;
    pCvColor->image.copyTo(color);
    find_cloud_=true;

}
int main(int argc,char **argv)
{
    if (argc<2) {
        cerr<<"Usage: (in.jpg|in.avi) [cameraParams.yml] [markerSize] [markerID center] [outImage] "<<endl;
        exit(0);
    }
    argc_=argc;
    path_image_ = argv[1];

    argc_=argc;
    if (argc>=3) {
        path_yml=argv[2];
    }
    if (argc>=4)
    {
        marker_size_=atof(argv[3]);
        marker_id_=atoi(argv[4]);
    }
    if (argc>=6) path_image_out=argv[5];
    std::cout<<path_image_<<std::endl<<path_yml<<std::endl<<path_image_out<<std::endl<<marker_id_<<std::endl<<marker_size_<<std::endl<<argc_<<std::endl;


    std::thread first (gen_o);
    ros::init(argc, argv, "image_converter");

    //ImageConverter ic;
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/rgb/image_raw", 1, cloud_cb);

    while(!end_of_prog)
        ros::spinOnce();

    first.join();

}

