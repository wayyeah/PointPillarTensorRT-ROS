/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include <dirent.h>
#include <memory>
#include <iostream>
#include <vector>
#include <iomanip>
#include <map>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <unistd.h>
#include <string>
#include <cstdio>
#include<thread>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_datatypes.h>
#include "cuda_runtime.h"
#include "./params.h"
#include "./pointpillar.h"
#include<time.h>
#include <unistd.h>
#include<limits.h>
#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

std::string Model_File = "/pointpillar.onnx";
char cwd[PATH_MAX];

void Getinfo(void)
{
  cudaDeviceProp prop;
  std::cout<<"cwd: "<<getcwd(cwd,sizeof(cwd))<<std::endl;
  Model_File =std::string(cwd)+"/src/"+Model_File;
  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}




ros::Publisher pub;
Params params;
cudaStream_t stream = NULL;
float *d_points = nullptr;
int count=0;
PointPillar* pointpillar = nullptr;
Params params_;
std::vector<Bndbox> nms_pred;



bool fileExists( std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

void SaveToBin(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& path) {
    std::cout << "bin_path: " << path << std::endl;
    //Create & write .bin file
    std::ofstream out(path.c_str(), std::ios::out|std::ios::binary);
    if(!out.good()) {
        std::cout<<"Couldn't open "<<path<<std::endl;
        return;
    }
    float zero = 0.0f;
    for (size_t i = 0; i < cloud.points.size (); ++i) {
        out.write((char*)&cloud.points[i].x, 3*sizeof(float));
       out.write((char*)&zero, sizeof(float));
       
    }
    
    out.close();
}

bool hasEnding(std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

static bool startswith(const char *s, const char *with, const char **last)
{
    while (*s++ == *with++)
    {
        if (*s == 0 || *with == 0)
            break;
    }
    if (*with == 0)
        *last = s + 1;
    return *with == 0;
}


int loadData(const char *file, void **data, unsigned int *length)
{
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open()) {
        std::cout << "Can't open files: "<< file<<std::endl;
        return -1;
    }

    unsigned int len = 0;
    dataFile.seekg (0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg (0, dataFile.beg);

    char *buffer = new char[len];
    if (buffer==NULL) {
        std::cout << "Can't malloc buffer."<<std::endl;
        dataFile.close();
        exit(EXIT_FAILURE);
    }

    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void*)buffer;
    *length = len;
    return 0;  
}
class RosSub_Pub
{  
public:  
    RosSub_Pub() 
    {        
        pub_1 = n.advertise<sensor_msgs::PointCloud2>("/my_topic", 10, true);
        sub_1 = n.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 10, &RosSub_Pub::callback, this);
        pub_bbox = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxes", 10,true);
    }  
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg)  
    {  
        clock_t start_time=clock();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        const size_t num_points = cloud.points.size();
        const size_t num_features = 4;
        float *d_points = new float[num_points * num_features];
        for(size_t i=0;i<num_points;i++){
            d_points[i*4]=cloud.points[i].x;
            d_points[i*4+1]=cloud.points[i].y;
            d_points[i*4+2]=cloud.points[i].z;
            d_points[i*4+3]=0.0f;
        }

        size_t points_size = cloud.points.size();
        //std::cout << "find points num: "<< points_size <<std::endl;
        float *points_data = nullptr;
        unsigned int points_data_size = points_size * 4 * sizeof(float);
        checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
        checkCudaErrors(cudaMemcpy(points_data,  d_points, points_data_size, cudaMemcpyDefault));
        checkCudaErrors(cudaDeviceSynchronize());
        pointpillar->doinfer(points_data, points_size, nms_pred);
        jsk_recognition_msgs::BoundingBoxArray arr_bbox;
        for (const auto box : nms_pred) {
            jsk_recognition_msgs::BoundingBox bbox;
           

            bbox.header.frame_id = msg->header.frame_id;  // Replace with your frame_id
            bbox.header.stamp = msg->header.stamp;
            bbox.pose.position.x =  box.x;
            bbox.pose.position.y =  box.y;
            bbox.pose.position.z = box.z;
            bbox.dimensions.x = box.w;  // width
            bbox.dimensions.y = box.l;  // length
            bbox.dimensions.z = box.h;  // height
            // Using tf::Quaternion for quaternion from roll, pitch, yaw
            tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, box.rt);
            bbox.pose.orientation.x = q.x();
            bbox.pose.orientation.y = q.y();
            bbox.pose.orientation.z = q.z();
            bbox.pose.orientation.w = q.w();
            bbox.value = box.score;
            bbox.label = box.id;
            if(box.score>0.6){
            arr_bbox.boxes.push_back(bbox);
            }

        }
        arr_bbox.header.frame_id = msg->header.frame_id;
        arr_bbox.header.stamp = msg->header.stamp;
        nms_pred.clear();
        pub_1.publish(msg);
        pub_bbox.publish(arr_bbox);
        //std::cout << "All Time: " << duration1.count() << " milliseconds" << std::endl;
        clock_t end_time=clock();
        std::cout<< "All Running time is: "<<static_cast<double>(end_time-start_time)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;//输出运行时间
    }  
      
private:  
    ros::NodeHandle n;   
    // 创建发布对象
    ros::Publisher pub_1; 
    // 创建订阅对象
    ros::Subscriber sub_1; 
    ros::Publisher pub_bbox;
      
};

int main(int argc,  char **argv)
{
    Getinfo();
    checkCudaErrors(cudaStreamCreate(&stream));
    pointpillar=new PointPillar(Model_File,stream);
    nms_pred.reserve(100);
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"pointcloud_detector");
    RosSub_Pub Sub_pub_obj;
    ros::spin();
    return 0;
}