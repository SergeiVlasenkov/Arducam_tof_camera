
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ctime>
#include "ArducamTOFCamera.hpp"

//serg <<
#include <pcl/filters/voxel_grid.h>
//serg >>

#define MAX_DISTANCE 4
using namespace Arducam;
boost::mutex updateModelMutex;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(0.01);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }
}

void getPreview(uint8_t *preview_ptr, float *depth_image_ptr, float *amplitude_image_ptr)
{
    auto len = 240 * 180;
    for (int i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(depth_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

int main()
{
    ArducamTOFCamera tof;
    ArducamFrameBuffer *frame;
    std::time_t t;
    tm *nowtime;
    //serg init var <<
    bool record_started = false;
    bool buffer_started = false;
    bool voxelFilter_enabled = false;
    float LeafSizeX = 0.01;
    float LeafSizeY = 0.01;
    float LeafSizeZ = 0.01;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;
    CloudType::Ptr output_buffer(new CloudType);
    //serg init vat >>
    if (tof.init(Connection::CSI))
    {
        std::cerr << "initialization failed" << std::endl;
        exit(-1);
    }
    if (tof.start())
    {
        std::cerr << "Failed to start camera" << std::endl;
        exit(-1);
    }
    tof.setControl(ControlID::RANGE, MAX_DISTANCE);
    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[43200];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_ptr->points.push_back(pcl::PointXYZ(10, 10, 4));
    cloud_ptr->width = cloud_ptr->size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
    vtkObject::GlobalWarningDisplayOff();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);
    // boost::thread vthread(&viewerRunner, viewer);
    char buff[60];
    const float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
    const float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
    unsigned long int frame_number = 0;
    
    for (;;)
    {
        frame = tof.requestFrame(200);

        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
            cv::Mat result_frame(180, 240, CV_8U, preview_ptr);
            cv::Mat amplitude_frame(180, 240, CV_32F, amplitude_ptr);
            if (record_started == false && buffer_started == false) {
                getPreview(preview_ptr, depth_ptr, amplitude_ptr);                

                cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);
                cv::resize(result_frame, result_frame, cv::Size(720, 540));
                cv::imshow("preview", result_frame);
            }
            cloud_ptr->clear();

            unsigned long int pos = 0;
            for (int row_idx = 0; row_idx < 180; row_idx++)
                for (int col_idx = 0; col_idx < 240; col_idx++,pos++)
            {
                if (amplitude_ptr[pos] > 30)
                {
                    float zz = depth_ptr[pos];

                    float xx = (((120 - col_idx)) / fx) * zz;
                    float yy = ((90 - row_idx) / fy) * zz;
                    pcl::PointXYZ ptemp(xx, yy, zz);
                    cloud_ptr->points.push_back(ptemp);
                }
                else
                {
                    pcl::PointXYZ ptemp(0, 0, 0);
                    cloud_ptr->points.push_back(ptemp);
                }
            }
            cloud_ptr->width = cloud_ptr->points.size();
            cloud_ptr->height = 1;
            cloud_ptr->is_dense = false;
            frame_number += 1;
            //apply voxel filter <<
            if (voxelFilter_enabled == true) {
                std::cout << "Number of points before: " << cloud_ptr->points.size() << std::endl;

                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
                voxel_grid.setInputCloud (cloud_ptr);
                voxel_grid.setLeafSize(LeafSizeX, LeafSizeY, LeafSizeZ);
                voxel_grid.filter(*cloud_ptr);

                std::cout << "Number of points after: " << cloud_ptr->points.size() << " LeafSizeX=" <<LeafSizeX << " LeafSizeY=" << LeafSizeY << " LeafSizeZ=" << LeafSizeZ << std::endl;
            }            
            //apply voxel filter >>
            //save frame to file if record started
            if (record_started == true) {
                t = std::time(0);
                nowtime = localtime(&t);
                sprintf(buff, "record/sensor_%d%d%d%d%d%d_%d.pcd", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,nowtime->tm_mday,nowtime->tm_hour + 1,nowtime->tm_min + 1,nowtime->tm_sec + 1,frame_number);
                pcl::io:: savePCDFileASCII(buff, *cloud_ptr); 
            }
            // save frame to buffer if buffer enabled
            if (buffer_started == true) {
                *output_buffer += *cloud_ptr;
            }

            boost::mutex::scoped_lock updateLock(updateModelMutex);            
            // do not update GUI if record or buffer enabled
            if (record_started == false && buffer_started == false) {
                viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");            
            }
            updateLock.unlock();
            // do not update GUI if record or buffer enabled
            if (record_started == false && buffer_started == false) {
                viewer->spinOnce(100);
            }
            boost::this_thread::sleep(boost::posix_time::microseconds(100));

            switch (cv::waitKey(1))
            {
            case 's':
                t = std::time(0);
                nowtime = localtime(&t);
                sprintf(buff, "image_%d%d%d%d%d%d_%d.png", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,nowtime->tm_mday,nowtime->tm_hour + 1,nowtime->tm_min + 1,nowtime->tm_sec + 1,frame_number) ;
                cv::imwrite(buff, result_frame);
                std::cout << "save image!" << std::endl;
                break;
            case 'q':
                cv::destroyAllWindows();
                viewer->close();
                // vthread.join();
                tof.stop();
                exit(0);
                break;
            case 'd':
                t = std::time(0);
                nowtime = localtime(&t);
                sprintf(buff, "sensor_%d%d%d%d%d%d_%d.pcd", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,nowtime->tm_mday,nowtime->tm_hour + 1,nowtime->tm_min + 1,nowtime->tm_sec + 1,frame_number);
                pcl::io:: savePCDFileASCII(buff, *cloud_ptr);                
               
                std::cout << "save pcd!" << std::endl;
                //
                break;
            case 'r':
                record_started = !record_started;
                if (record_started == true) {
                    std::cout << "Record started!" << std::endl;
                } else {
                    std::cout << "Record stopped!" << std::endl;
                }

                //
                break;
            case 'b':
                buffer_started = !buffer_started;
                if (buffer_started == true) {
                    std::cout << "buffer started!" << std::endl;
                } else {
                    std::cout << "buffer stopped!" << std::endl;
                }
                
                break;
            case 'v':
                voxelFilter_enabled = !voxelFilter_enabled;
                if (voxelFilter_enabled == true) {
                    std::cout << "voxelFilter started!" << std::endl;
                } else {
                    std::cout << "voxelFilter stopped!" << std::endl;
                }
                
                break;
            case 'w':
                std::cout << "write to file output_buffer pcd started. Please wait!" << std::endl;
                t = std::time(0);
                nowtime = localtime(&t);
                sprintf(buff, "output_buffer_%d%d%d%d%d%d_%d.pcd", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,nowtime->tm_mday,nowtime->tm_hour + 1,nowtime->tm_min + 1,nowtime->tm_sec + 1,frame_number);
                pcl::io:: savePCDFileASCII(buff, *output_buffer);                               
                std::cout << "write to file output_buffer pcd completed!" << std::endl;
                //
                break;
            case 'y':
                LeafSizeX += 0.001;                
                break;             
            case 'u':
                LeafSizeY += 0.001;
                break;             
            case 'i':
                LeafSizeZ += 0.001;
                break;
            case 'h':
                LeafSizeX -= 0.001;
                break;             
            case 'j':
                LeafSizeY -= 0.001;
                break;             
            case 'k':
                LeafSizeZ -= 0.001;
                break;                                                                                                                   
            case 'c':
            // try concantenate clouds <<
            //typedef pcl::PointXYZ PointType;
            //typedef pcl::PointCloud<PointType> CloudType;

            // Load the PCD files
            CloudType::Ptr cloud1(new CloudType);
            CloudType::Ptr cloud2(new CloudType);
            pcl::io::loadPCDFile("cloud1.pcd", *cloud1);
            pcl::io::loadPCDFile("cloud2.pcd", *cloud2);

            // Put in one output cloud
            CloudType::Ptr output(new CloudType);
            *output += *cloud1;
            *output += *cloud2;
            //*output += *cloud_ptr;

            // Save the output file
            pcl::io::savePCDFileASCII("output.pcd", *output);
            // >>

            std::cout << "cloud1 + cloud1 > output pcd!" << std::endl;
                //>>
            }
        }
        tof.releaseFrame(frame);
    }
    return 0;
}
