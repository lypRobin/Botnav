#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/point_cloud.h"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class PointCloudFilter{
protected:
    pcl::VoxelGrid<PointT> *sor_voxel;
    pcl::StatisticalOutlierRemoval<PointT> *sor_statistic;
public:
    PointCloudFilter()
    {
        sor_voxel = new pcl::VoxelGrid<PointT>;
        sor_statistic = new pcl::StatisticalOutlierRemoval<PointT>;
    }

    ~PointCloudFilter()
    {
        delete sor_voxel;
        delete sor_statistic;
    }

public:
    void setInputCloud(const PointCloudT::ConstPtr &cloud)
    {
        sor_statistic->setInputCloud (cloud);
        sor_voxel->setInputCloud (cloud);
    }

    void setParameters()
    {
        sor_statistic->setMeanK (50);
        sor_statistic->setStddevMulThresh (1.0);

        sor_voxel->setLeafSize(0.01f, 0.01f, 0.01f);
    }

    void pointFilter(PointCloudT::Ptr filtered_cloud)
    {
        sor_voxel->filter(*filtered_cloud);
        sor_statistic->filter(*filtered_cloud);

    }

};


boost::mutex cloud_mutex;
bool cloud_available_flag;
// Caputre point cloud data from kinect.
class CaptureCloud{

public:
    void capture_callback(const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr &cloud)
    {
        cloud_mutex.lock();
        *cloud = *callback_cloud;
        cloud_available_flag = true;
        cloud_mutex.unlock();
    }

public:
    void run()
    {
        cloud_available_flag = false;
        PointCloudT::Ptr cloud (new PointCloudT);
        PointCloudT::Ptr filter_cloud(new PointCloudT);
        PointCloudFilter filter;

        pcl::Grabber* interface = new pcl::OpenNIGrabber();
        boost::function<void (const PointCloudT::ConstPtr&)> f = 
          boost::bind(&CaptureCloud::capture_callback, this, _1, cloud);
        
        interface->registerCallback(f);
        interface->start();

        pcl::visualization::PCLVisualizer viewer("PCL origin");
        viewer.setCameraPosition(0,0,-2,0,-1,0,0);

        while(!cloud_available_flag)
           boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        
        cloud_available_flag = false;

        int i = 0;
        while(ros::ok()) 
        {
            if(10 == i)
            {
                std::cout << "Save PCD file done!" << std::endl;
                break;
            }

            if(cloud_available_flag && cloud_mutex.try_lock())
            {
                cloud_available_flag = false;
              
                viewer.removeAllPointClouds();
                viewer.removeAllShapes();
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
                viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");

                filter.setInputCloud(cloud);
                filter.setParameters();
                filter.pointFilter(filter_cloud);

                std::stringstream ss;
                ss << "test_pointcloud_" << i << ".pcd";
                pcl::io::savePCDFileASCII(ss.str(), *filter_cloud);

                std::cout << "Saving the No." << i << " PCD file." 
                    << "The total points is: " << filter_cloud->points.size() << "." 
                << std::endl;

                viewer.spinOnce(100);

                cloud_mutex.unlock();
                i++;
            }
            
        }

        interface->stop();

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_test");
    CaptureCloud cap;
    cap.run();
    ros::spin();
    return 0;
}





