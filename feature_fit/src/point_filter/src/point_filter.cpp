#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/point_cloud.h"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;

class PointFilter{
private:

    bool cloud_available_flag;

     
public:
    void catpure_callback(const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr &cloud)
    {
        cloud_mutex.lock();
        *cloud = *callback_cloud;
        cloud_available_flag = true;
        
        static unsigned cnt = 0;
        static double last = pcl::getTime ();
        if(30 == ++cnt)
        {
            double now = pcl::getTime();
            double rate = (double)cnt / (double)(now - last);
            ROS_INFO("Average FrameRate is %f HZ.", rate);
            last = now;
            cnt = 0;
        }

       cloud_mutex.unlock();
    }
public:
    void run()
    {
        cloud_available_flag = false;

        PointCloudT::Ptr cloud (new PointCloudT);
        PointCloudT::Ptr voxel_cloud(new PointCloudT);
        PointCloudT::Ptr statistic_cloud(new PointCloudT);

        pcl::Grabber* interface = new pcl::OpenNIGrabber();
        boost::function<void (const PointCloudT::ConstPtr&)> f = 
          boost::bind(&PointFilter::catpure_callback, this, _1, cloud);
        
        interface->registerCallback(f);
        interface->start();

        pcl::visualization::PCLVisualizer viewer("PCL origin");
        viewer.setCameraPosition(0,0,-2,0,-1,0,0);

        // pcl::visualization::PCLVisualizer voxel_viewer("PCL voxel");
        // voxel_viewer.setCameraPosition(0,0,-2,0,-1,0,0);

        pcl::visualization::PCLVisualizer statistic_viewer("PCL statistic");
        statistic_viewer.setCameraPosition(0,0,-2,0,-1,0,0);

        pcl::VoxelGrid<PointT> sor_voxel;
        pcl::StatisticalOutlierRemoval<PointT> sor_statistic;

        while(!cloud_available_flag)
           boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        
        cloud_available_flag = false;

        while(ros::ok()) 
        {
            if(cloud_available_flag && cloud_mutex.try_lock())
            {
                cloud_available_flag = false;
                viewer.removeAllPointClouds();
                viewer.removeAllShapes();
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
                viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");

                sor_statistic.setInputCloud (cloud);
                sor_statistic.setMeanK (50);
                sor_statistic.setStddevMulThresh (1.0);

                double start = pcl::getTime();
                sor_statistic.filter (*statistic_cloud);
                double t = pcl::getTime() - start;
                std::cout << "filter time is: " << t << std::endl;

                // sor_voxel.setInputCloud(cloud);
                // sor_voxel.setLeafSize(0.1f, 0.1f, 0.1f);
                // sor_voxel.filter(*voxel_cloud);

                // voxel_viewer.removeAllPointClouds();
                // voxel_viewer.removeAllShapes();
                // pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> voxel_geo(voxel_cloud);
                // voxel_viewer.addPointCloud<PointT> (voxel_cloud, voxel_geo, "voxel_cloud");

                statistic_viewer.removeAllPointClouds();
                statistic_viewer.removeAllShapes();
                pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> statistic_geo(statistic_cloud);
                statistic_viewer.addPointCloud<PointT> (statistic_cloud, statistic_geo, "statistic_cloud");

                viewer.spinOnce(100);

                cloud_mutex.unlock();
            }
            
        }
           
        interface->stop();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_test");
    PointFilter pf;
    pf.run();
    ros::spin();
    return 0;
}





