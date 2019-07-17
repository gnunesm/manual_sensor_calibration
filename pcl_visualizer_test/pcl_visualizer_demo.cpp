#include <iostream>
// #include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
    
// void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = *reinterpret_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr*> (viewer_void);
//    std::cout << "Picking event active" << std::endl;
//    if(event.getPointIndex()!=-1)
//    {
//        float x,y,z;
//        event.getPoint(x,y,z);
//        std::cout << x<< ";" << y<<";" << z << std::endl;
//        pcl::PointXYZ basic_point;
//         basic_point.x = 0.1;
//         basic_point.y = 0.1;
//         basic_point.z = 0.1;
//        cloud->points.push_back(basic_point);
//    }
// }

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
int pointIndex;

void pointPickingOccured( const pcl::visualization::PointPickingEvent &event,void* viewer_void)
{
    float x,y,z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = *reinterpret_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr*> (viewer_void);
    event.getPoint(x,y,z);
    pointIndex = event.getPointIndex();
    std::cout <<"Point No. " << pointIndex <<" ";
    std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;

    viewer->updateSphere(cloud->points[pointIndex], 0.003, 255, 0, 0, "pt");
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
//   viewer->registerPointPickingCallback(pp_callback, NULL);
  viewer->registerPointPickingCallback(pointPickingOccured, static_cast<void*> (&cloud));
  viewer->addSphere(cloud->points[pointIndex], 0.003, "pt", 0); viewer->spin();
  return (viewer);
}

int 
main ()
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("cloud_36.pcd", *cloud_blob);
    // reader.read ("table_scene_lms400_plane_9.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (cloud_blob);
    // sor.setLeafSize (0.1f, 0.1f, 0.1f);
    // sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud_filtered);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // viewer->updatePointCloud(cloud_filtered)
        // std::this_thread::sleep_for(100ms);
    }
}