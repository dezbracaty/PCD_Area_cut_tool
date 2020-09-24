#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <vtkAreaPicker.h>
#include <string>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
typedef pcl::PointXYZ PointType;
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Carviewer"));

void areapickingcallback(const pcl::visualization::AreaPickingEvent &event,void *userdata)
{
    pcl::PointCloud<PointType>::Ptr  secloud(new pcl::PointCloud<PointType>());
    std::cout<<"Into here"<<std::endl;
    std::vector<int> indices;
    event.getPointsIndices(indices);
    pcl::IndicesPtr ind_plane=boost::make_shared<std::vector<int>>(indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);                       //导入点云数据
    extract.setIndices(ind_plane);                      //设置点云索引
    extract.setNegative(false);                  //设置为false，选择索引指向的点导出
    extract.filter(*secloud);                         //输出所选点云
    std::cout<<"Nums selected\t"<<secloud->points.size()<<std::endl;
    pcl::io::savePCDFile("car.pcd",*secloud);

}
int main() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Carviewer"));
    pcl::io::loadPCDFile("/home/allen/soft/VTK-PCL/pcl-pcl-1.9.1/test/car6.pcd",*cloud);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(cloud,"car");
    viewer->registerAreaPickingCallback(areapickingcallback);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
