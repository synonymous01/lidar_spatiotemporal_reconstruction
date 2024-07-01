#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
int main()
{
 // Load the input point cloud from a PCD file
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
 if
(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/ibrahim/reconstruction_work/12-1-2023-combined.pcd",
*cloud_rgb) == -1)
 {
 // If failed to load as RGB, try loading as XYZ
 if
(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ibrahim/reconstruction_work/12-1-2023-combined.pcd",
*cloud_xyz) == -1)
 {
 PCL_ERROR("Couldn't read the input PCD file\n");
 return -1;
 }
 }
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
 if (cloud_rgb->size() > 0)
{
 cloud = cloud_rgb;

 }
 else
 {
 cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
 cloud->width = cloud_xyz->width;
 cloud->height = cloud_xyz->height;
 cloud->is_dense = cloud_xyz->is_dense;
 cloud->points.resize(cloud_xyz->points.size());


 for (std::size_t i = 0; i < cloud_xyz->points.size(); ++i)
 {
 cloud->points[i].x = cloud_xyz->points[i].x;
 cloud->points[i].y = cloud_xyz->points[i].y;
 cloud->points[i].z = cloud_xyz->points[i].z;
 cloud->points[i].r = 255; // Set the color to white
 cloud->points[i].g = 255;
 cloud->points[i].b = 255;
 }
 }

 std::cerr << "Cloud imported. " << cloud->width * cloud->height << " data points found." << std::endl;
 // Region Growing segmentation
 pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>>(new pcl::search::KdTree<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
 ne.setSearchMethod(tree);
 ne.setInputCloud(cloud);
 ne.setKSearch(500);
ne.compute(*normals);

std::cerr << "Normals estimated. Cloud size: " << cloud->width * cloud->height << std::endl;
 pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> rg;
 rg.setMinClusterSize(500);
 rg.setMaxClusterSize(1000000);
 rg.setSearchMethod(tree);
 rg.setNumberOfNeighbours(500);
 rg.setInputCloud(cloud);
 rg.setInputNormals(normals);
 rg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
 rg.setCurvatureThreshold(0.8);
 std::vector<pcl::PointIndices> clusters;
 rg.extract(clusters);
 std::cerr << "Clusters extracted. Cloud size: " << cloud->width * cloud->height << std::endl;
 // Color each cluster with a random color if the cloud is RGB
 srand(time(NULL));
 for (std::size_t i = 0; i < clusters.size(); ++i)
 {
 unsigned char r = rand() % 256;
 unsigned char g = rand() % 256;
 unsigned char b = rand() % 256;
 for (std::size_t j = 0; j < clusters[i].indices.size(); ++j)
 {
 int idx = clusters[i].indices[j];
 cloud->points[idx].r = r;
 cloud->points[idx].g = g;
 cloud->points[idx].b = b;
 }
 }
 cerr << "Segmented. Now showing." << endl;
// Visualize the segmented point cloud with cluster labels
 pcl::visualization::CloudViewer viewer("Segmented Point CloudViewer");
 viewer.showCloud(cloud);

cerr << "Writing all the clusters to drive." << endl;
pcl::PCDWriter writer;
int j = 0;
for (const auto& cluster: clusters)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : cluster.indices)
    {
        cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height=1;
    cloud_cluster->is_dense = true;

    cout << "Pointclouds representing the cluster: " << cloud_cluster->size() << " data points" << endl;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << j;
    writer.write<pcl::PointXYZRGB> ("../extracted_planes12-1/planar_points_" + ss.str() + ".pcd", *cloud_cluster, false);
    j++;
    if (j >= 15) break;

}


// cerr << "Deleting everything but the plates..." << endl;
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr plates_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
// for (const auto& idx: clusters[3].indices)
// {
//     plates_cloud->push_back((*cloud)[idx]);
// }
// for (const auto& idx : clusters[6].indices)
// {
//     plates_cloud->push_back((*cloud)[idx]);
// }
// plates_cloud->width = plates_cloud->size();
// plates_cloud->height=1;
// plates_cloud->is_dense=true;
// writer.write<pcl::PointXYZRGB> ("../extracted_planes/plates_only9-2.pcd", *plates_cloud, false);

// cerr << "Plates extracted!" << endl;
 for (std::size_t i = 0; i < clusters.size(); ++i)
 {
 if (!clusters[i].indices.empty())
 {
 pcl::PointXYZRGB center_point = cloud->points[clusters[i].indices[0]];
 //viewer.addText3D(std::to_string(i), center_point, 0.02,1.0, 1.0, 1.0, std::to_string(i));
 }
 }
 while (!viewer.wasStopped())
 {
 // Wait for the viewer to close
 }
 return 0;
}
