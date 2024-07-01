#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>

using namespace std;

int main ()
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    
    pcl::PCDReader reader;
    reader.read("../1-2-2023-combined.pcd", *cloud_blob);

    cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << endl;
    
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    cerr << "Point Cloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << endl;

    //write the downsampled version to disk
    pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ>("tsakla.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    //create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory might change this to normal plane TRY THIS LATER!!!!
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
    seg.setAxis(axis);
    seg.setEpsAngle(  30.0f * (3.1415/180.0f) );
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.5);

    //creat the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->size();
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        //segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size()==0)
        {
            cerr << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        cerr << "Pointcloud representing the planar component: " << cloud_p->width * cloud_p->height << " data  points." << endl;

        std::stringstream ss;
        ss << "../extracted_planes/1-2-2023-plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);

        // //visualize the planar parts
        // pcl::visualization::CloudViewer viewer("Planar points viewer");
        // viewer.showCloud(cloud_p);


        //create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;

    }
    return (0);
}