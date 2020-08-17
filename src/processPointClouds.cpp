// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "./cluster/kdtree.h"
extern std::vector<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr, int, float);
extern std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>&, KdTree *, float, int, int);

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr downsampledCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>());
    
    //downsample the input cloud data using the voxel dimension defined by filterRes
    pcl::VoxelGrid<PointT> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelFilter.filter(*downsampledCloud);

    //crop the region defined by min and max
    pcl::CropBox<PointT> cropBox;
    cropBox.setInputCloud(downsampledCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*croppedCloud);

    //remove the lidar points coming from the roof of the car
    std::vector<int> indices;
    pcl::CropBox<PointT> cropRoof;
    cropRoof.setInputCloud(croppedCloud);
    cropRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cropRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    cropRoof.filter(indices);//indices of the point cloud containing lidar points coming from the roof

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int index: indices)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(croppedCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>), planeCloud(new pcl::PointCloud<PointT>);
    //create filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());


    std::vector<int> inlierIndices = Ransac(cloud, maxIterations, distanceThreshold);
    for (int i : inlierIndices)
        inliers->indices.push_back(i);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree * tree = new KdTree();
    std::vector<std::vector<float>> points;
    for (int idx = 0; idx < cloud->size(); idx ++)
    {
        std::vector<float> point = {cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
        tree->insert(point, idx);
        points.push_back(point);
    }
  
    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);

    for (std::vector<std::vector<int>>::iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());
        for (std::vector<int>::iterator pit = it->begin(); pit != it->end(); ++pit)
        {
            cloud_cluster->push_back((*cloud)[*pit]);
        }
        cloud_cluster->width = it->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}