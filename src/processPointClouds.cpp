// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    //std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (filterRes, filterRes, filterRes);
  	sor.filter (*cloudFiltered);

  	typename pcl::PointCloud<PointT>::Ptr cloudCropped (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> boxFilter;
  	boxFilter.setMin(minPoint);
	boxFilter.setMax(maxPoint);
	boxFilter.setInputCloud(cloudFiltered);
	boxFilter.filter(*cloudCropped);
  
  	std::vector<int> indices;
  	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -2.5, 1.0));
	roof.setInputCloud(cloudCropped);
	roof.filter(indices);
  
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for(int point : indices)
      inliers->indices.push_back(point);
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud (cloudCropped);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*cloudCropped);
  	
  	

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudCropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>);
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);
  	
  // Extract the inliers
  
   // Create the filtering object
  	pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
  
  	// Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);
    //cloud_filtered.swap (cloud_f);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::SACSegmentation<PointT> seg;
  	
  	seg.setOptimizeCoefficients (true);

  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setDistanceThreshold (0.01);

  	seg.setInputCloud (cloud);
  	seg.segment (*inliers, *coefficients);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

/////////////////// MY CODE STARTS ////////////////////////////
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	 int i{ 0 };
    KdTree *kdTree{ new KdTree() };

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points) {
        const std::vector<float> p{ point.x, point.y, point.z };
        kdTree->insert(p, i++);
        points.push_back(p);
    }

    const std::vector<std::vector<int>> indicesList{ euclideanCluster(points, kdTree, clusterTolerance) };

    
    for (auto indices : indicesList) {

        if (indices.size() < minSize || indices.size() > maxSize)
        { 
          	continue;// out of bounds
        }

        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };

        for (auto index : indices)
        { 	
          cluster->points.push_back(cloud->points[index]); 
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
  	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
  	// Return indicies of inliers from fitted line with most inliers
    
  	for(int itnum = maxIterations; itnum > 0; itnum --)
    {
	   	std::unordered_set<int> inliersCur;
      	while(inliersCur.size() < 3)
         inliersCur.insert(rand()%(cloud->points.size()));
     
		
      	float x1, x2,x3, y1, y2, y3, z1, z2, z3;
      	
      	auto itrCur = inliersCur.begin();

        x1 = cloud->points.at((*itrCur)).x;
       	y1 = cloud->points.at((*itrCur)).y;
      	z1 = cloud->points.at((*itrCur)).z;
        itrCur++;
        x2 = cloud->points.at((*itrCur)).x;
      	y2 = cloud->points.at((*itrCur)).y;
      	z2 = cloud->points.at((*itrCur)).z;
      	itrCur++;
        x3 = cloud->points.at((*itrCur)).x;
      	y3 = cloud->points.at((*itrCur)).y;
      	z3 = cloud->points.at((*itrCur)).z;
       
      
      	float coA = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
      	float coB = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
      	float coC = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1) ;
       	float coD = -1 * (coA*x1 + coB*y1 + coC*z1);
      
     
      for(int indx =0; indx <cloud->points.size(); indx++)
      {
        float curX = cloud->points.at(indx).x;
        float curY = cloud->points.at(indx).y;
        float curZ = cloud->points.at(indx).z;
        float dist = std::fabs(coA*curX+coB*curY+coC*curZ+coD)/std::sqrt(coA*coA+coB*coB+coC*coC);
        
        if(dist < distanceTol)
          inliersCur.insert(indx);    
      }
       	int numInliers = 0;
      	numInliers = inliersCur.size();
      	if(numInliers > inliersResult.size())
           inliersResult = inliersCur;
     
    } 
	//return inliersResult;
    typename pcl::PointCloud<PointT>::Ptr cloudInliers{ new pcl::PointCloud<PointT>() };
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers{ new pcl::PointCloud<PointT>() };

    if (!inliersResult.empty()) {
        for (int index{ 0 }; index < cloud->points.size(); index++) {
            const PointT point{ cloud->points.at(index) };

            if (inliersResult.count(index)) {
                cloudInliers->points.push_back(point);
            } else {
                cloudOutliers->points.push_back(point);
            }
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        cloudOutliers, cloudInliers);
}

template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(std::vector<std::vector<float>> points, int id, std::vector<bool>& processedPoints, std::vector<int>& cluster, KdTree* tree, float distanceTol)
{
  std::vector<int> nearby;
  
  if (processedPoints[id] == false)
  {
    processedPoints[id] = true;
    cluster.push_back(id);
    nearby = tree->search(points[id],distanceTol);
    
    
      for(int itNearby = 0; itNearby < nearby.size() ; ++itNearby)
      {
        Proximity(points, nearby[itNearby], processedPoints, cluster, tree, distanceTol);
      }
  }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
  	
  	std::vector<bool> processedPoints(points.size(),false);
  
  	for(int pntNo = 0; pntNo < points.size(); pntNo++)
    {
		if(processedPoints[pntNo] == false)
        {
          std::vector<int> cluster;
          Proximity(points, pntNo, processedPoints, cluster, tree, distanceTol);
          clusters.push_back(cluster);
        }      
    }
  
	return clusters;
}

/////////////////// MY CODE ENDS ////////////////////////////

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


