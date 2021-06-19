/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
  	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
  	// Return indicies of inliers from fitted line with most inliers
    std::unordered_set<int> inliersCur;
  	for(int itnum = maxIterations; itnum > 0; itnum --)
    {
	   	
      	while(inliersCur.size() < 3)
          inliersCur.insert(rand()%(cloud->points.size()));
      
      	float x1, x2,x3, y1, y2, y3, z1, z2, z3;
      	
      	auto itrCur = inliersCur.begin();

      	x1 = cloud->points[*itrCur].x;
      	y1 = cloud->points[*itrCur].y;
      	z1 = cloud->points[*itrCur].z;
        itrCur++;
        x2 = cloud->points[*itrCur].x;
      	y2 = cloud->points[*itrCur].y;
      	z2 = cloud->points[*itrCur].z;
      	itrCur++;
        x3 = cloud->points[*itrCur].x;
      	y3 = cloud->points[*itrCur].y;
      	z3 = cloud->points[*itrCur].z;
      
      	float coA = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
      	float coB = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
      	float coC = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1) ;
       	float coD = -1 * (coA*x1 + coB*y1 + coC*z1);
      
       int numInliers = 0;
       
      for(int indx =0; indx <cloud->points.size(); indx++)
      {
        float curX = cloud->points[indx].x;
        float curY = cloud->points[indx].y;
        float curZ = cloud->points[indx].z;
        float dist = fabs(coA*curX+coB*curY+coC*curZ+coD)/sqrt(coA*coA+coB*coB+coC*coC);
        
        if(dist < distanceTol)
          inliersCur.insert(indx);    
      }
      	numInliers = inliersCur.size();
      	if(numInliers > inliersResult.size())
           inliersResult = inliersCur;
     
    } 
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.4);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
