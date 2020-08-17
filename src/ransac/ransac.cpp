#include <set>
#include <unordered_set>
#include "../render/render.h"
#include <cmath>

std::vector<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::set <int> sampleIndices;
    std::vector<int> inliersResult, inliers;
    srand(time(NULL));
    pcl::PointXYZI currentPoint;
    float A, B, C, D, distance;
    for (int i = 0; i < maxIterations; i++)
    {
        sampleIndices.clear();
        inliers.clear();
        //randomly select three points on the cloud and find coefficients of plane passing through them
        while (sampleIndices.size() < 3)
        {
            sampleIndices.insert(rand()%(cloud->size()));
        }

        std::vector<pcl::PointXYZI> points;
        for (std::set<int>::iterator it = sampleIndices.begin(); it != sampleIndices.end(); ++ it)
        {
            points.push_back(cloud->points[*it]);
        }

 
        A = (points[1].y - points[0].y)*(points[2].z - points[0].z) - (points[1].z - points[0].z)*(points[2].y - points[0].y);
        B = (points[1].z - points[0].z)*(points[2].x - points[0].x) - (points[1].x - points[0].x)*(points[2].z - points[0].z);
        C = (points[1].x - points[0].x)*(points[2].y - points[0].y) - (points[1].y - points[0].y)*(points[2].x - points[0].x);
        D = -(A*points[0].x + B*points[0].y + C*points[0].z);


        for (int j = 0; j < cloud->size(); j++) //iterate through all the points in the cloud
        {
            if (sampleIndices.count(j) == 0) //check that the index is not same as the sample indices of the point contained in the plane
            {
                currentPoint = cloud->points[j];
                distance = fabs(A*currentPoint.x + B*currentPoint.y + C*currentPoint.z + D)/sqrt(A*A + B*B + C*C);
                if (distance < distanceTol)
                {
                    inliers.push_back(j);
                }
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    return inliersResult;
}