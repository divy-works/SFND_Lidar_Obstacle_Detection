#include "../render/render.h"
#include "kdtree.h"
#include <set>

void proximity(const std::vector < std::vector <float> >& points, int id, std::vector <int> * cluster, std::set <int> * processedPointIds, KdTree * tree, float distanceTol)
{
	if (processedPointIds->count(id) == 0)
	{
		std::vector <int> nearbyIndices = tree->search(points[id], distanceTol);
		cluster->push_back(id);
		processedPointIds->insert(id);
		for (int i : nearbyIndices)
		{
			proximity(points, i, cluster, processedPointIds, tree, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::set <int> * processedPointIds = new std::set <int> ();
	for (int id = 0; id < points.size(); id++)
	{
		if (processedPointIds->count(id) == 0)
		{
			//create a cluster corresponding to the point
			std::vector <int> * cluster = new std::vector <int> ();
			proximity(points, id, cluster, processedPointIds, tree, distanceTol);
            if (cluster->size() >= minSize && cluster->size() <= maxSize)
            {
                clusters.push_back(*cluster);
            }
		}
	}
	return clusters;
}