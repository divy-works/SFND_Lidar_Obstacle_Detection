/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

    void addNode(std::vector<float> point, int depth, int id)
    {
        int cd = depth % 3;

        if (point[cd] > this->point[cd])
        {
            if (this->right == NULL)
            {
                this->right = new Node(point, id);
            }
            else
            {
                this->right->addNode(point, depth + 1, id);
            }
        }
        else
        {
            if (this->left == NULL)
            {
                this->left = new Node(point, id);
            }
            else
            {
                this->left->addNode(point, depth + 1, id);
            }   
        }
    }

	bool insideBox(std::vector <float> target, std::vector <float> refPoint, float distanceTol)
	{
		float x_upperbound = refPoint[0] + distanceTol;
		float x_lowerbound = refPoint[0] - distanceTol;
		float y_upperbound = refPoint[1] + distanceTol;
		float y_lowerbound = refPoint[1] - distanceTol;
        float z_upperbound = refPoint[2] + distanceTol;
        float z_lowerbound = refPoint[2] - distanceTol;

		return (bool)(target[0] <= x_upperbound && target[0] >= x_lowerbound && 
                      target[1] <= y_upperbound && target[1] >= y_lowerbound &&
                      target[2] <= z_upperbound && target[2] >= z_lowerbound);
	}

	std::vector <int> search(std::vector <float> target, float distanceTol, int depth)
	{
		std::vector <int> ids, rightChildrenIds, leftChildrenIds;
		if (insideBox(target, this->point, distanceTol))
		{
			//check the distance of reference from the target point
			float distance = sqrt(pow((target[0] - this->point[0]), 2) + 
                                  pow((target[1] - this->point[1]), 2) + 
                                  pow((target[2] - this->point[2]), 2));
			if (distance <= distanceTol)
			{
				ids.push_back(this->id);
			}
		}

		//search left or right, depends on depth
		int dim = depth % 3;
		if (target[dim] - this->point[dim] < distanceTol)
		{
			//search left
			if (this->left)
			{
				std::vector <int> leftChildrenIds = this->left->search(target, distanceTol, depth + 1);
				for (int id : leftChildrenIds)
					ids.push_back(id);
			}
				
		}
		if (this->point[dim] - target[dim] < distanceTol)
		{
			//search right
			if (this->right)
			{
				std::vector <int> rightChildrenIds = this->right->search(target, distanceTol, depth + 1);
				for (int id : rightChildrenIds)
					ids.push_back(id);
			}
				
		}
		return ids;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        if (this->root == NULL)
        {
            this->root = new Node(point, id);
        }
        else
        {
            this->root->addNode(point, 0, id);
        }
        
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        ids = this->root->search(target, distanceTol, 0);
		return ids;
	}
};

