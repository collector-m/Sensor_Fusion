/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
	{
		if(*node ==NULL)
			*node = new Node<PointT>(point,id);
		else
		{
			uint cd = depth % 3;
			switch(cd)
			{
				case 0:
				{
					if(point.x<((*node)->point.x))
					  insertHelper(&((*node)->left), depth +1, point, id);
					else
					  insertHelper(&((*node)->right), depth +1, point, id);
					break;
				}
				case 1:
				{
					if(point.y<((*node)->point.y))
					  insertHelper(&((*node)->left), depth +1, point, id);
					else
					  insertHelper(&((*node)->right), depth +1, point, id);
					break;
				}
				case 2:
				{
					if(point.z<((*node)->point.z))
					  insertHelper(&((*node)->left), depth +1, point, id);
					else
					  insertHelper(&((*node)->right), depth +1, point, id);
				}
			}
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root,0,point,id );

	}

	void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
    if(node!=NULL)
		{

			float distance_x = node->point.x - target.x;
			float distance_y = node->point.y - target.y;
			float distance_z = node->point.z - target.z;
			if((-distanceTol<= distance_x && distance_x<=distanceTol)&&
			   (-distanceTol<= distance_y && distance_y<=distanceTol)&&
			   (-distanceTol<= distance_z && distance_z<=distanceTol))
			{
        float distance = sqrt(distance_x * distance_x +
					                    distance_y * distance_y +
															distance_z * distance_z);
				if(distance<=  distanceTol)
					ids.emplace_back(node->id);
			}

			uint cd = depth % 3;
			switch(cd){
				case 0:{
					if(-distanceTol<= distance_x)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
				  if( distance_x<=distanceTol)
						searchHelper(target, node->right, depth+1, distanceTol, ids);
					break;
				}
				case 1:{
					if(-distanceTol<= distance_y)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
				  if(distance_y<=distanceTol)
						searchHelper(target, node->right, depth+1, distanceTol, ids);
					break;
				}
				case 2:{
					if(-distanceTol<= distance_z)
						searchHelper(target, node->left, depth+1, distanceTol, ids);
					if(distance_z<=distanceTol)
						searchHelper(target, node->right, depth+1, distanceTol, ids);
					break;
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol,ids);
		return ids;
	}


};
