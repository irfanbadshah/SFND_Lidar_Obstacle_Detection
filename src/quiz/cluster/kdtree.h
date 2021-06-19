/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
  	void insertDeeper(Node *&curNode, int curDepth, std::vector<float> point, int id)
    {
      if(curNode == NULL)
      { //when at start and no root exists or last node of tree branch hass been reached
        curNode = new Node(point,id); //assign memeory to NULL pointer and assign point and ID value using constructor
      }
      else if(point[curDepth%2] < curNode->point[curDepth%2]) 
       	insertDeeper( curNode->left, curDepth + 1, point, id);
      
      else 
        insertDeeper( curNode->right, curDepth + 1, point, id);
    }
  
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
      
      	// creating recurrsive list
       int startDepth = 0;
       insertDeeper( root, startDepth, point, id);

	}
	
  	void searchDeeper( Node* curNode, int curDepth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
    {
		 
      	  
     if(curNode!= NULL)
      {
          float xDistDiff = fabs(target[0] - curNode->point[0]);
      	  float yDistDiff = fabs(target[1] - curNode->point[1]);
       
      	  if((xDistDiff <= distanceTol) && (yDistDiff <= distanceTol))
          {
            if(sqrt((xDistDiff*xDistDiff)+(yDistDiff*yDistDiff)) < distanceTol)
            {
              //std::cout<<"dist = "<<sqrt((xDistDiff*xDistDiff)+(yDistDiff*yDistDiff))<<std::endl;
              ids.push_back(curNode->id);
            }
          }
         
       
      	  if((target[curDepth%2] - distanceTol) < curNode->point[curDepth%2])
         	 searchDeeper( curNode->left, curDepth+1, target, distanceTol, ids);
     
          if((target[curDepth%2] + distanceTol) > curNode->point[curDepth%2])
            	searchDeeper( curNode->right, curDepth+1, target, distanceTol, ids);
      
       }
     }
  
  
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		
      	int curDepth = 0;
      	searchDeeper( root, curDepth, target,distanceTol, ids);
      	return ids;
	}
	

};




