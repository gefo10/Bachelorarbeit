#include <ScannerLib/KdTree.h>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <sstream>
#include <cstdlib>

KdTree::KdTree(std::vector<Point>& cloud)
{
    if(cloud.empty())
        throw std::logic_error("Point Cloud is empty! KdTree can not be constructed");
    
    //std::vector<Point> points;
    //for(int i =0; i < cloud.size(); i++)
    //    points.push_back(*cloud[i]);

    _root  = new Node(cloud[0],ALLIGNMENT::ALLIGN_X);
    build(cloud,1,cloud.size());
}
KdTree::KdTree(std::vector<Point*>& cloud)
{
    if(cloud.empty())
        throw std::logic_error("Point Cloud is empty! KdTree can not be constructed");
    
    //std::vector<Point> points;
    //for(int i =0; i < cloud.size(); i++)
    //    points.push_back(*cloud[i]);

    _root  = new Node(*cloud[0],ALLIGNMENT::ALLIGN_X);
    build(cloud,1,cloud.size());
}
KdTree::~KdTree() {
    //recursively clear left and right children
    help_clear(_root);
    delete _root;
}

void KdTree::help_clear(Node* n) {
    if(n->left == nullptr && n->right == nullptr)
        return;
    else if(n->left != nullptr && n->right != nullptr){
        help_clear(n->left);
        help_clear(n->right);
        delete n->right;
        delete n->left;
    }
    else if(n->left == nullptr){
        help_clear(n->right);
        delete n->right;
    }
    else{
        help_clear(n->left);
        delete n->left;
    }
}
void KdTree::build(std::vector<Point>& cloud, int current, size_t size)
{
   for(int i = current; i < size; i++){
        Point& p = cloud[i];
    //std::cout << "p: " << p.x << " " << p.y << " " << p.z << std::endl << std::flush;
        insertNode((*_root),p);

   }   
    
}

void KdTree::build(std::vector<Point*>& cloud, int current, size_t size)
{
   for(int i = current; i < size; i++){
        Point p = *cloud[i];
        insertNode((*_root),p);

   }   
    
}
void KdTree::insertNode(Node& node,Point& new_point)
{

        ALLIGNMENT allignment = node.allignment;
        
        bool left = false;

        switch(allignment) {
          case ALLIGNMENT::ALLIGN_X:
              left = node.current->x > new_point.x;
              break;
          case ALLIGNMENT::ALLIGN_Y:
              left = node.current->y > new_point.y;
              break;
          case ALLIGNMENT::ALLIGN_Z:
              left = node.current->z > new_point.z;
              break;
          default:
              break;
        }


        if(left){
           if( node.left ){
               
               //std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl << std::flush;
               insertNode(*node.left,new_point);
           }
           else {
                //std::cout << "BBBBBBBBBBBBBBBBBBBBBBBB" << std::endl << std::flush;
                node.left = new Node(new_point,getNextAllignment(allignment));
           }

        } else {

           if ( node.right){
               
               //std::cout << "CCCCCCCCCCCCCCCCCCCCCCC" << std::endl << std::flush;
               insertNode( *node.right,new_point);
           }
           else {
               //std::cout << "DDDDDDDDDDDDDDDDDDDDDDDDDD" << std::endl << std::flush;
               node.right  = new Node(new_point,getNextAllignment(allignment));
               
           }
        }

}

void KdTree::search(Point& p, Point& result) 
{
    using namespace std;

   //std::cout << "SEARCH NEAREST CHILD "<< std::endl << std::flush;

    result = searchNearestChild(*_root, p);

  //std::cout << "SEARCH NEAREST CHILD DONE "<< std::endl << std::flush;
    float radius = sqrt(pow(p.x - result.x, 2.0) + pow(p.y - result.y, 2.0) + pow(p.z - result.z, 2.0));

  //std::cout << "RADIUS CALCULATES " << radius<< std::endl << std::flush;
    radiusSearch(p,radius,result);

  //std::cout << "SEARCH NEAREST CHILD "<< std::endl << std::flush;
}
void KdTree::radiusSearch(Point& p,float searchRadius, Point& result)
{
    radiusSearch(*_root,p,searchRadius,result);
}
void KdTree::radiusSearch(Node& node, Point& p,float& searchRadius,Point& result)
{
    using namespace std;

    if(node.current == nullptr) return;
    if (node.left == nullptr && node.right == nullptr) //then its a leaf
    {

    //    std::cout << "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD" << searchRadius << std::endl << std::flush;
        float distance = sqrt(pow(node.current->x - p.x, 2.0) + pow(node.current->y - p.y, 2.0) + pow(node.current->z - p.z, 2.0));
        if (distance < searchRadius)
        {
            searchRadius = distance;
            //result.x = node.current->x;
            //result.y = node.current->y;
            //result.z = node.current->z;
            result = *(node.current);
        }   
    }   
    else    
    {
        float distance_OnAllignAxis;
        float p_Axis;
        float node_Axis;
        
        switch(node.allignment) {
            case ALLIGNMENT::ALLIGN_X:
                distance_OnAllignAxis = abs(node.current->x-p.x);
                p_Axis = p.x;
                node_Axis = node.current->x;
                break;
            case ALLIGNMENT::ALLIGN_Y:
                distance_OnAllignAxis = abs(node.current->y-p.y);
                p_Axis = p.y;
                node_Axis = node.current->y;
                break;
            case ALLIGNMENT::ALLIGN_Z:
                distance_OnAllignAxis = abs(node.current->z-p.z);
                p_Axis = p.z;
                node_Axis = node.current->z;
                break;

        }

        if (distance_OnAllignAxis < searchRadius)
        {
            float distance = sqrt(pow(node.current->x - p.x, 2.0) + pow(node.current->y - p.y, 2.0) + pow(node.current->z - p.z, 2.0));
            if (distance < searchRadius)
            {
                searchRadius = distance;
                result.x = node.current->x;
                result.y = node.current->y;
                result.z = node.current->z;
               // result = *node.current;
            } 


      //     std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << searchRadius << std::endl << std::flush;
           if(node.left != nullptr)
               radiusSearch(*(node.left), p, searchRadius, result);
        //   std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << searchRadius <<std::endl << std::flush;
           if(node.right != nullptr)
               radiusSearch(*(node.right), p, searchRadius, result); 

           
          // std::cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << std::endl << std::flush;
        }   
        else
        {

            if (p_Axis >= node_Axis)
            {
                float distance = sqrt(pow(node.current->x - p.x, 2.0) + pow(node.current->y - p.y, 2.0) + pow(node.current->z - p.z, 2.0));
                if (distance < searchRadius)
                {
                    searchRadius = distance;
                    result.x = node.current->x;
                    result.y = node.current->y;
                    result.z = node.current->z;
                    //result = *(node.current);
                } 
                
                if(node.right != nullptr)
                    radiusSearch(*(node.right), p, searchRadius, result);
           //std::cout << "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG" << std::endl << std::flush;
            }
            else
            {
                float distance = sqrt(pow(node.current->x - p.x, 2.0) + pow(node.current->y - p.y, 2.0) + pow(node.current->z - p.z, 2.0));
                if (distance < searchRadius)
                {
                    searchRadius = distance;
                    result = *(node.current);
                } 


                if(node.left != nullptr) 
                    radiusSearch(*(node.left), p, searchRadius, result);
           //std::cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF" << std::endl << std::flush;
            }

        }
    }

}


Point  KdTree::searchNearestChild(Node& node,Point& p)
{
    if (node.left == nullptr && node.right == nullptr) //then its a leaf
    {
        return *node.current;
    }  
    else    
    {
        float p_Axis;
        float node_Axis;
        switch(node.allignment) {
            case ALLIGNMENT::ALLIGN_X:
                p_Axis = p.x;
                node_Axis = node.current->x;
                break;
            case ALLIGNMENT::ALLIGN_Y:
                p_Axis = p.y;
                node_Axis = node.current->y;
                break;
            case ALLIGNMENT::ALLIGN_Z:
                p_Axis = p.z;
                node_Axis = node.current->z;
                break;
        }
        
        if (p_Axis >= node_Axis && node.right != nullptr)
           return searchNearestChild(*node.right, p);
        else if(p_Axis < node_Axis && node.left != nullptr)
           return  searchNearestChild(*node.left,p);
        else if(node.right != nullptr)
           return searchNearestChild(*node.right, p);
        else if(node.left != nullptr) 
           return searchNearestChild(*node.left, p);
        else
            return *node.current;

    }

}

ALLIGNMENT KdTree::getNextAllignment (ALLIGNMENT allign)
{
    switch(allign) {
      case ALLIGNMENT::ALLIGN_X:
          return ALLIGNMENT::ALLIGN_Y;
      case ALLIGNMENT::ALLIGN_Y:
          return ALLIGNMENT::ALLIGN_Z;
      case ALLIGNMENT::ALLIGN_Z:
          return ALLIGNMENT::ALLIGN_X;
      default:
          return ALLIGNMENT::ALLIGN_X;
    }

}

//ALLIGNMENT getNextAllignment()
//{
//    switch( currentDepth % 3 ) {
//      case 0:
//          return ALLIGN_Y;
//      case 1:
//          return ALLIGN_Z;
//      case 2:
//          return ALLIGN_X;
//    }
//}
