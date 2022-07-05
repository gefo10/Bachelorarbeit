#pragma once

#include <stdio.h>
#include <vector>
#include "Point.h"
#include <iostream>


 enum class ALLIGNMENT {
        ALLIGN_X,ALLIGN_Y,ALLIGN_Z
    };

struct Node {
        
        Point* current;
        //unsigned int index;
        ALLIGNMENT allignment;

        Node* left;
        Node* right;

        Node() {
            this->current = nullptr;
            this->left = nullptr;
            this->right = nullptr;
        }

        Node(Point& p, ALLIGNMENT allign){
            this->current = &p;
            this->allignment = allign;
            this->left = nullptr;
            this->right = nullptr;

        }

        //~Node(){
            //delete current;
            //delete left;
            //delete right;
        //}


        bool ChildrenEmpty() { return left == nullptr && right == nullptr; }
    };
    
class KdTree 
{
public:
    KdTree() = delete;
    KdTree(std::vector<Point>& cloud);
    KdTree(std::vector<Point*>& cloud);
   ~KdTree();
    ALLIGNMENT getNextAllignment(ALLIGNMENT allign);
    //ALLIGNMENT getNextAllignment();
     
    //KdTree(std::vector<Point>& cloud, int start, int end, SORT_ON sortOn);

    //virtual ~KdTree();
    void build(std::vector<Point>& cloud, int current,size_t size);
    void build(std::vector<Point*>& cloud, int current,size_t size);
    
    void search(Point& p,Point& result);
    void radiusSearch(Point& p, float searchRadius,Point& result);
    //void search(Point& p, Point& result);
    
    Node& getRoot() {return *_root;}
private:
    void insertNode(Node& node,Point& new_point);
    
    Point searchNearestChild(Node& node,Point& p);

    void radiusSearch(Node& node,Point& p, float& searchRadius, Point& result);
   // void insertionSort(std::vector<Point> &pointCloud, int start, int end, int sortOn);
   // void mergeSort(std::vector<Point> &pointCloud, int start, int end);
   // void merge(std::vector<Point> &pointCloud, int left, int mid, int right);
    void help_clear(Node* n);
    Node* _root;
    //unsigned int currentDepth;
};


