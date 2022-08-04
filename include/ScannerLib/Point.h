#pragma once 


struct Point {
   float x;
   float y;
   float z;
   float r=1.0f;
   float g=1.0f;
   float b=1.0f;

    Point operator+(const Point& p)
    {
        Point p_new;
        p_new.x = x + p.x;
        p_new.y = y + p.y;
        p_new.z = z + p.z;
        return p_new;
    }

    Point operator-(const Point& p)
    {
        Point p_new;
        p_new.x = x - p.x;
        p_new.y = y - p.y;
        p_new.z = z - p.z;
        return p_new;
    }

    Point& operator=(const Point& p)
    {
        this->x = p.x;
        this->y = p.y;
        this->z = p.z;
        this->r = p.r;
        this->g = p.g;
        this->b = p.b;

        return *this;
    }



};


