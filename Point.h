#pragma once

#ifndef POINT_H
#define POINT_H

template<class T>
class Point
{
public:
    Point(T x = T(), T y = T());

    T GetX() const;
    T GetY() const;

    void SetX(T x);
    void SetY(T y);

private:
    T m_x, m_y;
};

#include "Point.cpp"

#endif // POINT_H
