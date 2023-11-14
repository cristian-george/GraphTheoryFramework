#include "Point.h"

template<class T>
Point<T>::Point(T x, T y)
    : m_x(x)
    , m_y(y) {}

template<class T>
T Point<T>::GetX() const
{
    return m_x;
}

template<class T>
T Point<T>::GetY() const
{
    return m_y;
}

template<class T>
void Point<T>::SetX(T x)
{
    m_x = x;
}

template<class T>
void Point<T>::SetY(T y)
{
    m_y = y;
}
