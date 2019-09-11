/****************************************************************************
**
** This file is part of FVVR, Free-Viewpoint Video Renderer
** covered by the terms of the GNU Lesser General Public License (LGPL)
** Copyright(c) 2008 by Jonathan Starck
**
** FixedVector.h
**
** Base class for a fixed size 1D array
** This class is non-reference counted and should be used for small vectors
** The class is based on the TFVectorC from RAVL, Recognition And Vision Library
** covered by GNU Lesser General Public License
** For the latest info, see http://sourceforge.net/projects/ravl
**
****************************************************************************/

#ifndef FIXEDVECTOR_
#define FIXEDVECTOR_

#include <assert.h>
#include <iostream>
#include <cmath>

template<typename DataT, unsigned int SizeT>
class FixedVectorC
{
public:

	FixedVectorC()
    {}
    // Default constructor, data is initialised using the default constructor for DataT
    // NOTE for built-in types values will be undefined

public:
// Access
	
    unsigned int Size() const
    { 
    	return SizeT; 
    }
    // Return the size of array
    
    DataT &operator[](unsigned int i) 
    { 
        assert(i < SizeT);
        return m_data[i]; 
    }
    // Access item.

    const DataT &operator[](unsigned int i) const 
    { 
        assert(i < SizeT);
        return m_data[i]; 
    }
    // Access item.

    inline void Set(const DataT &dat)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
        	m_data[i] = dat;
        }
    }
    // Set the elements of the vector

    inline void Limit(const DataT &min,const DataT &max)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
        	if (m_data[i] > max) m_data[i] = max;
        	else if (m_data[i] < min) m_data[i] = min;
        }
    }
    // Limit the values in the vector

public:
// Distance

    inline DataT Sum() const
    {
    	DataT ret = m_data[0];
        for (unsigned int i = 1; i < SizeT; i++)
        {
        	ret += m_data[i];
        }
        return ret;
    }
    // Return the sum of elements
	
    inline DataT SumOfSqr() const
    {
      return Dot( *this );
      
//     	DataT ret = std::pow( m_data[0], 2 );
//         for (unsigned int i = 1; i < SizeT; i++)
//         {
//         	ret += std::pow( m_data[i], 2 );
//         }
//         return ret;
    }
    // Return the sum of elements squared
	
    inline DataT Magnitude() const
    {
    	return std::sqrt( SumOfSqr() );
    }
    // Return the 2-norm of the vector 
    
    inline DataT SqrEuclidDistance(const FixedVectorC<DataT,SizeT> & oth) const
    {
    	DataT ret = std::pow( m_data[0] - oth[0], 2 );
        for (unsigned int i = 1; i < SizeT; i++)
        {
        	ret += std::pow( m_data[i] - oth[i], 2 );
        }
        return ret;
    }
    // Return the square Euclid distance
     
    inline DataT EuclidDistance(const FixedVectorC<DataT,SizeT> & oth) const
    {
    	return std::sqrt( SqrEuclidDistance(oth) );
    }
    // Return the Euclid distance

    inline DataT Dot(const FixedVectorC<DataT,SizeT> & oth) const
    {
    	DataT ret = m_data[0]*oth[0];
        for (unsigned int i = 1; i < SizeT; i++)
        {
        	ret += m_data[i]*oth[i];
        }
        return ret;
    }
    // Return the dot product of this vector and oth    

public:
// Operators
	
    inline bool operator==(const FixedVectorC<DataT,SizeT> & oth) const
    {
        for (unsigned int i=0; i < SizeT; i++)
        {
        	if (m_data[i] != oth[i])
        	{
        		return false;
        	}
        }
        return true;
    }
    // Test for equality

    inline bool operator!=(const FixedVectorC<DataT,SizeT> & oth) const
    {
        for (unsigned int i=0; i < SizeT; i++)
        {
        	if (m_data[i] != oth[i])
        	{
        		return true;
        	}
        }
        return false;
    }
    // Test for inequality
    
    inline const FixedVectorC<DataT,SizeT> & operator+=(const FixedVectorC<DataT,SizeT> & oth)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
          m_data[i] += oth[i];
        }
        return *this;
    }
    // Add in place
    
    inline const FixedVectorC<DataT,SizeT> & operator-=(const FixedVectorC<DataT,SizeT> & oth)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
          m_data[i] -= oth[i];
        }
        return *this;
    }
    // Subtract in place

    inline const FixedVectorC<DataT,SizeT> & operator*=(const FixedVectorC<DataT,SizeT> & oth)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
          m_data[i] *= oth[i];
        }
        return *this;
    }
    // Multiply in place

    inline const FixedVectorC<DataT,SizeT> & operator/=(const FixedVectorC<DataT,SizeT> & oth)
	{
	    for (unsigned int i = 0; i < SizeT; i++)
	    {
	      m_data[i] /= oth[i];
	    }
	    return *this;
	}
    // Divide in place

    
    inline FixedVectorC<DataT,SizeT> operator+(const FixedVectorC<DataT,SizeT> & oth) const
    {
	    FixedVectorC<DataT,SizeT> ret;
	    for (unsigned int i = 0; i < SizeT; i++)
	    {
	      ret[i] = m_data[i] + oth[i];
	    }
	    return ret;
    }
    // Add two vectors

    inline FixedVectorC<DataT,SizeT> operator-(const FixedVectorC<DataT,SizeT> & oth) const
    {
	    FixedVectorC<DataT,SizeT> ret;
	    for (unsigned int i = 0; i < SizeT; i++)
	    {
	      ret[i] = m_data[i] - oth[i];
	    }
	    return ret;
    }
    // Subtract two vectors

    inline FixedVectorC<DataT,SizeT> operator*(const FixedVectorC<DataT,SizeT> & oth) const
    {
	    FixedVectorC<DataT,SizeT> ret;
	    for (unsigned int i = 0; i < SizeT; i++)
	    {
	      ret[i] = m_data[i] * oth[i];
	    }
	    return ret;
    }
    // Multiply two vectors

    inline FixedVectorC<DataT,SizeT> operator/(const FixedVectorC<DataT,SizeT> & oth) const
    {
	    FixedVectorC<DataT,SizeT> ret;
	    for (unsigned int i = 0; i < SizeT; i++)
	    {
	      ret[i] = m_data[i] / oth[i];
	    }
	    return ret;
    }
    // Divide two vectors
    
    inline const FixedVectorC<DataT,SizeT> & operator*=(const DataT &val)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
          m_data[i] *= val;
        }
        return *this;
    }
    // Multiply in place by a single value

    inline const FixedVectorC<DataT,SizeT> & operator/=(const DataT &val)
    {
        for (unsigned int i = 0; i < SizeT; i++)
        {
          m_data[i] /= val;
        }
        return *this;
    }
    // Divide in place by a single value
    
    inline FixedVectorC<DataT,SizeT> operator*(const DataT &val) const
    {
	    FixedVectorC<DataT,SizeT> ret;
        for (unsigned int i = 0; i < SizeT; i++)
        {
          ret[i] = m_data[i] * val;
        }
        return ret;
    }
    // Multiply by a single value
    
    inline FixedVectorC<DataT,SizeT> operator/(const DataT &val) const
    {
	    FixedVectorC<DataT,SizeT> ret;
        for (unsigned int i = 0; i < SizeT; i++)
        {
          ret[i] = m_data[i] / val;
        }
        return ret;
    }
    // Divide by a single value

public:
	
	template<typename D,unsigned int S>
    friend std::istream &operator>> (std::istream &in, FixedVectorC<D,S> &dat);  
	template<typename D,unsigned int S>
    friend std::ostream &operator<< (std::ostream &out,const FixedVectorC<D,S> &dat);
    
protected:
    DataT m_data[SizeT];
};


template<typename DataT,unsigned int SizeT>
inline std::ostream &operator<<(std::ostream &out,const FixedVectorC<DataT,SizeT> &dat) 
{
    for (unsigned int i = 0; i < SizeT; i++)
    {
    	out << dat[i] << ' ';
    }
    return out;
}
  
template<typename DataT,unsigned int SizeT>
inline std::istream &operator>>(std::istream &in,FixedVectorC<DataT,SizeT> &dat) 
{
    for (unsigned int i = 0; i < SizeT; i++)
    {
    	in >> dat[i];
    }
    return in;
}
 
#endif /*FIXEDVECTOR_*/
