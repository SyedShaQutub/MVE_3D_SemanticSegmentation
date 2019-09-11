/****************************************************************************
**
** This file is part of FVVR, Free-Viewpoint Video Renderer
** covered by the terms of the GNU Lesser General Public License (LGPL)
** Copyright(c) 2008 by Jonathan Starck
**
** FixedMatrix.h
**
** Base class for a fixed size 2D array
** This class is non-reference counted and should be used for small matrices
** The class is based on the TFMatrixC from RAVL, Recognition And Vision Library
** covered by GNU Lesser General Public License
** For the latest info, see http://sourceforge.net/projects/ravl
**
****************************************************************************/

#ifndef FIXEDMATRIX_H
#define FIXEDMATRIX_H

#include <assert.h>
#include <iostream>
#include <cmath>

#include "FixedVector.h"

template<typename DataT, unsigned int RowSizeT, unsigned int ColSizeT>
class FixedMatrixC
{
public:
	FixedMatrixC()
    {}
    // Default constructor, the matrix is initialised using the default constructor for DataT
    // NOTE for built-in types values will be undefined

    static FixedMatrixC<DataT,RowSizeT,ColSizeT> Identity()
    {
	    FixedMatrixC<DataT,RowSizeT,ColSizeT> ret;
	    DataT* p = ret[0];
	    for (unsigned int i = 0; i < RowSizeT; i++)
	    {
		    for (unsigned int j = 0; j < ColSizeT; j++, p++)
		    {
		    	*p = (i == j) ? 1.0f : 0.0f;
		    }
	    }
	    return ret;
    }
    // Create an identity matrix.
   		
public:
// Access
	
    unsigned int Rows() const
    { 
    	return RowSizeT; 
    }
    // Return the size of array
    
    unsigned int Cols() const
    { 
    	return ColSizeT; 
    }
    // Return the size of array
    
    DataT* operator[](unsigned int row) 
    { 
        assert(row < RowSizeT);
        return (m_data[ row ]); 
    }
    const DataT* operator[](unsigned int row) const 
    { 
        assert(row < RowSizeT);
        return (m_data[ row ]); 
    }
    // Access a row in the matrix, returns a pointer to the start of the row
    // Access elements as matrix[row][column]
    // WARNING: there is no bounds checking on column access

    inline void Set(const DataT &dat)
    {
    	DataT* p = &m_data[0][0];
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p++)
        {
        	*p = dat;
        }
    }
    // Set the elements of the matrix
    
    inline void Limit(const DataT &min,const DataT &max)
    {
    	DataT* p = m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p++)
        {
        	if (*p > max) *p = max;
        	else if (*p < min) *p = min;
        }
    }
    // Limit the values in the vector

    inline void T(FixedMatrixC<DataT,ColSizeT,RowSizeT>& ret) const
    {
	    const DataT* p = m_data[0];
	    for (unsigned int i = 0; i < RowSizeT; i++)
	    {
		    for (unsigned int j = 0; j < ColSizeT; j++, p++)
		    {
		    	ret[j][i] = *p;
		    }
	    }
    }
    // Set the transpose of the matrix
 
    inline FixedMatrixC<DataT,ColSizeT,RowSizeT> T() const
    {
	    FixedMatrixC<DataT,ColSizeT,RowSizeT> ret;
	    T(ret);
	    return ret;
    }
    // Return the transpose of the matrix
 
public:
// Distance

    inline DataT Sum() const
    {
    	DataT ret = *m_data;
    	DataT* p = m_data + 1;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=1; i < size; i++, p++)
        {
        	ret += *p;
        }
        return ret;
    }
    // Return the sum of elements
	
    inline DataT SumOfSqr() const
    {
    	DataT ret = std::pow( *m_data, 2 );
    	DataT* p = m_data + 1;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=1; i < size; i++, p++)
        {
        	ret += std::pow( *p, 2 );
        }
        return ret;
    }
    // Return the sum of elements squared
	     
public:
// Operators
	
    inline bool operator==(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth) const
    {
    	const DataT* p1 = m_data;
    	const DataT* p2 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
        	if (*p1 != *p2)
        	{
        		return false;
        	}
        }
        return true;
    }
    // Test for equality

    inline bool operator!=(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth) const
    {
    	const DataT* p1 = m_data;
    	const DataT* p2 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
        	if (*p1 != *p2)
        	{
        		return true;
        	}
        }
        return false;
    }
    // Test for inequality
    
    inline const FixedMatrixC<DataT,RowSizeT,ColSizeT> & operator+=(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth)
    {
    	DataT* p1 = m_data;
    	const DataT* p2 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
    		*p1 += *p2;
        }
        return *this;
    }
    // Add in place
    
    inline const FixedMatrixC<DataT,RowSizeT,ColSizeT> & operator-=(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth)
    {
    	DataT* p1 = m_data;
    	const DataT* p2 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
    		*p1 -= *p2;
        }
        return *this;
    }
    // Subtract in place

    inline FixedMatrixC<DataT,RowSizeT,ColSizeT> operator+(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth) const
    {
	    FixedMatrixC<DataT,RowSizeT,ColSizeT> ret;
    	DataT* p1 = ret.m_data;
    	const DataT* p2 = m_data;
    	const DataT* p3 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++, p3++)
        {
    		*p1 = *p2 + *p3;
        }
	    return ret;
    }
    // Add two vectors

    inline FixedMatrixC<DataT,RowSizeT,ColSizeT> operator-(const FixedMatrixC<DataT,RowSizeT,ColSizeT> & oth) const
    {
	    FixedMatrixC<DataT,RowSizeT,ColSizeT> ret;
    	DataT* p1 = ret.m_data;
    	const DataT* p2 = m_data;
    	const DataT* p3 = oth.m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++, p3++)
        {
    		*p1 = *p2 - *p3;
        }
	    return ret;
    }
    // Subtract two vectors

    FixedVectorC<DataT,RowSizeT> operator*(const FixedVectorC<DataT,ColSizeT> & vec) const;
    // Multiply and return a new vector

    template<unsigned int SizeT>
    FixedMatrixC<DataT,RowSizeT,SizeT> operator*(const FixedMatrixC<DataT,ColSizeT,SizeT> & oth) const
    {
    	FixedMatrixC<DataT,RowSizeT,SizeT> result;
		unsigned int i, j, k;
		DataT* rp = result[0];
		for(i = 0; i < RowSizeT; i++) 
		{
			for(j = 0; j < SizeT; j++, rp++) 
			{
				const DataT* mp1 = m_data[i];
				const DataT* mp2 = &oth[0][j];
				DataT val = *mp1 * *mp2;
				for(mp1++, mp2+=SizeT, k=1; k < ColSizeT; mp1++, mp2+=SizeT, k++)
				{
					val += *mp1 * *mp2;
				}
				*rp = val;
			}
		}
    	return result;
    }
    // Multiply and return a new matrix

    inline const FixedMatrixC<DataT,RowSizeT,ColSizeT> & operator*=(const DataT &val)
    {
    	DataT* p = m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p++)
        {
    		*p *= val;
        }
        return *this;
    }
    // Multiply in place by a single value

    inline const FixedMatrixC<DataT,RowSizeT,ColSizeT> & operator/=(const DataT &val)
    {
    	DataT* p = m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p++)
        {
    		*p /= val;
        }
        return *this;
    }
    // Divide in place by a single value
    
    inline FixedMatrixC<DataT,RowSizeT,ColSizeT> operator*(const DataT &val) const
    {
	    FixedMatrixC<DataT,RowSizeT,ColSizeT> ret;
    	DataT* p1 = ret.m_data;
    	const DataT* p2 = m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
    		*p1 = *p2 * val;
        }
	    return ret;
    }
    // Multiply by a single value
    
    inline FixedMatrixC<DataT,RowSizeT,ColSizeT> operator/(const DataT &val) const
    {
	    FixedMatrixC<DataT,RowSizeT,ColSizeT> ret;
    	DataT* p1 = ret.m_data;
    	const DataT* p2 = m_data;
    	unsigned int i, size = RowSizeT*ColSizeT;
    	for (i=0; i < size; i++, p1++, p2++)
        {
    		*p1 = *p2 / val;
        }
	    return ret;
    }
    // Divide by a single value

    FixedVectorC<DataT,RowSizeT> TMul(const FixedVectorC<DataT,ColSizeT> & vec) const;
    // Tanspose multiply and return a new vector
    
public:

	template<typename D, unsigned int R, unsigned int C>
    friend std::istream &operator>> (std::istream &in, FixedMatrixC<D,R,C> &dat);  
	template<typename D, unsigned int R, unsigned int C>
    friend std::ostream &operator<< (std::ostream &out,const FixedMatrixC<D,R,C> &dat);

protected:
    DataT m_data[RowSizeT][ColSizeT];
};

template<typename DataT, unsigned int RowSizeT, unsigned int ColSizeT>
inline std::ostream &operator<<(std::ostream &out,const FixedMatrixC<DataT,RowSizeT,ColSizeT> &dat) 
{
	const DataT* p = dat[0];
        out <<"[";
    for (unsigned int i = 0; i < RowSizeT; i++)
    {
	    for (unsigned int j = 0; j < ColSizeT; j++, p++)
	    {
	    	out << *p << ((j<ColSizeT-1)?", ":";" );
	    }
	    out << std::endl;
    }
    out <<"] ";
    return out;
}
  
template<typename DataT, unsigned int RowSizeT, unsigned int ColSizeT>
inline std::istream &operator>>(std::istream &in,FixedMatrixC<DataT,RowSizeT,ColSizeT> &dat) 
{
	DataT* p = dat[0];
	unsigned int i, size = RowSizeT*ColSizeT;
	for (i=0; i < size; i++, p++)
    {
    	in >> *p ;
    }
    return in;
}


/****************************************************************************
** Return by value can be slow, 
** this gets around that by passing the location to store
****************************************************************************/

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT>
void Mul(const FixedMatrixC<DataT,RowSizeT,ColSizeT> &mat,const FixedVectorC<DataT,ColSizeT> &vec, FixedVectorC<DataT,RowSizeT> &result) 
{
	unsigned int i, j;
	DataT* rp = &result[0];
	const DataT* mp = mat[0];
	for(i = 0; i < RowSizeT; i++, rp++) 
	{
		const DataT* vp = &vec[0];
		DataT val = *mp * *vp;
		for(mp++, vp++, j=1; j < ColSizeT; mp++, vp++, j++)
		{
			val += *mp * *vp;
		}
		*rp = val;
	}
}
// Compute result = mat * vec;

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT,unsigned int SizeT>
void Mul(const FixedMatrixC<DataT,RowSizeT,ColSizeT> &mat1,const FixedMatrixC<DataT,ColSizeT,SizeT> &mat2, FixedMatrixC<DataT,RowSizeT,SizeT> &result) 
{
	unsigned int i, j, k;
	DataT* rp = result[0];
	for(i = 0; i < RowSizeT; i++) 
	{
		for(j = 0; j < SizeT; j++, rp++) 
		{
			const DataT* mp1 = mat1[i];
			const DataT* mp2 = &mat2[0][j];
			DataT val = *mp1 * *mp2;
			for(mp1++, mp2+=SizeT, k=1; k < ColSizeT; mp1++, mp2+=SizeT, k++)
			{
				val += *mp1 * *mp2;
			}
			*rp = val;
		}
	}
}
// Compute result = mat1 * mat2

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT>
void TMul(const FixedMatrixC<DataT,RowSizeT,ColSizeT> &mat,const FixedVectorC<DataT,RowSizeT> &vec, FixedVectorC<DataT,ColSizeT> &result) 
{
	unsigned int i, j;
	DataT* rp = &result[0];
	for(i = 0; i < ColSizeT; i++, rp++) 
	{
		const DataT* mp = &mat[0][i];
		const DataT* vp = &vec[0];
		DataT val = *mp * *vp;
		for(mp+=ColSizeT, vp++, j=1; j < RowSizeT; mp+=ColSizeT, vp++, j++)
		{
			val += *mp * *vp;
		}
		*rp = val;
	}
}
// Compute result = mat.T() * vec;

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT,unsigned int SizeT>
void TMul(const FixedMatrixC<DataT,RowSizeT,ColSizeT> &mat1,const FixedMatrixC<DataT,RowSizeT,SizeT> &mat2, FixedMatrixC<DataT,ColSizeT,SizeT> &result) 
{
	unsigned int i, j, k;
	DataT* rp = result[0];
	for(i = 0; i < ColSizeT; i++) 
	{
		for(j = 0; j < SizeT; j++, rp++) 
		{
			const DataT* mp1 = &mat1[0][i];
			const DataT* mp2 = &mat2[0][j];
			DataT val = *mp1 * *mp2;
			for(mp1+=ColSizeT, mp2+=SizeT, k=1; k < RowSizeT; mp1+=ColSizeT, mp2+=SizeT, k++)
			{
				val += *mp1 * *mp2;
			}
			*rp = val;
		}
	}
}
// Compute result = mat1.T() * mat2

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT>
FixedVectorC<DataT,RowSizeT> 
inline FixedMatrixC<DataT,RowSizeT,ColSizeT>::operator*(const FixedVectorC<DataT,ColSizeT> & vec) const
{
	FixedVectorC<DataT,RowSizeT> ret;
	::Mul(*this,vec,ret);
	return ret;
}
// Multiply and return a new vector

template<class DataT,unsigned int RowSizeT,unsigned int ColSizeT>
FixedVectorC<DataT,RowSizeT> 
inline FixedMatrixC<DataT,RowSizeT,ColSizeT>::TMul(const FixedVectorC<DataT,ColSizeT> & vec) const
{
	FixedVectorC<DataT,RowSizeT> ret;
	::TMul(*this,vec,ret);
	return ret;
}
// Transpose multiply and return a new vector

#endif /*FIXEDVECTOR_*/
