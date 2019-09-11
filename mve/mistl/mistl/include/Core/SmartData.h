/****************************************************************************
**
** This file is part of FVVR, Free-Viewpoint Video Renderer
** covered by the terms of the GNU Lesser General Public License (LGPL)
** Copyright(c) 2008 by Jonathan Starck
**
** Smart data pointer
**   A reference counted data class
**
****************************************************************************/

#ifndef SMARTDATA_H
#define SMARTDATA_H

#include <assert.h>

// Base for a reference counted data class
class SmartDataC
{
public:
// Construction creates no references	
    inline SmartDataC(): m_refcount(0) { }
    inline SmartDataC(const SmartDataC &): m_refcount(0) { }
    virtual ~SmartDataC() { }

public:
// The smart pointer handles reference counting for the data
    inline bool IncRef()
    { return ( ++(m_refcount) ) != 0; }
    inline bool DecRef()
    { return ( --(m_refcount) ) != 0; }

private:
    SmartDataC &operator=(const SmartDataC &);
    // Prevent use of the assignment operator as this will break the reference count

private:
    // Generic reference counter
    mutable int m_refcount;
};

// Smart pointer for a reference counted data class
template <class DataT> 
class SmartDataPointerC
{
public:
// Construction

	inline SmartDataPointerC(): d(0)
	{ }
	// Default constructor creates an invalid data handle
	
    inline ~SmartDataPointerC() 
    { 
    	if (d && !d->DecRef()) delete d; 
    }
    // Delete the data if no reference remains
    
    inline SmartDataPointerC(DataT& data): d(&data) 
    { 
    	d->IncRef(); 
    }
    // Construct a new pointer to data 
    
    inline SmartDataPointerC(const DataT* data): d(data) 
    { 
    	if (d) d->IncRef(); 
    }
    // Construct a new pointer to data 
    
    inline SmartDataPointerC(const SmartDataPointerC<DataT>& oth): d(oth.d) 
    { 
    	if (d) d->IncRef(); 
    }
    // Construct a new pointer to data 
    
public:
// Assignment

	inline SmartDataPointerC<DataT>& operator=(const SmartDataPointerC<DataT> &oth) 
    {
        if (oth.d != d) 
        {
            DataT *x = oth.d;
            if (x) x->IncRef();
            if (d && !d->DecRef()) delete d;
            d = x;
        }
        return *this;
    }
    inline SmartDataPointerC &operator=(DataT *data) 
    {
        if (data != d) 
        {
            DataT *x = data;
            if (x) x->IncRef();
            if (d && !d->DecRef()) delete d;
            d = x;
        }
        return *this;
    }

public:
// Access
    inline DataT &operator*() { return *d; }
    inline const DataT &operator*() const { return *d; }
    inline DataT *operator->() { return d; }
    inline const DataT *operator->() const { return d; }
    inline operator DataT *() { return d; }
    inline operator const DataT *() const { return d; }

    inline bool operator==(const SmartDataPointerC<DataT> &oth) const { return d == oth.d; }
    inline bool operator!=(const SmartDataPointerC<DataT> &oth) const { return d != oth.d; }

private:
    DataT *d;
};

#endif
