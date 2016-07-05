/* 
 * ===========================================================================
 * 
 * FloatArrays.h --
 * multidimensional array templates
 * 
 * Ralf Moeller <moeller@mpipf-muenchen.mpg.de>
 * 
 *    Copyright (C) 2002
 *    Cognitive Robotics Group
 *    Max Planck Institute for Psychological Research
 *    Munich
 * 
 * 1.0 / 23. May 02 (rm)
 * - from scratch
 * 1.1 / 24. May 02 (rm)
 * - continued
 * 1.2 / 29. May 02 (rm)
 * - operator& in BlockArray
 * - transpose in BlockMatrix
 * - constructors without init in all classes
 * 1.3 / 29. May 02 (rm)
 * - Axel doesn't like operator&, so now it's BlockArray::operator()
 * 1.4 /  3. Jun 02 (rm)
 * - removed operator() and introduced type-conversion operator instead
 * 1.5 / 17. Jul 02 (rm)
 * - resize functions
 * - introduced "if (data)" before "delete [] data"
 * 1.6 / 21. Aug 02 (rm)
 * - Axel wants a transpose that affects *this
 * 1.7 / 28. Aug 02 (rm)
 * - with const in operator[]
 * 1.8 / 12. Sep 02 (rm)
 * - more const added to functions
 * 1.9 / 25. Oct 02 (rm)
 * - memCopy
 * 1.10 / 25. Oct 02 (rm)
 * - some comments
 * - we check the sizes in operator= and avoid restructuring when
 *   the size doesn't change
 * 1.11 / 27. Oct 02 (rm)
 * - there is not much difference in time between checking or not checking
 *   so we always check (actually, not checking was even a bit slower -> ???)
 * - assert is used for check
 * - I also removed the debug stuff
 * - BlockArray has no operator () any longer, now called at()
 * 1.12 / 28. Oct 02 (rm)
 * - reintroduced BlockArray::operator()
 * 1.13 / 14. Mar 03 (rm)
 * - changed operators []: now 1st with const xxx const and 2nd without const
 * 1.14 /  1. Apr 03 (rm)
 * - reset function (combination of resize and fill)
 * - cast operator in cpp-brackets
 * 1.15 / 18. Aug 03 (rm)
 * - removed cast operator (see undef)
 * 1.16 / 18. Feb 08 (av)
 * - removed templates and fixed data type to floats to avoid strange compiler
 *   problems
 * - renamed to FloatArrays.h
 *
 * ===========================================================================
 */

#ifndef FLOATARRAYS_H
#define FLOATARRAYS_H

#include <assert.h>
#include <stdio.h>
#include <string.h>

/*
  basic ideas:
  ------------

  - each array can be either used "standalone", which means, it's responsible
    for allocating memory and freeing it in the end, or "non-standalone",
    which means, it is part of a higher-dimensional structure and is therefore
    initialized with a pointer to data space that was allocated by some
    higher-dimensional class (standalone arrays have sablk != 0)

  - assign() should not be used from outside (unfortunately I didn't manage
    to work with friend functions, since you can not do a forward declaration
    with template classes --- in this case, assign() would have been protected)

  - block() gives access to the internal data buffer (blk) which is
    organized in "last-index-varies-fastest" manner (opposite to FORTRAN!!!) 

  - index access in C-style with cube[i][j][k]: WITH index check!
  - index access cube(i)(j)(k): WITHOUT index check!

  - block data access with cube(m) (indexed) or cube.block() or &cube

  - all classes are derived from BlockArray; it doesn't make much sense to
    use BlockArray directly

*/

// 18. Aug 03 (rm)
// now we have a new compiler, so we have to switch this off
#ifdef ARRAYS_USE_CAST_OPERATOR
#undef ARRAYS_USE_CAST_OPERATOR
#endif

namespace floatarrays {
  
  //--------------------------------------------------------------------------
  // FloatBlockArray
  //--------------------------------------------------------------------------

  // FloatBlockArray is base class to all array classes
  // it takes case of block allocation and deallocation and provides
  // fill and copy functions

  class FloatBlockArray
  {
  public:

    // called from all constructors of derived classes
    FloatBlockArray() : blksz(0), blk(0), sablk(0)
    {
    }

    // has to be virtual to be called from destructors of derived classes
    virtual ~FloatBlockArray()
    {
      if (sablk) delete [] sablk;
    }

    // access to block data, to be used with care
    float* block() const { return blk; }

    // access to block data, to be used with care
    // THIS MAY ACTUALLY CAUSE PROBLEMS IN LATER VERSION OF G++
    // OBVIOUSLY IT IS SEEN AS A CANDIDATE WHEN AN OBJECT IS ACCESSED
    // THROUGH [], AND THE COMPILER PRODUCES SOME WARNING LIKE
    // choosing `arrays::FloatBlockMatrix&
    // arrays::FloatBlockCube::operator[](unsigned int) const [with T = Complex]'
    // over `operator[]'
#ifdef ARRAYS_USE_CAST_OPERATOR
    operator float*() const
    {
      return blk; 
    }
#endif
 
    // element-wise access to block data
    float& at(unsigned int i) const
    {
      assert(i < blksz);
      return blk[i];
    }

    // same as at()
    float& operator() (unsigned int i) const
    {
      assert(i < blksz);
      return blk[i];
    }
    
    // fill data block with initial value
    void fill(const float& init)
    {
      for (unsigned int i = 0; i < blksz; i++) blk[i] = init;
    }

    // copy data block from another array
    // we do it element-by-element, since these elements could
    // be objects (with pointers etc.) that are not suitable for
    // memory copy
    void copy(const FloatBlockArray &ba)
    {
      for (unsigned int i = 0; i < blksz; i++) blk[i] = ba.blk[i];
    }

    // copy with memcpy
    // we only check the size of the block, the user has to take care
    // the sizes of vectors/matrices etc. are correct
    // this uses memcpy and is only suited for contents without pointers
    void memCopy(const FloatBlockArray &ba)
    {
      assert(blksz == ba.blksz);
      memcpy(blk, ba.blk, sizeof(float) * blksz);
    }

    unsigned int blockSize() const { return blksz; }

  protected:
    
    unsigned int blksz;		// data block size
    float *blk, *sablk;		// data block, self-allocated data block
  };

  //--------------------------------------------------------------------------
  // FloatBlockVector
  //--------------------------------------------------------------------------
  
  class FloatBlockVector : public FloatBlockArray
  {    
  public:

    // default constructor: no elements
    FloatBlockVector() : sz(0)
    {
    }
    
    // constructor for standalone vectors
    FloatBlockVector(unsigned int size)
    {
      sablk = new float[size];
      assign(size, sablk);
    }
 
    // constructor for standalone vectors
    FloatBlockVector(unsigned int size, const float& init)
    {
      sablk = new float[size];
      assign(size, sablk);
      fill(init);
    }
    
    // copy-constructor (creates a standalone object)
    FloatBlockVector(const FloatBlockVector &bv)
    {
      sablk = new float[bv.blksz];
      assign(bv.sz, sablk);
      copy(bv);
    }

    // destructor (no further data to be deallocated here)
    ~FloatBlockVector()
    {
    }

    // resizing
    void resize(unsigned int size)
    {
      // only for standalone objects or empty objects
      assert(sablk || !blk);
      _resize(size);
    }

    // reset: resize and filling
    void reset(unsigned int size, const float& init)
    {
      resize(size);
      fill(init);
    }

    // = operator
    FloatBlockVector& operator=(const FloatBlockVector &bv)
    {
      if (sablk || !blk) {
	// for standalone objects we deallocate the self-allocated data and
	// create new space; for empty objects we create a stand-alone
	_resize(bv.sz);
      } 
      else {
	// for non-standalone objects, we should not change the size (they
	// are contained in some larger object!)
	assert(sz == bv.sz);
	// just copy the data to the data block, blk and sablk are untouched
      }
      copy(bv);
      return *this;
    }

    // not to be called from the outside
    void assign(unsigned int size, float* block)
    {
      blksz = sz = size;
      blk = block;
    }

    // index access with check
    float& operator[] (unsigned int i)
    {
      assert(i < sz);
      return blk[i];
    }

    // index access with check
    const float& operator[] (unsigned int i) const
    {
      assert(i < sz);
      return blk[i];
    }

    // vector size
    unsigned int size() const { return sz; }

  protected:
    
    // resizing
    void _resize(unsigned int size)
    {
      // only if size changes
      if (size != sz) {
	if (sablk) delete [] sablk;
	sablk = new float[size];
	assign(size, sablk);
      }
    }

    unsigned int sz;		// vector size
  };

  //--------------------------------------------------------------------------
  // FloatBlockMatrix
  //--------------------------------------------------------------------------

  class FloatBlockMatrix : public FloatBlockArray
  {
  public:

    // default constructor: no elements
    FloatBlockMatrix() : sz(0), subsz(0), data(0)
    {
    }
    
    // constructor for standalone matrices
    FloatBlockMatrix(unsigned int size, unsigned int subsize) 
    {
      sablk = new float[size * subsize];
      assign(size, subsize, sablk);
    }

    // constructor for standalone matrices
    FloatBlockMatrix(unsigned int size, unsigned int subsize, const float& init) 
    {
      sablk = new float[size * subsize];
      assign(size, subsize, sablk);
      fill(init);
    }
    
    // copy-constructor: creates a standalone object
    FloatBlockMatrix(const FloatBlockMatrix& bm)
    {
      sablk = new float[bm.blksz];
      assign(bm.sz, bm.subsz, sablk);
      copy(bm);
    }

    // destructor: we deallocate our pointer structure
    ~FloatBlockMatrix()
    {
      if (data) delete [] data;
    }
   
    // resizing
    void resize(unsigned int size, unsigned int subsize)
    {
      // only for standalone objects or empty objects
      assert(sablk || !blk);
      _resize(size, subsize);
    }

    // reset: resize and filling
    void reset(unsigned int size, unsigned int subsize, const float& init)
    {
      resize(size, subsize);
      fill(init);
    }

    // = operator
    FloatBlockMatrix& operator=(const FloatBlockMatrix& bm)
    {
      if (sablk || !blk) {
	// standalone objects are restructured, empty objects are created
	_resize(bm.sz, bm.subsz);
      }
      else {
	// non-standalone objects have to have the right size
	assert((sz == bm.sz) && (subsz == bm.subsz));
      }
      copy(bm);
      return *this;
    }

    // returns the transposed of *this (*this is unaffected)
    FloatBlockMatrix transposed() const
    {
      FloatBlockMatrix t(subsz, sz);

      for (unsigned int i = 0; i < sz; i++)
	for (unsigned int j = 0; j < subsz; j++)
	  t[j][i] = data[i][j];
      return t;
    }
    
    // transposes *this (and returns reference to modified *this)
    FloatBlockMatrix& transpose()
    {
      // keep old matrix
      FloatBlockMatrix old(*this);
      // resize this
      resize(subsz, sz);
      // note that sz and subsz are the new values now
      for (unsigned int i = 0; i < subsz; i++)
	for (unsigned int j = 0; j < sz; j++)
	  data[j][i] = old[i][j];
      return *this;
    }

    // not to be called from the outside
    void assign(unsigned int size, unsigned int subsize, float* block)
    {
      sz = size;
      subsz = subsize;
      blksz = sz * subsz;
      blk = block;
      data = new FloatBlockVector[sz];     
      float* bptr = blk;
      for (unsigned int i = 0; i < sz; i++, bptr += subsz)
	data[i].assign(subsz, bptr);
    }
 
    // index access with check
    FloatBlockVector& operator[] (unsigned int i)
    {
      assert(i < sz);
      return data[i];
    }

    // index access with check
    const FloatBlockVector& operator[] (unsigned int i) const
    {
      assert(i < sz);
      return data[i];
    }

    // sizes
    unsigned int size() const { return sz; }
    unsigned int subsize() const { return subsz; }

  protected:

    // resizing
    void _resize(unsigned int size, unsigned int subsize)
    {
      // only if size changes
      if ((size != sz) || (subsize != subsz)) {
	if (sablk) delete [] sablk;
	if (data) delete [] data;
	sablk = new float[size * subsize];
	assign(size, subsize, sablk);
      }
    }

    unsigned int sz, subsz;
    FloatBlockVector* data;
  };

  //--------------------------------------------------------------------------
  // FloatBlockCube
  //--------------------------------------------------------------------------

  class FloatBlockCube : public FloatBlockArray
  {
  public:

    // default constructor: no elements
    FloatBlockCube() : sz(0), subsz(0), sub2sz(0), data(0)
    {
    }

    // constructor for standalone cubes
    FloatBlockCube(unsigned int size, unsigned int subsize, unsigned int sub2size)
    {
      sablk = new float[size * subsize * sub2size];
      assign(size, subsize, sub2size, sablk);
    }

    // constructor for standalone cubes
    FloatBlockCube(unsigned int size, unsigned int subsize, unsigned int sub2size,
	      const float& init)
    {
      sablk = new float[size * subsize * sub2size];
      assign(size, subsize, sub2size, sablk);
      fill(init);
    }
    
    // copy-constructor: creates a standalone cube
    FloatBlockCube(const FloatBlockCube& bc)
    {
      sablk = new float[bc.blksz];
      assign(bc.sz, bc.subsz, bc.sub2sz, sablk);
      copy(bc);
    }

    // destructor: we have to deallocate the pointer structure
    ~FloatBlockCube()
    {
      if (data) delete [] data;
    }

    // resizing
    void resize(unsigned int size, unsigned int subsize,
		unsigned int sub2size)
    {
      // only for standalone objects or empty objects
      assert(sablk || !blk);
      _resize(size, subsize, sub2size);
    }

    // reset: resize and filling
    void reset(unsigned int size, unsigned int subsize,
	       unsigned int sub2size, const float& init)
    {
      resize(size, subsize, sub2size);
      fill(init);
    }

    // = operator
    FloatBlockCube& operator=(const FloatBlockCube& bc)
    {
      if (sablk || !blk) {
	// standalone objects are restructured, empty objects just created
	_resize(sz, subsz, sub2sz);
      }
      else {
	// non-standalone objects are just filled with new data
	assert((sz == bc.sz) && (subsz == bc.subsz) && \
		    (sub2sz == bc.sub2sz));
      }
      copy(bc);
      return *this;
    }
    
    // not to be called from the outside
    void assign(unsigned int size, unsigned int subsize, 
		unsigned int sub2size, float* block)
    {
      sz = size;
      subsz = subsize;
      sub2sz = sub2size;
      blksz = sz * subsize * sub2size;
      blk = block;
      data = new FloatBlockMatrix[sz];
      float* bptr = blk;
      for (unsigned int i = 0; i < size; i++, bptr += subsz * sub2sz)
	data[i].assign(subsz, sub2sz, bptr);
    }

    // index access with check
    FloatBlockMatrix& operator[] (unsigned int i)
    {
      assert(i < sz);
      return data[i];
    }

    // index access with check
    const FloatBlockMatrix& operator[] (unsigned int i) const
    {
      assert(i < sz);
      return data[i];
    }
    
    // sizes
    unsigned int size() const { return sz; }
    unsigned int subsize() const { return subsz; }
    unsigned int sub2size() const { return sub2sz; }

  protected:

    // resizing
    void _resize(unsigned int size, unsigned int subsize,
		 unsigned int sub2size)
    {
      // only if size changes
      if ((size != sz) || (subsize != subsz) || (sub2size != sub2sz)) {
	if (sablk) delete [] sablk;
	if (data) delete [] data;
	sablk = new float[size * subsize * sub2size];
	assign(size, subsize, sub2size, sablk);
      }
    }

    unsigned int sz, subsz, sub2sz;
    FloatBlockMatrix* data;
  };

  //--------------------------------------------------------------------------
  // FloatBlockHyper4
  //--------------------------------------------------------------------------

  class FloatBlockHyper4 : public FloatBlockArray
  {
  public:

    // default constructor: no elements
    FloatBlockHyper4() : sz(0), subsz(0), sub2sz(0), sub3sz(0), data(0)
    {
    }
    
    // constructor for standalone cubes
    FloatBlockHyper4(unsigned int size, unsigned int subsize, 
		unsigned int sub2size, unsigned int sub3size)
    {
      sablk = new float[size * subsize * sub2size * sub3size];
      assign(size, subsize, sub2size, sub3size, sablk);
    }
 
    // constructor for standalone cubes
    FloatBlockHyper4(unsigned int size, unsigned int subsize, 
		unsigned int sub2size, unsigned int sub3size,
		const float& init)
    {
      sablk = new float[size * subsize * sub2size * sub3size];
      assign(size, subsize, sub2size, sub3size, sablk);
      fill(init);
    }
    
    // copy-constructor: creates a standalone cube
    FloatBlockHyper4(const FloatBlockHyper4& bh)
    {
      sablk = new float[bh.blksz];
      assign(bh.sz, bh.subsz, bh.sub2sz, bh.sub3sz, sablk);
      copy(bh);
    }
    
    // destructor: we have to deallocate the pointer structure
    ~FloatBlockHyper4()
    {
      if (data) delete [] data;
    }
    
    // resizing
    void resize(unsigned int size, unsigned int subsize,
		unsigned int sub2size, unsigned int sub3size)
    {
      // only for standalone objects or empty objects
      assert(sablk || !blk);
      _resize(size, subsize, sub2size, sub3size);
    }

    // reset: resize and filling
    void reset(unsigned int size, unsigned int subsize,
	       unsigned int sub2size, unsigned int sub3size,
	       const float& init)
    {
      resize(size, subsize, sub2size, sub3size);
      fill(init);
    }
    
    // = operator
    FloatBlockHyper4& operator=(const FloatBlockHyper4& bh)
    {
      if (sablk || !blk) {
	// standalone objects are restructured, empty objects just created
	_resize(bh.sz, bh.subsz, bh.sub2sz, bh.sub3sz);
      }
      else {
	// non-standalone objects are just filled with new data
	assert((sz == bh.sz) && (subsz == bh.subsz) && \
		    (sub2sz == bh.sub2sz) && (sub3sz == bh.sub3sz));
      }
      copy(bh);
      return *this;
    }
    
    // not to be called from the outside
    void assign(unsigned int size, unsigned int subsize, 
		unsigned int sub2size, unsigned int sub3size, float* block)
    {
      sz = size;
      subsz = subsize;
      sub2sz = sub2size;
      sub3sz = sub3size;
      blksz = sz * subsize * sub2size * sub3size;
      blk = block;
      data = new FloatBlockCube[sz];
      float* bptr = blk;
      for (unsigned int i = 0; i < size; i++, bptr += subsz * sub2sz * sub3sz)
	data[i].assign(subsz, sub2sz, sub3sz, bptr);
    }

    // index access with check
    FloatBlockCube& operator[] (unsigned int i)
    {
      assert(i < sz);
      return data[i];
    }

    // index access with check
    const FloatBlockCube& operator[] (unsigned int i) const
    {
      assert(i < sz);
      return data[i];
    }
    
    // sizes
    unsigned int size() const { return sz; }
    unsigned int subsize() const { return subsz; }
    unsigned int sub2size() const { return sub2sz; }
    unsigned int sub3size() const { return sub3sz; }
    
  protected:

    // resizing
    void _resize(unsigned int size, unsigned int subsize,
		unsigned int sub2size, unsigned int sub3size)
    {
      // only if size changes
      if ((size != sz) || (subsize != subsz) ||
	  (sub2size != sub2sz) || (sub3size != sub3sz)) {
	if (sablk) delete [] sablk;
	if (data) delete [] data;
	sablk = new float[size * subsize * sub2size * sub3size];
	assign(size, subsize, sub2size, sub3size, sablk);
      }
    }

    unsigned int sz, subsz, sub2sz, sub3sz;
    FloatBlockCube* data;
  };

}

#endif
