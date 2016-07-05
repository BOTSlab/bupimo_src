/**
 * The functions here serve as a bridge for a Python module that connects to
 * Ralf Moeller's warping code using SWIG.  The algorithm implemented comes from
 * the following paper:
 *
 *  Where did I take that snapshot? Scene-based homing by image matching
 *  Journal Biological Cybernetics
 *  Publisher   Springer Berlin / Heidelberg
 *  ISSN    0340-1200 (Print) 1432-0770 (Online)
 *  Issue   Volume 79, Number 3 / October, 1998
 *
 * @author Andrew Vardy
 */

#ifndef WARPING_H
#define WARPING_H

#include <vector>

/**
 * Initialize the module (Python terminology).  The argument is the number of
 * snapshot images (and associated Warping objects) to create.
 */
void init(int n);

/**
 * Sets snapshot image 'i' and creates the internal warping table.
 *
 * Precondition: i < the value of n set in 'init'.
 */
void set_snapshot(int i, std::vector < float > oneDimage);

/**
 * Sets the current image and computes the home vector for snapshot image 'i'.
 * This doesn't return anything only because I haven't figured out how to
 * pass-by-reference with swig.  So instead you have to call 'get_home_x' and
 * 'get_home_y' afterwards.
 *
 * Precondition: i < the value of n set in 'init'.
 */
void compute_home_vector(int i, std::vector < float > oneDimage);

/**
 * Returns computed home vector's x-component.  Must be called after
 * 'compute_home_vector'.
 */
float get_home_x();

/**
 * Returns computed home vector's y-component.  Must be called after
 * 'compute_home_vector'.
 */
float get_home_y();

#endif
