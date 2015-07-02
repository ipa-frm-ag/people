/*
 * lap.h
 *
 *  Created on: Jul 1, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_LAP_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_LAP_H_

/************************************************************************
*
*  lap.h
   version 1.0 - 21 june 1996
   author  Roy Jonker, MagicLogic Optimization Inc.

   header file for LAP

*
      pyLAPJV by Harold Cooper (hbc@mit.edu)
      2004-08-24: changed cost type from double to float, because the
          following matrix (in addition to others) wasn't working:
          http://mit.edu/harold/Public/pyLAPJV-double-fail.txt
      2004-08-13:
          -- fixed Jonker's function declarations to actually use row, col,
             and cost types
    -- row, col, and cost now based on numarray types
*
**************************************************************************/

/*************** CONSTANTS  *******************/

  #define BIG 100000

/*************** TYPES      *******************/

  //typedef npy_int row;
  //#define ROW_TYPE NPY_INT
  //#define ROW_TYPE int
  //typedef npy_int col;
  //typedef npy_int col;
  //#define COL_TYPE NPY_INT
  //#define COL_TYPE int
  //typedef npy_double cost;
  typedef double cost;
  typedef int col;
  typedef int row;
  //#define COST_TYPE NPY_DOUBLE
  //#define COST_TYPE double

/*************** FUNCTIONS  *******************/

extern cost lap(int dim, cost **assigncost,
                col *rowsol, row *colsol, cost *u, cost *v);

//extern void checklap(int dim, cost **assigncost,
//                     col *rowsol, row *colsol, cost *u, cost *v);





#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_JPDA_LAP_H_ */
