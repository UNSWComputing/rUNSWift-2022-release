#pragma once

#include "FieldDimensionsJSON.hpp"

/** The field coordinate system in mm and radians (rad)
 *  X -- is along the length of the field, +ve towards opponent's goal
 *  Y -- is along the width of the field, +ve towards the left hand side
 *  0 rad -- facing straight towards opponent's goal at origin
 *  radians are calculated counter clock-wise
 *  NOTE: we use -PI, not PI for 180 degrees
 *  NOTE: unless explicitly specified all dimensions includes line width
 */

#define FIELD_LINE_WIDTH 50
//#define USING_SMALL_FIELD
#define USE_JSON


// Will need to re-measure field when it is built
#if defined(USE_JSON)
   #define ROBOTS_PER_TEAM 6

   /** Field line dimensions */
   #define FIELD_LENGTH FIELDJSON_LENGTH
   #define FIELD_WIDTH FIELDJSON_WIDTH

   #define FIELD_LENGTH_OFFSET FIELDJSON_BORDERSTRIPWIDTH
   #define FIELD_WIDTH_OFFSET FIELDJSON_BORDERSTRIPWIDTH

   #define OFFNAO_FIELD_LENGTH_OFFSET FIELDJSON_BORDERSTRIPWIDTH + 30
   #define OFFNAO_FIELD_WIDTH_OFFSET FIELDJSON_BORDERSTRIPWIDTH + 30

   /** Goal box */
   #define GOAL_BOX_LENGTH FIELDJSON_GOALBOXAREALENGTH
   #define GOAL_BOX_WIDTH FIELDJSON_GOALBOXAREAWIDTH

   /** Penalty Cross */
   #define PENALTY_CROSS_DIMENSIONS FIELDJSON_PENALTYCROSSSIZE /* i.e. dimensions of square fitted around it */
   #define DIST_GOAL_LINE_TO_PENALTY_CROSS FIELDJSON_PENALTYCROSSDISTANCE /* to middle of closest penalty cross */
   #define PENALTY_CROSS_ABS_X (FIELD_LENGTH / 2 - DIST_GOAL_LINE_TO_PENALTY_CROSS)

   /** Center Circle */
   #define CENTER_CIRCLE_DIAMETER FIELDJSON_CENTERCIRCLEDIAMETER

   /** Goal Posts */
   #define GOAL_POST_DIAMETER FIELDJSON_POSTDIAMETER
   #define GOAL_BAR_DIAMETER 100  // Double check this once field is built
   #define GOAL_POST_HEIGHT FIELDJSON_HEIGHT // Measured from the bottom of the crossbar to the ground

   #define GOAL_SUPPORT_DIAMETER 46
   #define GOAL_WIDTH FIELDJSON_INNERWIDTH /* top view end-to-end from middle of goal posts */
   #define GOAL_DEPTH FIELDJSON_DEPTH /* Measured from the front edge of the crossbar to the centre of the rear bar */

   #define PENALTY_AREA_LENGTH FIELDJSON_PENALTYAREALENGTH // Measured from inside of goal line to outside of penalty box
   #define PENALTY_AREA_WIDTH FIELDJSON_PENALTYAREAWIDTH  // Measured from the outside of one side of the penalty box line to the inside of the line on the other side

#elif !defined(USING_SMALL_FIELD)
   #define ROBOTS_PER_TEAM 6

   // When updating these constants make sure you update the constants in simserver/app.py as well
   // https://github.com/UNSWComputing/rUNSWift/blob/master/utils/simserver/app.py

   /** Field line dimensions */
   #define FIELD_LENGTH 9010
   #define FIELD_WIDTH 6020

   #define FIELD_LENGTH_OFFSET 700
   #define FIELD_WIDTH_OFFSET 700

   #define OFFNAO_FIELD_LENGTH_OFFSET 730
   #define OFFNAO_FIELD_WIDTH_OFFSET 730

   /** Goal box */
   #define GOAL_BOX_LENGTH 615
   #define GOAL_BOX_WIDTH 2220

   /** Penalty Cross */
   #define PENALTY_CROSS_DIMENSIONS 100 /* i.e. dimensions of square fitted around it */
   #define DIST_GOAL_LINE_TO_PENALTY_CROSS 1290 /* to middle of closest penalty cross */
   #define PENALTY_CROSS_ABS_X (FIELD_LENGTH / 2 - DIST_GOAL_LINE_TO_PENALTY_CROSS)

   /** Center Circle */
   #define CENTER_CIRCLE_DIAMETER 1500

   /** Goal Posts */
   #define GOAL_POST_DIAMETER 90
   #define GOAL_BAR_DIAMETER 100  // Double check this once field is built
   #define GOAL_POST_HEIGHT 800 // Measured from the bottom of the crossbar to the ground

   #define GOAL_SUPPORT_DIAMETER 46
   #define GOAL_WIDTH 1565 /* top view end-to-end from middle of goal posts */
   #define GOAL_DEPTH 450 /* Measured from the front edge of the crossbar to the centre of the rear bar */

   #define PENALTY_AREA_LENGTH 1650 // Measured from inside of goal line to outside of penalty box
   #define PENALTY_AREA_WIDTH 4000  // Measured from the outside of one side of the penalty box line to the inside of the line on the other side

   //////////////////////////////////////////////////////////////

   // May need to define white support bar dimensions for field lines

#else
     #define ROBOTS_PER_TEAM 6

   /** Field line dimensions */
   #define FIELD_LENGTH 4240
   #define FIELD_WIDTH 2350

   #define FIELD_LENGTH_OFFSET 130
   #define FIELD_WIDTH_OFFSET 130

   #define OFFNAO_FIELD_LENGTH_OFFSET 130
   #define OFFNAO_FIELD_WIDTH_OFFSET 130

   /** Goal box */
   #define GOAL_BOX_LENGTH 420
   #define GOAL_BOX_WIDTH 1300

   /** Penalty Cross */
   #define PENALTY_CROSS_DIMENSIONS 100 /* i.e. dimensions of square fitted around it */
   #define DIST_GOAL_LINE_TO_PENALTY_CROSS 1300 /* to middle of closest penalty cross */
   #define PENALTY_CROSS_ABS_X (FIELD_LENGTH / 2 - DIST_GOAL_LINE_TO_PENALTY_CROSS)

   /** Center Circle */
   #define CENTER_CIRCLE_DIAMETER 780

   /** Goal Posts */
   #define GOAL_POST_DIAMETER 40
   #define GOAL_BAR_DIAMETER 100  // Double check this once field is built
   #define GOAL_POST_HEIGHT 800 // Measured from the bottom of the crossbar to the ground

   #define GOAL_SUPPORT_DIAMETER 46
   #define GOAL_WIDTH 1600 /* top view end-to-end from middle of goal posts */
   #define GOAL_DEPTH 500 /* Measured from the front edge of the crossbar to the centre of the rear bar */

   // TODO: Add penalty lines to small field and update defs here!
   #define PENALTY_AREA_LENGTH 776 // Measured from inside of goal line to outside of penalty box
   #define PENALTY_AREA_WIDTH 1560  // Measured from the outside of one side of the penalty box line to the inside of the line on the other side

#endif

/** Field dimensions including edge offsets */
#define FULL_FIELD_LENGTH (FIELD_LENGTH + (FIELD_LENGTH_OFFSET * 2))
#define OFFNAO_FULL_FIELD_LENGTH (FIELD_LENGTH + (OFFNAO_FIELD_LENGTH_OFFSET * 2))
#define FULL_FIELD_WIDTH (FIELD_WIDTH + (FIELD_WIDTH_OFFSET * 2))
#define OFFNAO_FULL_FIELD_WIDTH (FIELD_WIDTH + (OFFNAO_FIELD_WIDTH_OFFSET * 2))

/** Ball Dimensions */
#define BALL_RADIUS 50

/** Post positions in AbsCoord */
#define GOAL_POST_ABS_X (FIELD_LENGTH / 2.0) - (FIELD_LINE_WIDTH / 2.0) + (GOAL_POST_DIAMETER / 2.0)  // the front of the goal post lines up with the line (as shown in spl rule book)
#define GOAL_POST_ABS_Y (GOAL_WIDTH / 2)

/** Goal Free Kick Positions in AbsCoord */
#define GOAL_KICK_ABS_X PENALTY_CROSS_ABS_X
#define GOAL_KICK_ABS_Y (GOAL_BOX_WIDTH / 2)

/** Corner Kick Positions in AbsCoord */
#define CORNER_KICK_ABS_X (FIELD_LENGTH / 2)
#define CORNER_KICK_ABS_Y (FIELD_WIDTH / 2)
