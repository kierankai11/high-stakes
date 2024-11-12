#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_

void baseMove(double L, double R, double p_KP);
void baseMove(double L, double R);
void baseMove(double X);
void baseTurn(double bearing, double p_KP);
void baseTurn(double bearing);

// UNCOMMET SIDES AND COMMENT NON SIDES
// #define REDNEGATIVE
//  #define REDPOSITIVE
#define BLUENEGATIVE
//#define BLUEPOSITIVE



// GENERAL
#define INPERDEG 0.0359791667
#define DIS 6.5
#define TORAD 0.01745329251

#define DEFAULTKP 0.4
#define TURNKP 0.69
#define LEEWAY 20

#define DEFAULTCAP 100

// UNCOMMENT IF POSITIVE
// #define SECONDDONUT 80
// #define FACEADJUST -100
// #define THIRDDONUT 100

// SPECIFIC SIDE ANGLE VARIABLES
#ifdef REDNEGATIVE
#define QUADRANT 2
#define STAKEANGLE 33
#define FIRSTDONUT 64
#define SECONDDONUT 90
#define FACEADJUST -100
#define THIRDDONUT 95
#define LADDER 90
#endif

#ifdef REDPOSITIVE
#define QUADRANT 3
#define STAKEANGLE -44
#define FIRSTDONUT -50
#define LADDER 215 // should not matter
#endif

#ifdef BLUENEGATIVE
#define QUADRANT 1
#define STAKEANGLE -45
#define FIRSTDONUT -63
#define SECONDDONUT -90
#define FACEADJUST 100
#define THIRDDONUT -95
#define LADDER -90
#endif

#ifdef BLUEPOSITIVE
#define QUADRANT 4
#define STAKEANGLE 33
#define FIRSTDONUT 50
#define LADDER 170 // should not matter
#endif

#endif