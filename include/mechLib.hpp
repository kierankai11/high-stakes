#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_

void baseMove(double L, double R, double p_KP);
void baseMove(double L, double R);
void baseTurn(double bearing, double p_KP);
void baseTurn(double bearing);

#define INPERDEG 0.0359791667
#define DIS 6.5 
#define TORAD 0.01745329251

#define DEFAULTKP 0.35
#define TURNKP    0.7
#define LEEWAY 20

#endif

