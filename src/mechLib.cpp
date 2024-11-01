#include "mechLib.hpp"
#include "main.h"
#include <cmath>


double TARGL = 0, TARGR = 0, ERRL, ERRR, KP, CURRL, CURRR, LPOW, RPOW;

double abscap(double pow, double cap) { return fmax(fmin(pow, cap), -cap); }

void baseMove(double L, double R, double p_KP) {
  Motor FL(DRIVE_FL_P);
  Motor FR(DRIVE_FR_P);
  Motor BL(DRIVE_BL_P);
  Motor BR(DRIVE_BR_P);

  Controller master(E_CONTROLLER_MASTER);

  TARGL = FL.get_position() + (L / INPERDEG);
  TARGR = FR.get_position() + (R / INPERDEG);

  ERRL = L / INPERDEG;
  ERRR = R / INPERDEG;

  KP = p_KP;

  while (fabs(ERRL) >= LEEWAY || fabs(ERRR) >= LEEWAY) {
    CURRL = FL.get_position();
    CURRR = FR.get_position();
    ERRL = TARGL - CURRL;
    ERRR = TARGR - CURRR;
    LPOW = ERRL * KP;
    RPOW = ERRR * KP;
    FL.move(abscap(LPOW, DEFAULTCAP));
    FR.move(abscap(RPOW, DEFAULTCAP));
    BL.move(abscap(LPOW, DEFAULTCAP));
    BR.move(abscap(RPOW, DEFAULTCAP));

    master.print(2, 0, "L: %.2f", ERRL);
    delay(5);
  }

  master.print(2, 0, "not running");
  FL.move(0);
  FR.move(0);

  BL.move(0);
  BR.move(0);
}

void baseMove(double L, double R) { baseMove(L, R, DEFAULTKP); }

double turnL, turnR;

void baseTurn(double bearing, double p_KP) {
  turnL = bearing * TORAD * DIS;
  turnR = -bearing * TORAD * DIS;
  baseMove(turnL, turnR, p_KP);
}

void baseTurn(double bearing) { baseTurn(bearing, TURNKP); }