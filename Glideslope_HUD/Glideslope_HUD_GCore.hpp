// ==============================================================
//
//	Glideslope_HUD (Global Core Header)
//	=============================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include <list>
#include <string>
#include "windows.h"
#include "orbitersdk.h"
#include "Glideslope_HUD_Buttons.hpp"
#include "MFDPersist.hpp"
using namespace std;

#ifndef _GLIDESLOPE_HUD_GCORE_H
#define _GLIDESLOPE_HUD_GCORE_H


//+++++
// Global Persistence core. One of these is instantiated for the whole orbiter session, on the first launch of this MFD type
//+++++

class Glideslope_HUD_GCore {
  public:
    void corePreStep(double SimT,double SimDT,double mjd);

    // Global references ... instantiation and a link to the persistence library (running the linked lists)
    Glideslope_HUD_GCore();
    ~Glideslope_HUD_GCore();
    MFDPersist P;
    char moduleName[32];

  private:
    double coreSimT{ 0.0 };
    double coreSimDT;
};


#endif // _GLIDESLOPE_HUD_GCORE_H
