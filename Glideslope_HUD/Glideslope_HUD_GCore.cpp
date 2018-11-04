// ==============================================================
//
//	Glideslope_HUD (Core Persistence)
//	================================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include "Glideslope_HUD_GCore.hpp"
#include "Glideslope_HUD_VCore.hpp"

Glideslope_HUD_GCore::Glideslope_HUD_GCore() {
  return;
}

Glideslope_HUD_GCore::~Glideslope_HUD_GCore() {
  return;
}


void Glideslope_HUD_GCore::corePreStep(double simT,double simDT,double mjd) {
  if (coreSimT == 0) {
    coreSimT = simT;
    return;
  }
  if (coreSimT == simT) return;

  if (P.firstVC() == NULL) return; // No vessels interested in Glideslope_HUD yet

  coreSimDT = simT - coreSimT;
  coreSimT = simT;
  //sprintf(oapiDebugString(),"GCORE PRESTEP: %15.15f", coreSimDT);

  // Once per update - call vessel corePreSteps
  for (Glideslope_HUD_VCore* VC = (Glideslope_HUD_VCore*) P.firstVC(); VC != NULL; VC = (Glideslope_HUD_VCore*) P.nextVC()) {
    VC->corePreStep(coreSimT, coreSimDT, mjd);
  }

}
