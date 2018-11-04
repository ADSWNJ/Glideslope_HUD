// ==========================================================================
//
//	Glideslope_HUD (Local (Vessel+MFD Panel) Core Persistence)
//	=========================================================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==========================================================================

#include "Glideslope_HUD_GCore.hpp"
#include "Glideslope_HUD_VCore.hpp"
#include "Glideslope_HUD_LCore.hpp"

Glideslope_HUD_LCore::Glideslope_HUD_LCore(VESSEL *vin, UINT mfdin, Glideslope_HUD_GCore* gcin) {
  GC = gcin;
  v = vin;
  m = mfdin;
  VC = (Glideslope_HUD_VCore*)GC->P.findVC(v);
  return;
}
