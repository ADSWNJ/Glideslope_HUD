// ====================================================================================================================//
//	Glideslope_HUD MFD
//	=================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	Description:
//
//	This module adds HUD capabilities to the Glideslope MFD. It is modeled loosely on
//  the Space Shuttle HUD, with some simplifications.
//
//	Copyright Notice: 
//
//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	For full licencing terms, pleaserefer to the GNU General Public License
//	(gpl-3_0.txt) distributed with this release, or see
//	http://www.gnu.org/licenses/.
//
//
//	Credits:
//	
//	Orbiter Simulator	(c) 2003-2018 Martin (Martins) Schweiger
// 	MFDButtonPage		  (c) 2012-2018 Szymon (Enjo) Ender
// 	HUDDrawerSDK	  	(c) 2012-2018 Szymon (Enjo) Ender
//	
//
//	Release History:
//
//  V0.01	Alpha
// ====================================================================================================================

#define STRICT
#define ORBITER_MODULE

#include "windows.h"
#include "orbitersdk.h"
#include "Glideslope_HUD.hpp"
#include "Glideslope_HUD_GCore.hpp"
#include "Glideslope_HUD_VCore.hpp"
#include "Glideslope_HUD_LCore.hpp"
#include "ParseFunctions.h"
#include "MFDPersist.hpp"

// ====================================================================================================================
// Global variables

Glideslope_HUD_GCore *g_SC = nullptr;    // points to the static persistence core
char *g_moduleName = "Glideslope_HUD";
char *g_moduleVersion = "1.0";
char *g_moduleCompileDate = __DATE__;

// ====================================================================================================================
// MFD class implementation

// Constructor executes on any F8, any resize of an ExtMFD, or any vessel switch
Glideslope_HUD::Glideslope_HUD (DWORD w, DWORD h, VESSEL *vessel, UINT mfd)
: MFD2 (w, h, vessel)
{
  if (g_SC == nullptr) {
    g_SC = new Glideslope_HUD_GCore();
    GC = g_SC;
    strcpy_s(GC->moduleName, 32, g_moduleName);
  }
  GC = g_SC;


  VC = (Glideslope_HUD_VCore*) GC->P.findVC(vessel);		  // Locate our vessel core
  if (!VC) {
    VC = new Glideslope_HUD_VCore(vessel, GC);				    // ... if missing, initialize it.
    GC->P.addVC(vessel, VC);
  }

  LC = (Glideslope_HUD_LCore*) GC->P.findLC(vessel, mfd);	// Locate our local (vessl+MFD position) core
  if (!LC) {
    LC = new Glideslope_HUD_LCore(vessel, mfd, GC);			  // ... if missing, initialize it.
    GC->P.addLC(vessel, mfd, LC);
  }

  // Any construction for the display side of this MFD instance
  font = oapiCreateFont (h/25, true, "Fixed", FONT_NORMAL, 0);

  return;
}

Glideslope_HUD::~Glideslope_HUD ()
{
  oapiReleaseFont(font);
  //for (int i = 0; i < 12; i++) oapiReleasePen(pen[i]);
  return;
}





// ====================================================================================================================
// Save/load from .scn functions
void Glideslope_HUD::ReadStatus(FILEHANDLE scn) {

  char *line;
  char *ll;
  char *key;
  int pI;
  double pD;

  while (oapiReadScenario_nextline(scn, line)) {

    ll = line;
    if (!ParseString(&ll, &key)) break;
    if (!_stricmp(key, "END_MFD")) break;

    if (!_stricmp(key, "AP_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0 || pI == 1 || pI == 2) VC->apState = pI;
      continue;
    }

    if (!_stricmp(key, "LOG_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0) {
        if (VC->logState == 1) {
          VC->logClose();
        }
      } else if (pI == 1) {
        if (VC->logState == 0) {
          VC->logOpen();
        }
      }
      continue;
    }

    if (!_stricmp(key, "DIAG_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0) VC->showDiags = false;
      else if (pI == 1) VC->showDiags = true;
      continue;
    }

    if (!_stricmp(key, "DP_TGT")) {
      if (!ParseDouble(&ll, &pD)) continue;
      if (pD < 1.0) continue;
      if (pD > 200.0) continue;
      VC->DPTgt = pD;
      continue;
    }

    if (!_stricmp(key, "VACC_TGT")) {
      if (!ParseDouble(&ll, &pD)) continue;
      if (pD < -30.0) continue;
      if (pD > 30.0) continue;
      VC->vAccTgt = pD;
      continue;
    }

  }

  return;
  
}

void Glideslope_HUD::WriteStatus(FILEHANDLE scn) const {

  oapiWriteScenario_float(scn, "DP_TGT", VC->DPTgt);
  oapiWriteScenario_float(scn, "VACC_TGT", VC->vAccTgt);
  oapiWriteScenario_int(scn, "AP_MODE", VC->apState);
  oapiWriteScenario_int(scn, "LOG_MODE", VC->logState);
  oapiWriteScenario_int(scn, "DIAG_MODE", VC->showDiags? 1 : 0);

  return;
}
