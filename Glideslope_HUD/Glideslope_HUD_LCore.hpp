// ==============================================================
//
//	Glideslope_HUD (Local Core Header)
//	============================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include "windows.h"
#include "orbitersdk.h"
#include "Glideslope_HUD_Buttons.hpp"
#include "MFDPersist.hpp"
#include "Glideslope_HUD_GCore.hpp"
#include "Glideslope_HUD_VCore.hpp"
#include <list>
#include <string>

using namespace std;

#ifndef _GLIDESLOPE_HUD_LCORE_H
#define _GLIDESLOPE_HUD_LCORE_H

//+++++
// Local Persistence core. One of these is instantiated per Vessel AND MFD panel location. Local defaults for that combination.
//+++++

class Glideslope_HUD_LCore {
  public:
    // Local references ... instantiation, references for vesseland mfd position, and links to the appropriate VC, MC and GC
    Glideslope_HUD_LCore(VESSEL *vin, UINT mfdin, Glideslope_HUD_GCore* gcin);
    VESSEL *v;
    UINT m;
    Glideslope_HUD_GCore* GC;
    Glideslope_HUD_VCore* VC;

    // Add local vessel+panel data here

    Glideslope_HUD_Buttons B;
    bool showMessage{ false };
    bool okMessagePage{ true };
    string Message;
    int mode{ 0 };
    int PrvNxtMode;

    oapi::Sketchpad *skp; // points to local sketchpad for this MFD and vessel
    int skpLoB;           // Lowest precision for skp eng numnber formatting
    char skpBuf[128];     // Formatting buffer for MFD updates
    char skpFmtBuf[128];  // Formatting buffer for MFD updates
    int skpColPix;        // X-offset pixel (top left origin)
    int skpLinePix;       // Y offsel pixel (top left origin)

};


#endif // _GLIDESLOPE_HUD_CORE_CLASSES
