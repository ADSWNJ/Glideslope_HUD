// ==============================================================
//
//	Glideslope_HUD (Button Handling Code)
//	===============================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include "Glideslope_HUD.hpp"
#include <math.h>

// ==============================================================
// MFD button hooks to Button Page library
//
char* Glideslope_HUD::ButtonLabel (int bt)
{
	return LC->B.ButtonLabel(bt);
}

// Return button menus
int Glideslope_HUD::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	return LC->B.ButtonMenu(menu);
}

// Return clicked button
bool Glideslope_HUD::ConsumeButton (int bt, int event) {
  return LC->B.ConsumeButton(this, bt, event);
}

// Return pressed keystroke
bool Glideslope_HUD::ConsumeKeyBuffered (DWORD key) {
  return LC->B.ConsumeKeyBuffered(this, key);
}



// ==============================================================
// MFD Button Handler Callbacks
//

// POS = Set Position
void Glideslope_HUD::Button_POS() {
  switch (VC->toggle) {
  case 0:
    VC->setPos(0,0, 10000, 1000, 270);
    break;
  case 1:
    VC->setPos(0,0, 20000, 1000, 270);
    break;
  case 2:
    VC->setPos(0,0, 30000, 1000, 270);
    break;
  case 3:
    VC->setPos(0,0, 40000, 1000, 270);
    break;
  case 4:
    VC->setPos(0,0, 50000, 1000, 270);
    break;
  case 5:
    VC->setPos(0,0, 60000, 1000, 270);
    break;
  case 6:
    VC->setPos(0,0, 70000, 1000, 270);
    break;
  case 7:
    VC->setPos(0,0, 80000, 1000, 270);
    break;
  }
  return;
};

// HLD = toggle POS hold
void Glideslope_HUD::Button_HLD() {
  VC->toggle++;
  if (VC->toggle == 8) VC->toggle = 0;
/*  VC->holdPos = !VC->holdPos;
  VC->holdPosCnt++; */
  return;
};

// ROT
void Glideslope_HUD::Button_ROT() {
  // manual edit in ArotD
  VC->ArotD = _V(0, 0, 0);
  VC->ArotR = VC->ArotD * RAD;
  return;
};
