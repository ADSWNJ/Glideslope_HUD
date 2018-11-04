// ==============================================================
//
//	Glideslope_HUD (Button Handling Headers)
//	==================================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================



#ifndef _GLIDESLOPE_HUD_BUTTON_CLASS
#define _GLIDESLOPE_HUD_BUTTON_CLASS
#include "MFDButtonPage.hpp"

class Glideslope_HUD;

class Glideslope_HUD_Buttons : public MFDButtonPage<Glideslope_HUD>
{
  public:
    Glideslope_HUD_Buttons();
  protected:
    bool SearchForKeysInOtherPages() const;
  private:
};
#endif // _GLIDESLOPE_HUD_BUTTON_CLASS

