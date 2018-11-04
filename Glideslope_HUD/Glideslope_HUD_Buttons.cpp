// ==============================================================
//
//	Glideslope_HUD (MFD Button Management)
//	=====================================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include "MFDButtonPage.hpp"
#include "Glideslope_HUD_Buttons.hpp"
#include "Glideslope_HUD.hpp"


Glideslope_HUD_Buttons::Glideslope_HUD_Buttons() 
{
    static const MFDBUTTONMENU mnu0[] =
    {
      {"Position", 0, 'P'},
      {"Hold Pos", 0, 'H'},
      {"Enter Rot", 0, 'R'}
    };
    RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));
    RegisterFunction("POS", OAPI_KEY_P, &Glideslope_HUD::Button_POS);
    RegisterFunction("HLD", OAPI_KEY_P, &Glideslope_HUD::Button_HLD);
    RegisterFunction("ROT", OAPI_KEY_P, &Glideslope_HUD::Button_ROT);

    // Page 2, etc...
    //static const MFDBUTTONMENU mnu1[] =
    //{
    //  { "Mode Select", 0, 'M' }
    //};
    //RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));
    //RegisterFunction("MOD", OAPI_KEY_M, &Glideslope_HUD::Button_MOD);

    return;
}

bool Glideslope_HUD_Buttons::SearchForKeysInOtherPages() const
{
    return false;
}