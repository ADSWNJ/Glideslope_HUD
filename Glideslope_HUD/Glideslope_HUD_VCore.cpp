// ==============================================================
//
//	Glideslope_HUD (Vessel Core Persistence)
//	=======================================
//
//	Copyright (C) 2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See Glideslope_HUD.cpp
//
// ==============================================================

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include "Glideslope_HUD_GCore.hpp"
#include "Glideslope_HUD_VCore.hpp"
#include "ParseFunctions.h"


Glideslope_HUD_VCore::Glideslope_HUD_VCore(VESSEL *vin, Glideslope_HUD_GCore* gcin) : cf(vin) {
	// Vessel core constructor
  GC = gcin;
	v = vin;
  strcpy_s(clName, 128, v->GetClassName());

  char buf[256];

  for (int i = 0; i < MAX_HIST; i++) {
    simT_hist[i] = 0.0;
    alt_hist[i] = v->GetAltitude();
    DP_hist[i] = v->GetDynPressure() / 1000.0;
  }
  maxMach = mach = v->GetMachNumber();
  startSumAbsErr = lastSumAbsErr = oapiGetSimTime();
  //dumpMW();

  if (!GetVesselClassControlSettings()) { // validate the info is there
    sprintf_s(buf, 256, "   >>> %s module could not load ref settings for vessel class %s; using defaults", GC->moduleName, clName);
    oapiWriteLog(buf);
  }

  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      MW_ControlTable[r][c] = MW_DesiredVAccTable[MW_ControlStateTable[r][c] + 4];
    }
  }
  histIx = 0;
  histOld = 1;
  histMid = (reqHist + 1) / 2 ;

  desired_llad = _V(-80.6489808, 28.5728722, 60000); // KSC @ 60km
  desired_ahdd = _V(45, 2000, 100);
  holdPos = false;
  holdPosCnt = 0;
  return;
};

Glideslope_HUD_VCore::~Glideslope_HUD_VCore() {

}


void Glideslope_HUD_VCore::corePreStep(double p_simT,double p_simDT,double p_mjd) {


  if (holdPos) setPos(0, 0, 0, 0, 0);

  mjd = p_mjd;
  simT = p_simT;
  simDT = simT - lastSimT;
  if (simDT < minSimD) return; // Don't run too aggressively

  lastSimT = simT;

  TAS = v->GetAirspeed();
  mach = v->GetMachNumber();
  if (maxMach < mach) maxMach = mach;
  DP = v->GetDynPressure() / 1000.0;

  if (lastSumAbsErr + 1.0 < simT) {
    sumAbsErr += abs(DP - DPTgt);
    lastSumAbsErr = simT;
  }

  trim = v->GetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM);
  alt = v->GetAltitude();

  alt_hist[histIx] = alt;
  DP_hist[histIx] = DP;
  simT_hist[histIx] = simT;

  if (simT_hist[reqHist - 1] == 0.0) {
    histIx = (histIx + 1) % reqHist;
    histOld = (histOld + 1) % reqHist;
    histMid = (histMid + 1) % reqHist;
    return; // Want to populate the history buffer before doing any control activity
  }

  altAvg = 0.0;
  vSpdAvg = 0.0;
  vAccAvg = 0.0;
  DPSpdAvg = 0.0;
  DPAccAvg = 0.0;

  for (int i = 0; i < reqHist; i++) {
    altAvg += alt_hist[i];
  }
  altAvg /= (double) reqHist;
   
  double vSpdH1 = (alt_hist[histMid] - alt_hist[histOld]) / (simT_hist[histMid] - simT_hist[histOld]);
  double vSpdH2 = (alt_hist[histIx] - alt_hist[histMid]) / (simT_hist[histIx] - simT_hist[histMid]);
  double DPSpdH1 = (DP_hist[histMid] - DP_hist[histOld]) / (simT_hist[histMid] - simT_hist[histOld]);
  double DPSpdH2 = (DP_hist[histIx] - DP_hist[histMid]) / (simT_hist[histIx] - simT_hist[histMid]);
  double simTH1 = (simT_hist[histMid] + simT_hist[histOld]) / 2.0;
  double simTH2 = (simT_hist[histIx] + simT_hist[histMid]) / 2.0;
  double a = oapiGetTimeAcceleration();
  if (a < 1.0) a = 1.0;

  vAccAvg = (vSpdH2 - vSpdH1) / (simTH2 - simTH1);
  vSpdAvg = (alt_hist[histIx] - alt_hist[histOld]) / (simT_hist[histIx] - simT_hist[histOld]);
  DPAccAvg = (DPSpdH2 - DPSpdH1) / (simTH2 - simTH1);
  DPSpdAvg = (DP_hist[histIx] - DP_hist[histOld]) / (simT_hist[histIx] - simT_hist[histOld]);

  histIx = (histIx + 1) % reqHist;
  histOld = (histOld + 1) % reqHist;
  histMid = (histMid + 1) % reqHist;

#define fwd_s (2.0 * a)
  DPDelta = DP + DPSpdAvg * fwd_s + 0.5 * DPAccAvg * fwd_s * fwd_s - DPTgt;  /// Predict DP delta in 'fwd' secs
  vAccMW = Macvicar_Whelan_Fuzzy_Function(DPDelta, DPSpdAvg); // drive VSpd from the main fuzzy logic control function

  if (apState == 1) {
    vAccTgt = vAccMW;
    if (a > 1.0) {
      warpLock = true;
      oapiSetTimeAcceleration(1.0);
    }

 //   if (vAccTgt > vAccPrevTgt + 3.0 * simDT) {
 //     vAccTgt = vAccPrevTgt + 3.0 * simDT
 //   } else if (vAccTgt < vAccPrevTgt - 3.0 * simDT) {
 //       vAccTgt = vAccPrevTgt - 3.0 * simDT;
 //   }
  }

  if (apState > 0) {
    double trim_delta = 0.000;
    double absVaccErr = abs(vAccTgt - vAccAvg);
    for (int i = 0; i < 5; i++) {
      if (absVaccErr > trim_ControlTable[i][0]) {
        trim_delta = trim_ControlTable[i][1] * simDT / (100.0);
      } else {
        break;
      }
    }

    if (vAccTgt < vAccAvg) trimTgt -= trim_delta;
    else trimTgt += trim_delta;
    if (trimTgt > 1.0) trimTgt = 1.0;
    else if (trimTgt < -1.0) trimTgt = -1.0;

    v->SetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM, trimTgt);
  }

  if (logState == 1) logWrite();
  vAccPrevTgt = vAccTgt;


  return;
}


void Glideslope_HUD_VCore::setPos(double lon, double lat, double alt, double vel, double azimuth)
{
  VESSELSTATUS2 vs2, vs2p; 
  vs2.version = 2;
  vs2.flag = 0;
  v->GetStatusEx(&vs2);

  VECTOR3 tgt_llad = _V(lon, lat, alt);
  VECTOR3 tgt_ahdd = _V(azimuth, vel, 0.0);

  VECTOR3 tgt_llr = _V(lon * RAD, lat * RAD, oapiGetSize(v->GetGravityRef()));
  double circle_radius = tgt_llr.z;
  double circle_lat_rad = tgt_llr.y;
  double circle_circum = 2.0 * PI * circle_radius * cos(circle_lat_rad);
  double rot_period = oapiGetPlanetPeriod(v->GetGravityRef());
  double ground_rot_speed = circle_circum / rot_period;


  VECTOR3 desired_rpos = cf.cnv(RPOS, LLAD, tgt_llad);
  VECTOR3 desired_rvel = cf.cnv(RPOS, AHDD, tgt_ahdd, RPOS, desired_rpos) - desired_rpos;
  VECTOR3 gs_ahdd = _V(90.0, ground_rot_speed, 0.0);
  VECTOR3 gs_rvel = cf.cnv(RPOS, AHDD, gs_ahdd, RPOS, desired_rpos) - desired_rpos;

  VECTOR3 combined_rvel = desired_rvel + gs_rvel;
  
  vs2.rpos = desired_rpos;
  vs2.rvel = combined_rvel;
  vs2.vrot = _V(0.0, 0.0, 0.0);
  vs2.flag = 0;
  vs2.status = 0;
  v->DefSetStateEx(&vs2);
  return;
  
  /*

  VECTOR3 desired_gpos = cf.cnv(GPOS, RPOS, desired_rpos);
  VECTOR3 desired_gposv = cf.cnv(GPOS, RPOS, desired_rposv);
  VECTOR3 desired_gvel = desired_gposv - desired_gpos;

  

  VECTOR3 xhat, yhat, zhat;
  zhat = desired_gvel; // want to go in primary Z axis (remember: x right, y up, z forward) 
  normalise(zhat);
  yhat = desired_rpos; // initially use z and the radial to find x
  normalise(yhat);
  xhat = crossp(yhat, zhat); // x orthogonal to z and radial
  normalise(xhat);
  yhat = crossp(zhat, xhat); // now fix up y to be orthogonal to x and z
  normalise(yhat);
  double gamma = atan2(xhat.y, xhat.x);
  MATRIX3 M3 = _M(cos(gamma), sin(gamma), 0.0, -sin(gamma), cos(gamma), 0.0, 0.0, 0.0, 1.0);
  xhat = mul(M3, xhat);
  yhat = mul(M3, yhat);
  zhat = mul(M3, zhat);
  double beta = atan2(zhat.z, zhat.x);
  MATRIX3 M2 = _M(cos(beta), 0.0, -sin(beta),  0.0, 1.0, 0.0,  sin(beta), 0.0, cos(beta));
  xhat = mul(M2, xhat);
  yhat = mul(M2, yhat);
  zhat = mul(M2, zhat);
  double alpha = atan2(-zhat.y, zhat.z);
  MATRIX3 M1 = _M(1.0, 0.0, 0.0,  0.0, cos(alpha), sin(alpha), 0.0, -sin(alpha), cos(alpha));
  xhat = mul(M1, xhat);
  yhat = mul(M1, yhat);
  zhat = mul(M1, zhat);
  rot_act = vs2.arot;
  rot_test = _V(alpha, beta, gamma);
*/

/*
  vs2p.version = 2;
  vs2p.flag = 0;
  v->GetStatusEx(&vs2p);

  MATRIX3 rm;
  v->GetRotationMatrix(rm);
  alpha = atan2(rm.m23, rm.m33);
  beta = -asin(rm.m13);
  gamma = atan2(rm.m12, rm.m11);
  rot_test = _V(alpha, beta, gamma);
  vs2p.flag = 0;
  vs2p.status = 0;

  vs2p.arot = rot_test;
  v->DefSetStateEx(&vs2p);

  
  return;
*/
}


void Glideslope_HUD_VCore::dumpMW() {
  FILE *hFile = nullptr;
  errno_t err = fopen_s(&hFile, ".\\Config\\MFD\\Glideslope_HUD\\Glideslope_HUD_MW.csv", "w");
  if (err != 0 || hFile == nullptr) return;

  fprintf(hFile, "E, f(E)\n");

#define Ehi 22.0
#define Elo -10.0
#define Estep -.3
#define dEhi .7
#define dElo -.7
#define dEstep -.03

  for (double E = Ehi; E >= Elo; E += Estep) {
    fprintf(hFile, "%.2f, %.3f\n", E, Macvicar_Whelan_Err_Function(E));
  }
  fprintf(hFile, "\n\ndE, f(dE)\n");
  for (double dE = dEhi; dE >= dElo; dE += dEstep) {
    fprintf(hFile, "%.2f, %.3f\n", dE, Macvicar_Whelan_dErr_Function(dE));
  }

  fprintf(hFile, "\n\nCONTROL TABLE\n");
  fprintf(hFile, "E:, ");
  for (double E = Ehi; E >= Elo; E += Estep) {
    fprintf(hFile, "%.2f, ", E);
  }
  fprintf(hFile, "\ndE:\n");

  for (double dE = dEhi; dE >= dElo; dE += dEstep) {
    fprintf(hFile, "%.2f", dE);
    for (double E = Ehi; E >= Elo; E += Estep) {
      fprintf(hFile, ", %.2f", Macvicar_Whelan_Fuzzy_Function(E, dE));
    }
    fprintf(hFile, "\n");
  }
  fclose(hFile);
}

void Glideslope_HUD_VCore::logOpen() {
  logState = 0;
  errno_t err = fopen_s(&hLogFile, ".\\Config\\MFD\\Glideslope_HUD\\Glideslope_HUD_Dump.csv", "w");
  if (err != 0 || hLogFile == nullptr) {
    logError(err, "trying to open .\\Config\\MFD\\Glideslope_HUD\\Glideslope_HUD_Dump.csv at MJD %.5f", mjd);
    hLogFile = nullptr;
    return;
  }

  int bytesWritten;
  bytesWritten = fprintf(hLogFile, "MJD, T, DT, AP, Mach, TAS, Alt, VSpd, VAcc,    DP_tgt, DP_act, DP_spd, DP_acc,   Trim_act, Trim_tgt, VAcc_tgt, MW_Seg(E), MW_Seg(DE), MW_tl, MW_bl, MW_tr, MW_br, MW_tl%%, MW_bl%%, MW_tr%%, MW_br%%,  T, MW_Out, DP_d, VAcc, SumAbsErr\n");
  if (bytesWritten < 0) {
    logError("writing MJD %.5f", mjd);
    return;
  }
  logState = 1;
  return;
}

void Glideslope_HUD_VCore::logWrite() {
  int bytesWritten;
  double fE = Macvicar_Whelan_Err_Function(DPDelta);
  double fdE = Macvicar_Whelan_dErr_Function(DPSpdAvg);
  bytesWritten = fprintf(hLogFile, "%.5f, %.3f, %.5f,    %d,      %.3f, %.1f, %.1f, %.3f, %.3f,          %.1f, %.3f, %.3f, %.3f,          %.4f, %.4f, %.1f, %.2f, %0.2f, %.2f, %.2f,%.2f, %.2f, %.2f, %.2f, %.2f, %.2f,           %.3f, %.2f, %.3f, %.3f, %.3f\n",
                                     mjd, simT, simDT,   apState, mach, TAS, alt, vSpdAvg, vAccAvg,     DPTgt, DP, DPSpdAvg, DPAccAvg,   trim, trimTgt, vAccTgt, fE, fdE, MW_v0, MW_v1, MW_v2, MW_v3, MW_w0, MW_w1, MW_w2, MW_w3,      simT, vAccMW, DPDelta, vAccAvg, sumAbsErr);
  if (bytesWritten < 0) {
    logError("writing log at MJD %.5f", mjd);
  }
  return;
}

void Glideslope_HUD_VCore::logClose() {
  fclose(hLogFile);
  hLogFile = nullptr;
  logState = 0;
  return;
}

void Glideslope_HUD_VCore::logError(errno_t err, const char *fmt...) {
  char buf1[256];
  char fmtBuf[256];
  char buf2[256];
  
  sprintf_s(buf1, "   >>> %s: Error %d (%s) ", GC->moduleName, err, strerror(err));
  sprintf_s(fmtBuf, 256, "%s%s", buf1, fmt);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(buf2, 256, fmtBuf, args);
  va_end(args);
  oapiWriteLog(buf2);
  if (hLogFile) fclose(hLogFile);
  hLogFile = nullptr;
  logState = 2;
  return;
}

void Glideslope_HUD_VCore::logError(const char *fmt...) {
  char buf1[256];
  char fmtBuf[256];
  char buf2[256];

  sprintf_s(buf1, "   >>> %s: Error ", GC->moduleName);
  sprintf_s(fmtBuf, 256, "%s%s", buf1, fmt);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(buf2, 256, fmtBuf, args);
  va_end(args);
  oapiWriteLog(buf2);
  if (hLogFile) fclose(hLogFile);
  hLogFile = nullptr;
  logState = 2;
  return;
}

// My Macvicar-Whelan control matrix is based on work in this 1994 paper:
// "A Fuzzy Supervisor for PD Control of Unknown Systems" Robert P. Copeland, Kuldip S. Rattan of Wright State University
// http://cecs.wright.edu/~krattan/courses/419/louisville.pdf  <<-- nice work, guys, driving a space simulation control loop 23 years later!
//
double Glideslope_HUD_VCore::Macvicar_Whelan_Err_Function(double E) {
  // Generate a double representing the trapezoid fuzzing function from the error, for the Macvicar-Whelan fuzz control
  // Response is in the range -4.0 to 4.0
  // -4.00 represents Negative X-Large (NX) response
  // -3.00 represents Negative Large (NL) response
  // -2.99 .. -2.01 represents the blend of NL and NM
  // -2.00 represents the Negative Medium (NM) response
  // -1.99 .. -1.01 represents the blend of NM to NS
  // -1.00 represents the Negative Small (NS) response
  // -1.00 .. -0.01 rpresents the blend of NS to Zero
  // 0.00 represents the Zero (Z0) response
  // ... etc for positives: PS, PM, PL , PX

  // Input E represents the error in Dyn Pressure. E.g. want  10.0 kPa, current 30.0 kPa => E = 20.0  



  double fuzz = -4.0;

  for (int i = 0; i < 8; i++) {
    if (E >= MW_SegmentationTableE[i].start) break;                                 // pre-ramp... done, and return the whole number
    if (E >= MW_SegmentationTableE[i].end) {
      fuzz += (MW_SegmentationTableE[i].start - E) / (MW_SegmentationTableE[i].start - MW_SegmentationTableE[i].end); // mid-ramp ... done, and return the fraction along ramp
      break;
    } 
    fuzz += 1.0;                                                   // post-ramp ... go to the next test
  }
  return fuzz;
}
double Glideslope_HUD_VCore::Macvicar_Whelan_dErr_Function(double dE) {
  // Generate a double representing the trapezoid fuzzing function from the delta-error, for the Macvicar-Whelan fuzz control
  // Response is in the range -4.0 to 4.0 as the Err function
  // Input dE represents the rate of change of Dyn Pressure. Goal to keep it in the range +-0.5. 

  double fuzz = -4.0;
  for (int i = 0; i < 8; i++) {
    if (dE >= MW_SegmentationTableDE[i].start) break;                                 // pre-ramp... done, and return the whole number
    if (dE >= MW_SegmentationTableDE[i].end) {
      fuzz += (MW_SegmentationTableDE[i].start - dE) / (MW_SegmentationTableDE[i].start - MW_SegmentationTableDE[i].end); // mid-ramp ... done, and return the fraction along ramp
      break;
    }
    fuzz += 1.0;                                                    // post-ramp ... go to the next test
  }
  return fuzz;
}

double Glideslope_HUD_VCore::Macvicar_Whelan_Fuzzy_Function(double E, double dE) {
  // Generate a desired control response from the Macvicar-Whelan fuzzy rule matrix
  // Search for "A Fuzzy Supervisor for PD Control of Unknown Systems" by Robert P. Copeland and Kuldip S. Rattan
  // For us, the control output is desired vertical speed (dVAS), which drives the elevator trim
  // Desired rate for this is represented by these NX..Z0..PX parameters

  double fE = Macvicar_Whelan_Err_Function(E);
  double fdE = Macvicar_Whelan_dErr_Function(dE);


  // find the top left of the 4 entries (i.e. lower index numbers). Fuzz comes in with -4 to +4, we need 0-8, so add 4
  // suppress the C4244 double to int possible loss of data ... we know what we are doing!
#pragma warning(push)
#pragma warning(disable:4244)
  int x = 4.0 + (int)floor(fdE);
  int y = 4.0 + (int)floor(fE);
#pragma warning(pop)

  // find the fractions of how close we are to the lower or upper index for each range
  double xf1 = fdE - floor(fdE);
  double yf1 = fE - floor(fE);
  double xf0 = 1.0 - xf1;
  double yf0 = 1.0 - yf1;

  // pull the 4 values
  MW_v0 = MW_ControlTable[x][y];
  MW_v1 = MW_ControlTable[x+1][y];
  MW_v2 = MW_ControlTable[x][y+1];
  MW_v3 = MW_ControlTable[x+1][y+1];

  // pull the 4 weights
  MW_w0 = xf0 * yf0;
  MW_w1 = xf1 * yf0;
  MW_w2 = xf0 * yf1;
  MW_w3 = xf1 * yf1;

  MW_E0 = y;
  MW_DE0 = x;
  MW_E0W = yf1;
  MW_DE0W = xf1;

  // do a weighted average of the values 
  MW_RESP = (MW_w0 * MW_v0 + MW_w1 * MW_v1 + MW_w2 * MW_v2 + MW_w3 * MW_v3) / (MW_w0 + MW_w1 + MW_w2 + MW_w3);

  return MW_RESP; 
}

bool Glideslope_HUD_VCore::GetVesselClassControlSettings() {

  FILE* rf;
  char buf[256];
  int tokItem;
  char *tok;
  char *bp;
  double itemValueArray[4][16];

  int lineNo = 0;
  bool parseValid = false;
  bool inVesselClass = false;
  bool inOurVesselClass = false;
  unsigned short foundItem{ 0 };

  const char *tokList[] = { "SEG_E_CTL", "SEG_DE_CTL", "DES_VACC_CTL", "TRIM_CTL", "HIST_COUNT", "MIN_SIMD" , "DP_TGT", "BEGIN_VESSEL_CLASS", "END_VESSEL_CLASS" };
  const int itemValueCount[4] = { 16, 16, 9, 10 };
  int itemHistCount{ 0 };
  double itemMinSimD{ 0.0 };
  int itemDPTgt{ 0 };


  const char VesselParamsFile[] = ".\\Config\\MFD\\Glideslope_HUD\\Glideslope_HUD_VesselParams.cfg";

  try {
    if (fopen_s(&rf, VesselParamsFile, "r")) throw ParseException("could not open file");

    while (fgets(buf, 255, rf) != NULL) {
      lineNo++;
      bp = buf;
      if (!ParseWhiteSpace(&bp)) continue;
      ParseStringFomList(&bp, &tokItem, tokList, 9, true);
      foundItem |= 1 << tokItem;
      switch (tokItem) {
      case 0: // SEG_E_CTL
      case 1: // SEG_DE_CTL
      case 2: // DES_VACC_CTL
      case 3: // TRIM_CTL
        if (!inVesselClass) throw ParseException("%s must be inside a BEGIN_VESSEL_CLASS block", tokList[tokItem]);
        if (!inOurVesselClass) continue;
        ParseJustDoubleArray(&bp, itemValueArray[tokItem], itemValueCount[tokItem], true);
        continue;
      case 4: // HIST_COUNT
        ParseInt(&bp, &itemHistCount, true);
        if (itemHistCount < 3 || itemHistCount > MAX_HIST) throw ParseException("%s range error: must be between 3 and %d", tokList[tokItem], MAX_HIST);
        continue;
      case 5: // MIN_SIMD
        ParseDouble(&bp, &itemMinSimD, true);
        if (itemMinSimD < 0.0 || itemMinSimD > 2.0) throw ParseException("%s range error: must be between 0.0 and 2.0", tokList[tokItem]);
        continue;
      case 6: // DP_TGT
        ParseInt(&bp, &itemDPTgt, true);
        if (itemDPTgt < 0 || itemDPTgt > 200) throw ParseException("%s range error: must be between 3 and 200", tokList[tokItem]);
        continue;
      case 7: // BEGIN_VESSEL_CLASS
        if (inVesselClass) throw ParseException("missing END_VESSEL_CLASS");
        inVesselClass = true;
        ParseQuotedString(&bp, &tok, true);
        if (!_stricmp(tok, clName)) inOurVesselClass = true;
        if (ParseWhiteSpace(&bp)) throw ParseException("extra data found on end of line");
        continue;
      case 8: // END_VESSEL_CLASS
        if (ParseWhiteSpace(&bp)) throw ParseException("extra data found on end of line");
        if (!inVesselClass) throw ParseException("missing BEGIN_VESSEL_CLASS");
        if (inOurVesselClass) {
          if (foundItem == 0x1FF) {
            parseValid = true;
            break; // parsed our class, and all 9 control lines found ... so now break from the file read
          } else {
            throw ParseException("Missing settings in vessel definition");
          }
        }
        inVesselClass = inOurVesselClass = false;
        foundItem = 0;
        continue;
      }
      if (parseValid) break;
    }
  }
  catch (ParseException& e) {
    sprintf_s(buf, 256, "   >>> %s module parse error in %s, line %i: %s", GC->moduleName, VesselParamsFile, lineNo, e.getMsg());
    oapiWriteLog(buf);
  };

  if (rf != nullptr) fclose(rf);

  if (parseValid) {
    for (int i = 0, j=0; j < 8; j++) {
      MW_SegmentationTableE[j].start = itemValueArray[0][i];
      MW_SegmentationTableDE[j].start = itemValueArray[1][i++];
      MW_SegmentationTableE[j].end = itemValueArray[0][i];
      MW_SegmentationTableDE[j].end = itemValueArray[1][i++];
    }
    for (int i = 0; i < 9; i++) {
      MW_DesiredVAccTable[i] = itemValueArray[2][i];
    }
    for (int i = 0, j = 0; i < 5; i++) {
      trim_ControlTable[i][0] = itemValueArray[3][j++];
      trim_ControlTable[i][1] = itemValueArray[3][j++];
    }
    reqHist = itemHistCount;
    minSimD = itemMinSimD;
    DPTgt = itemDPTgt;

    sprintf_s(buf, 256, "   >>> %s module loaded vessel class %s settings from %s", GC->moduleName, clName, VesselParamsFile);
    oapiWriteLog(buf);
  }
  return parseValid;
}


/*  OBJHANDLE ohE = oapiGetObjectByName("Earth");

VECTOR3 gEpos;
oapiGetGlobalPos(ohE, &gEpos);

VECTOR3 gpos, lpos;
oapiLocalToGlobal(ohE, &_V(0, 0, 0), &gpos);
oapiGlobalToLocal(ohE, &gpos, &lpos);

oapiLocalToGlobal(ohE, &_V(0, 0, 0), &gpos);
oapiGlobalToLocal(ohE, &gpos, &lpos);

VECTOR3 ecef, chk_xyz;

VECTOR3 rpos;
v->GetRelativePos(vs2.rbody, rpos);

MATRIX3 M_rpos_to_xyz;
oapiGetRotationMatrix(vs2.rbody, &M_rpos_to_xyz);

chk_xyz = tmul(M_rpos_to_xyz, rpos);


VECTOR3 gVpos = gEpos + rpos;
VECTOR3 lVpos;
oapiGlobalToLocal(ohE, &gVpos, &lVpos);


double pos_lng, pos_lat, pos_rad;
v->GetEquPos(pos_lng, pos_lat, pos_rad);
VECTOR3 my_llr{ pos_lat, pos_lng, pos_rad };


VECTOR3 zz_llad = _V(-74.5, 40.6342, 0);
VECTOR3 zz_lla =   cf.cnv(LLA,  LLAD, zz_llad);
VECTOR3 zz_llr =   cf.cnv(LLR,  LLA,  zz_lla);
VECTOR3 chk_lla =  cf.cnv(LLA,  LLR,  zz_llr);
VECTOR3 chk_llad = cf.cnv(LLAD, LLA,  chk_lla);
chk_llad =         cf.cnv(LLAD, LLR,  zz_llr);

VECTOR3 zz_ecef =  cf.cnv(ECEF, LLR,  zz_llr);
chk_llad =         cf.cnv(LLAD,  ECEF, zz_ecef);

VECTOR3 zz_rpos =  cf.cnv(RPOS, ECEF, zz_ecef);
VECTOR3 chk_ecef = cf.cnv(ECEF, RPOS, zz_rpos);

VECTOR3 tgt_ahdd = _V(135, 1000, 43);
VECTOR3 tgt_ahd =  cf.cnv(AHD,  AHDD, tgt_ahdd);
VECTOR3 tgt_ned =  cf.cnv(NED,  AHD,  tgt_ahd);
VECTOR3 tgt_ecef = cf.cnv(ECEF, NED,  tgt_ned, zz_ecef);

VECTOR3 chk_tgt_ned = cf.cnv(NED, ECEF,  tgt_ecef, zz_ecef);
VECTOR3 chk_tgt_ahd = cf.cnv(AHD, NED,   chk_tgt_ned);
VECTOR3 chk_tgt_ahdd = cf.cnv(AHDD, AHD, chk_tgt_ahd);


ecef = cf.cnv(ECEF, LLAD, _V(0, 0, 1000));
oapiLocalToGlobal(ohE, &ecef, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

/*  xyz = llr_to_xyz(_V( 90 * RAD,  0 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

xyz = llr_to_xyz(_V(  0 * RAD, 90 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

xyz = llr_to_xyz(_V(-90 * RAD,  0 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

xyz = llr_to_xyz(_V(-90 * RAD, 90 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

xyz = llr_to_xyz(_V(  0 * RAD, 30 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

xyz = llr_to_xyz(_V( 30 * RAD,  0 * RAD, 1000));
oapiLocalToGlobal(ohE, &xyz, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);

ecef = cf.llr_to_ecef(llr);
oapiLocalToGlobal(ohE, &ecef, &gpos);
oapiGlobalToLocal(ohE, &gpos, &chk_xyz);


ecef = cf.llr_to_ecef(llr);
*/
