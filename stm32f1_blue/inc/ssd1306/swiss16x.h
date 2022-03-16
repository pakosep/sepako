#ifndef swiss16xH
#define swiss16xH

#ifndef SMfgTypes
#define SMfgTypes

/*======= binar input =======*/

/*============================================================================*/
/* You have to manualy set correct types TCDATA and TCLISTP                   */
/* for your platform and compiler.                                            */
/*                                                                            */
/* Keil C51 example:                                                          */
/* Character data (TCDATA) are stored in code memory,                         */
/* array of pointers to characters (TCLISTP) is stored in code memory         */
/* and pointers are pointing into code memory.                                */
/*                                                                            */
/* typedef unsigned char code TCDATA;                                         */
/* typedef TCDATA * code TCLISTP;                                             */
/*============================================================================*/

#include "config.h"

#endif

/*======= Character pointers table =======*/
extern TCLISTP font10x16[256];

/*======= Characters data =======*/
TCDATA font10x16_ssp[21];
TCDATA font10x16_sexc[5];
TCDATA font10x16_sdq[11];
TCDATA font10x16_sfr[21];
TCDATA font10x16_sdlr[21];
TCDATA font10x16_sprc[25];
TCDATA font10x16_sand[25];
TCDATA font10x16_sap[5];
TCDATA font10x16_sprl[11];
TCDATA font10x16_sprr[11];
TCDATA font10x16_sstr[17];
TCDATA font10x16_spls[21];
TCDATA font10x16_scmm[5];
TCDATA font10x16_smin[9];
TCDATA font10x16_sssp[5];
TCDATA font10x16_sslh[15];
TCDATA font10x16_n0[21];
TCDATA font10x16_n1[21];
TCDATA font10x16_n2[21];
TCDATA font10x16_n3[21];
TCDATA font10x16_n4[21];
TCDATA font10x16_n5[21];
TCDATA font10x16_n6[21];
TCDATA font10x16_n7[21];
TCDATA font10x16_n8[21];
TCDATA font10x16_n9[21];
TCDATA font10x16_scln[5];
TCDATA font10x16_ssmc[9];
TCDATA font10x16_sbrl[27];
TCDATA font10x16_seql[21];
TCDATA font10x16_sbrr[27];
TCDATA font10x16_sqst[17];
TCDATA font10x16_szvn[31];
TCDATA font10x16_UA[23];
TCDATA font10x16_UB[19];
TCDATA font10x16_UC[21];
TCDATA font10x16_UD[21];
TCDATA font10x16_UE[19];
TCDATA font10x16_UF[19];
TCDATA font10x16_UG[21];
TCDATA font10x16_UH[21];
TCDATA font10x16_UI[5];
TCDATA font10x16_UJ[15];
TCDATA font10x16_UK[21];
TCDATA font10x16_UL[17];
TCDATA font10x16_UM[27];
TCDATA font10x16_UN[21];
TCDATA font10x16_UO[21];
TCDATA font10x16_UP[19];
TCDATA font10x16_UQ[23];
TCDATA font10x16_UR[21];
TCDATA font10x16_US[19];
TCDATA font10x16_UT[21];
TCDATA font10x16_UU[21];
TCDATA font10x16_UV[23];
TCDATA font10x16_UW[31];
TCDATA font10x16_UX[23];
TCDATA font10x16_UY[21];
TCDATA font10x16_UZ[19];
TCDATA font10x16_sbl[9];
TCDATA font10x16_sbsl[15];
TCDATA font10x16_sbr[9];
TCDATA font10x16_ssqr[27];
TCDATA font10x16_sul[21];
TCDATA font10x16_shc[9];
TCDATA font10x16_la[21];
TCDATA font10x16_lb[21];
TCDATA font10x16_lc[21];
TCDATA font10x16_ld[21];
TCDATA font10x16_le[21];
TCDATA font10x16_lf[21];
TCDATA font10x16_lg[17];
TCDATA font10x16_lh[17];
TCDATA font10x16_li[5];
TCDATA font10x16_lj[13];
TCDATA font10x16_lk[17];
TCDATA font10x16_ll[5];
TCDATA font10x16_lm[29];
TCDATA font10x16_ln[17];
TCDATA font10x16_lo[17];
TCDATA font10x16_lp[17];
TCDATA font10x16_lq[17];
TCDATA font10x16_lr[11];
TCDATA font10x16_ls[15];
TCDATA font10x16_lt[11];
TCDATA font10x16_lu[17];
TCDATA font10x16_lv[19];
TCDATA font10x16_lw[25];
TCDATA font10x16_lx[21];
TCDATA font10x16_ly[17];
TCDATA font10x16_lz[15];
TCDATA font10x16_spsl[21];
TCDATA font10x16_c128[9];
TCDATA font10x16_blank[17];

#endif
