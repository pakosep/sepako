#include "swiss24x.h"

/*============================================*/
/*========= Character pointers table =========*/
/*============================================*/

TCLISTP font24[256]=
	{font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_ssp,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_spls,
	 font24_blank,
	 font24_smin,
	 font24_sssp,
	 font24_blank,
	 font24_n0,
	 font24_n1,
	 font24_n2,
	 font24_n3,
	 font24_n4,
	 font24_n5,
	 font24_n6,
	 font24_n7,
	 font24_n8,
	 font24_n9,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank,
	 font24_blank};

/*===================================*/
/*========= Characters data =========*/
/*===================================*/

/**  0x20 - 32  - ' '  **/
TCDATA font24_ssp[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x2B - 43  - '+'  **/
TCDATA font24_spls[25]={8,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,1,1,1,1),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,1,1,1,1),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x2D - 45  - '-'  **/
TCDATA font24_smin[25]={8,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x2E - 46  - '.'  **/
TCDATA font24_sssp[25]={8,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x30 - 48  - '0'  **/
TCDATA font24_n0[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,1,0,0,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,0,0,1,1,1),
	b2b(1,1,1,1,1,1,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,1,1,1,1,1,1),
	b2b(0,0,1,1,1,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,1,1,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,1,1,1,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,1,1,0,0),
	b2b(1,1,1,1,1,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(1,1,1,0,0,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,0,0,1,1,1)};

/**  0x31 - 49  - '1'  **/
TCDATA font24_n1[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,1,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,1,0,0),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,1,0,0),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,0,0),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,1,1,1,0,0),
	b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),
	b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x32 - 50  - '2'  **/
TCDATA font24_n2[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,1,1,1,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,1,1,1,0),
	b2b(0,0,1,1,1,1,1,1),b2b(0,0,0,0,0,0,0,0),b2b(0,0,1,1,1,1,1,0),
	b2b(0,1,1,1,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,0,0,0,0),
	b2b(1,1,1,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,1,0,0,0,0,0),
	b2b(1,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,1,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,1,1,1,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,1,1,0,0),b2b(1,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,1,1,1,1,0,0,0),b2b(0,1,1,1,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,1,1,0,0,0,0),b2b(0,0,1,1,1,1,1,1),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,1,1,1,1,1)};

/**  0x33 - 51  - '3'  **/
TCDATA font24_n3[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,1,1,1,1,1,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,1,1,1,0,0),
	b2b(0,0,0,0,1,1,1,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,1,1,0,0),
	b2b(0,0,0,0,0,1,1,1),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,1,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(1,1,0,1,1,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,0,1,1,1,0),b2b(1,1,0,0,1,1,0,0),b2b(0,1,1,1,1,1,1,1),
	b2b(1,1,1,1,1,1,0,0),b2b(0,0,0,0,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0)};

/**  0x34 - 52  - '4'  **/
TCDATA font24_n4[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,1),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,1,1,1),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,1,1,1,1,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,1,1,1,1,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(1,1,1,0,0,0,0,0),b2b(0,0,0,0,0,0,0,1),
	b2b(1,1,0,0,0,0,0,0),b2b(1,0,0,0,0,0,0,0),b2b(0,0,0,0,0,1,1,1),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,1,1,1,1,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,1,0,0,0),
	b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),
	b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),b2b(1,1,1,1,1,1,1,1),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x35 - 53  - '5'  **/
TCDATA font24_n5[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,1,1,1,1,1,0,0),b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,0,1),
	b2b(0,0,0,1,1,1,1,0),b2b(1,1,1,1,0,0,0,0),b2b(1,1,1,1,1,1,1,1),
	b2b(0,0,0,0,0,1,1,1),b2b(0,1,1,0,0,0,0,0),b2b(1,1,1,1,1,1,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(1,1,1,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,1,1,1,1,0),b2b(0,1,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(1,1,1,1,1,1,0,0),b2b(0,0,1,1,1,1,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(1,1,1,0,0,0,0,0),b2b(0,0,0,0,1,1,1,1),b2b(0,0,0,0,0,0,0,0)};

/**  0x36 - 54  - '6'  **/
TCDATA font24_n6[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,1,0,0,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,0,0,0,1,1),
	b2b(1,1,1,1,1,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(0,0,1,1,1,1,1,0),b2b(0,0,1,1,1,0,0,0),b2b(0,0,1,1,1,1,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,1,1,0,0,0,0,0),b2b(0,1,1,1,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(1,1,1,0,0,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,1,1,1,1,0),b2b(0,1,1,1,0,0,0,0),b2b(0,1,1,1,0,0,0,0),
	b2b(1,1,1,1,1,1,0,0),b2b(0,0,1,1,1,1,1,1),b2b(0,0,0,1,1,0,0,0),
	b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,1,1,1,1),b2b(0,0,0,0,0,0,0,0)};

/**  0x37 - 55  - '7'  **/
TCDATA font24_n7[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,1,1,1,1,1),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(1,1,1,0,0,0,0,0),b2b(0,0,0,1,1,1,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,1,1,1,1,1,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(1,1,1,0,0,0,0,0),b2b(1,1,0,0,0,0,0,1),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,0,0,1,1,1),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,0,1,1,1,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,1,1,1,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(1,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};

/**  0x38 - 56  - '8'  **/
TCDATA font24_n8[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0),
	b2b(1,1,1,1,1,1,0,0),b2b(1,0,0,0,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(0,0,0,0,1,1,1,0),b2b(1,1,0,1,1,1,0,0),b2b(0,1,1,1,1,1,1,1),
	b2b(0,0,0,0,0,1,1,1),b2b(1,1,1,1,1,0,0,0),b2b(0,1,1,1,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,1,1,0,0,0,0),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,1),b2b(1,1,1,1,1,0,0,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,0,1,1,1,0),b2b(1,1,0,1,1,1,0,0),b2b(0,1,1,1,1,1,1,1),
	b2b(1,1,1,1,1,1,0,0),b2b(1,0,0,0,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,0,0)};

/**  0x39 - 57  - '9'  **/
TCDATA font24_n9[40]={13,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(1,1,1,1,0,0,0,0),b2b(0,0,0,0,1,1,1,1),
	b2b(0,0,0,1,1,0,0,0),b2b(1,1,1,1,1,1,0,0),b2b(0,0,1,1,1,1,1,1),
	b2b(0,0,0,1,1,1,1,0),b2b(0,0,0,0,1,1,1,0),b2b(0,1,1,1,1,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,1,1,1),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,1,1),b2b(0,0,0,0,0,0,1,1),b2b(1,1,0,0,0,0,0,0),
	b2b(0,0,0,0,0,1,1,0),b2b(0,0,0,0,0,1,1,0),b2b(0,1,1,0,0,0,0,0),
	b2b(0,0,1,1,1,1,0,0),b2b(0,0,0,1,1,1,0,0),b2b(0,1,1,1,1,1,0,0),
	b2b(1,1,1,1,1,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,1,1,1,1,1),
	b2b(1,1,0,0,0,0,0,0),b2b(1,1,1,1,1,1,1,1),b2b(0,0,0,0,0,1,1,1)};

/**  0x100 - 256  **/
TCDATA font24_blank[25]={8,
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),
	b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0),b2b(0,0,0,0,0,0,0,0)};
