#ifndef __RTC_H
#define __RTC_H

#include "inc/stm32f10x.h"
#include "config.h"

#define bRTC_CRL_CNF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 4)
#define bRTC_CRL_RTOFF				bitband_t m_BITBAND_PERIPH(&RTC->CRL, 5)
#define bRTC_CRL_SECF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 0)
#define bPWR_CR_DBP						bitband_t m_BITBAND_PERIPH(&PWR->CR,  8)

//extern const u08 _ytab[2][12];

typedef struct {
    int     tm_sec;         /* sekundy od pe³nej minuty*/
    int     tm_min;         /* minuty  od pe³nej godziny*/
    int     tm_hour;        /* godzina na 24-godzinnym zegarze*/
    int     tm_mday;        /* dzieñ miesi¹ca */
    int     tm_mon;         /* miesi¹c licz¹c od zera */
    int     tm_year;        /* rok - 1900 */
    int     tm_wday;        /* dzieñ tygodnia niedziela ma numer 0*/
    int     tm_yday;        /* dzieñ roku licz¹c od zera*/
    int     tm_isdst;       /* znacznik czasu letniego */
}T_DATETIME;

typedef struct {
	u08		sec;	/* 0..59 */
	u08		min;	/* 0..59 */
	u08		hor;	/* 0..23 */
	u08		mdy;	/* 1..31 */
	u08		mon;	/* 1..12 */
	u08		wdy;	/* 0..6 (Sun..Sat) */
	u16		yer;	/* 1970..2106 */
}T_RTC;

#define _RTC_TDIF	+ 2.0	/* JST = UTC+2.0 */

#pragma pack(push)       /* zapamiêtaj bie¿¹c¹ wartoœæ wyrównania */
#pragma pack(1)          /* brak wyrównania */
/* Nalezy kazdorozowo sprawdzac wyrownanie kompilatora */

typedef struct {					/* struktura 32 bity */
    u08    sec:6;   			/* sekundy od pe³nej minuty*/
    u08    min:6;       	/* minuty  od pe³nej godziny*/
    u08    hor:5;       	/* godzina na 24-godzinnym zegarze*/
    u08    day:5;       	/* dzien miesi¹ca */
		u08    mon:4;       	/* miesiac */
		u08    yer:6;       	/* rok */
		
}T_TIMDAT;	

#pragma pack(pop)					// Przywroc wczesniejsza wartosc wyrownania

void RtcInit(void);
T_RTC * rtc_unix2time 	(u32 uxt, T_RTC *); //Oblicz date z unixtime
u08  rtc_time2unix 	(u32 *uxt, const T_RTC *);

void rtc_GetTime(u08 force , T_RTC *rtc );
void rtc_GetTim2(u08 force , T_RTC *rtc );

void rtc_SetTime(						 T_RTC *rtc );
void SleepMode(u08 sleep);
void BkpRegInit(void);

#endif 
