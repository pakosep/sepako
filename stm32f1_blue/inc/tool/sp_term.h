/*
 * sp_term.h
 *
 *  Created on: 02-05-2013
 *      Author: admin
 */

#ifndef SP_TERM_H_
#define SP_TERM_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"

// atrybuty znaku
#define BOLD 			1
#define DIM				2
#define UNDERLINE 3
#define BLINK			4
#define REVERSE		7
#define HIDDEN		8

// kolory czcionki lub t³a
#define TBLACK 		0
#define TRED			1
#define TGREEN		2
#define TYELLOW		3
#define TBLUE			4
#define TMAGENTA	5
#define TCYAN			6
#define	TWHITE		7

void tr_erasescr		(void);
void tr_cls					(u08 cur_onoff);	// kasuj ekran, kursor
void tr_cursor_hide	(u08 hide);				// cursor show=0 / hide=1
void tr_locate			(u08 y, u08 x);		// ustaw wiersz i kolumn
void tr_loc					(u08 y, u08 x);		// ustaw wiersz i kolumn
void tr_pen_color		(u08 cl);					// ustaw kolor czcionki

void tr_brush_color	(u08 cl);					// ustaw kolor tla
void tr_attr				(u08 atr, u08 fg, u08 bg );// ustaw atrybut: znaku oraz kolor czcionki i tla

void fill_line			(char ascii, u08 cnt);
void tr_cur_back		(u08 c);
void tr_cur_forward	(u08 c);

#endif /* SP_TERM_H_ */


