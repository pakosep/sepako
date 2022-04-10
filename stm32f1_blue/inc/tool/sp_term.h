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
#define TBOLD 			1
#define TDIM				2
#define TUNDERLINE 	3
#define TBLINK			4
#define TREVERSE		7
#define THIDDEN			8

// kolory czcionki lub t³a
#define TBLACK 		0
#define TRED			1
#define TGREEN		2
#define TYELLOW		3
#define TBLUE			4
#define TMAGENTA	5
#define TCYAN			6
#define	TWHITE		7

#define TRCLS "\x1b[K" // or "\x1b""[K"

void tr_erasescr		( void );
void tr_cls					( u08 cur_onoff);		// kasuj ekran , w³/wy³ kursor
void tr_cursor_hide	( u08 hide );			  // cursor show=0 / hide=1
void tr_locate			( u08 y, u08 x );		// ustaw wiersz i kolumnê
void tr_pen_color		( u08 cl );					// ustaw kolor czcionki

void tr_brush_color	( u08 cl );				  // ustaw kolor t³a
void tr_attr				( u08 atr, u08 fg, u08 bg );// ustaw atrybut: znaku oraz kolor czcionki i t³a

void fill_line			( char ascii, u08 cnt );
void tr_cur_back		(	u08 c );
void tr_cur_forward	(	u08 c );


#endif /* SP_TERM_H_ */

/*
	ESC[y,xH	Cursor position y,x
	ESC[nA		Cursor Up n lines
	ESC[nB		Cursor Down n lines
	ESC[nC		Cursor Forward n characters
	ESC[nD		Cursor Backward n characters
	ESC[y;xf	Cursor position y,x (less frequently used)
	ESC[y;xR	Cursor position report y,x
	ESC[6n		Device status report (cursor pos)(n is constant 'n')
	ESC[s		  Save cursor position
	ESC[u		  Restore cursor position
	ESC[2J		Erase display
	ESC[K		  Erase to end of line
	ESC[nL		Inserts n blank lines at cursor line.	(NANSI)
	ESC[nM		Deletes n lines including cursor line.	(NANSI)
	ESC[n@		Inserts n blank chars at cursor.	(NANSI)
	ESC[nP		Deletes n chars including cursor char.	(NANSI)
	ESC[n;ny	Output char translate			(NANSI)
	
*/
