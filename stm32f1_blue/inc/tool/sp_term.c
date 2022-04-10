/*
 * sp_term.c
 *
 *  Created on: 02-05-2013
 *  Author: 
 */

//#include "inc/stm32f10x.h"
#include "sp_term.h"
#include "inc/tool/delay.h"
#include "inc/tool/UARTLib.h"

// aby miec polskie ogonki w PUTTY ----> ustaw WINDOW / Translation / Win1250
// http://www.termsys.demon.co.uk/vtansi.htm
//	ESC[K		  Erase to end of line

//const char RCLS[]  			= { "\x1b""[K" };
const char UCLS[]  			= { "\x1b""[2J" };
const char UHOME[] 			= { "\x1b""[;H" };
const char UCUR_HIDE[]  = { "\x1b""[?25l" };
const char UCUR_SHOW[]  = { "\x1b""[?25h" };
const char U_ATTR_OFF[] = { "\x1b""[m" };

void tr_cursor_hide( uint8_t hide ) {
		if(hide) UaPutK( UCUR_HIDE );
		else UaPutK( UCUR_SHOW );
	}

void tr_cls(uint8_t cur_onoff) {

		//UaPutK( U_ATTR_OFF );
		//tr_cursor_hide(cur_onoff);
		UaPutK( UCLS );
		UaPutK( UHOME );
		
	}

void fill_line( char ascii, uint8_t cnt ) {
		for(uint8_t i=0; i<cnt; i++) UaPutC( ascii );
	}
	
void tr_attr( uint8_t atr, uint8_t fg, uint8_t bg ) {
		UaPutC( 0x1b );		// <ESC>[0;32;44m
		UaPutC( '[' );
		UaPutC( atr+'0' );
		UaPutC( ';' );
		UaPutC( '3' );
		UaPutC( fg+'0' );
		UaPutC( ';' );
		UaPutC( '4' );
		UaPutC( bg+'0' );
		UaPutC( 'm' );
	}

void tr_pen_color( uint8_t cl ) {
		UaPutC( 0x1b );		// <ESC>[34m
		UaPutC( '[' );
		UaPutC( '3' );
		UaPutC( cl+'0' );
		UaPutC( 'm' );
	}

void tr_brush_color( uint8_t cl ) {
		UaPutC( 0x1b );		// <ESC>[44m
		UaPutC( '[' );
		UaPutC( '4' );
		UaPutC( cl+'0' );
		UaPutC( 'm' );
	}

void tr_locate( uint8_t y, uint8_t x ) {
		
		//UaPutS("\033[y;xH");
		//UaPutC(0x1b);	// <ESC>[y;xH <ESC>[{ROW};{COLUMN}f
		//UaPutC('[');
		UaPutS("\x1b[");
		uint2uart(y);
		//UaPutC(y+48);
		UaPutC(';');
		//UaPutC(x+48);
		uint2uart(x);
		UaPutC('H');
		//UaPutC('f');
		
	}

void tr_erasescr( void ) {
		UaPutC( 0x1b );		// <ESC>[2J
		UaPutC( '[' );
		UaPutC( '1' );
		UaPutC( 'J' );
	}
	
void tr_cur_back( uint8_t c ) {
		UaPutC( 0x1b );	// <ESC>[{ROW};{COLUMN}f
		UaPutC('['  );
		UaPutC(c+'0');
		UaPutC('D'  );
	}

void tr_cur_forward( uint8_t c ) {
		UaPutC( 0x1b );	// <ESC>[{ROW};{COLUMN}f
		UaPutC('['  );
		UaPutC(c+'0');
		UaPutC('C'  );
	}
	
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
		When first char is encountered in output request, it
		is replaced with the second char.  When no parameters
		are given, all chars are reset.
	ESC["str"p	Keyboard Key Reassignment. The first char of str gives
		the key to redefine; the rest of the string is the
		key's new value.  To specify unprintable chars, give
		the ASCII value of the char outside of quotes, as a
		normal parm.  IBM function keys are two byte strings.

		Ex:  ESC[0;";dir a:";13;p

		redefines F1 to have the value "dir a:" followed by CR.
		If no parameters given, all keys are reset to their
		default values.  Single or double quotes are valid.

	ESC[n;n;...nm	Set Graphics Rendition is used to set attributes as
		well as foreground and background colors.  If multiple
		parameters are used, they are executed in sequence, and
		the effects are cumulative. 'n' is one of the following
		attributes or colors:

		0  All attributes off		5  Blink
		1  Bold				7  Reverse Video
		2  Dim				8  Invisible
		4  Underline


		Foreground colors	Background colors

		    30	Black		    40	Black
		    31	Red		    41	Red
		    32	Green		    42	Green
		    33	Yellow		    43	Yellow
		    34	Blue		    44	Blue
		    35	Magenta		    45	Magenta
		    36	Cyan		    46	Cyan
		    37	White		    47	White


	ESC[=nh	Set mode (see screen modes for n)
	ESC[=nl	Reset Mode (see screen modes for n)


Screen modes are similar to those found in the IBM BIOS:


			 0   text 40x25 Black & White
			 1   text 40x25 Color
			 2   text 80x25 Black & White
			 3   text 80x25 Color
			 4   320x200 4 bits/pixel
			 5   320x200 1 bit/pixel
			 6   640x200 1 bit/pixel
			 7   cursor wrap
			13   320x200 4 bits/pixel (EGA)
			14   640x200 4 bits/pixel (EGA)
			16   640x350 4 bits/pixel (EGA)
			
			
			*/
