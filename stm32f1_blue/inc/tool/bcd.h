#ifndef __BCD_H
#define __BCD_H

inline uint8_t bcdToDec(uint8_t val) {  return ( (val/16*10) + (val%16) ); }
inline uint8_t decToBcd(uint8_t val) {  return ( (val/10*16) + (val%10) ); }

#endif 