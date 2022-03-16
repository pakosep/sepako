void *memmove (void* dest, const void* src, size_t size);
//Funkcja kopiuje SIZE bajtow z obiektu SRC do obiektu DEST. 
//Rozni sie od memcpy tym, ze obszar obiektow source oraz dest moga sie czesciowo pokrywac.
//Funkcja zwraca wskaznik na dest. 

char *strcpy (char* strTo, const char* strFrom);
//Funkcja kopiuje tekst z tablicy strFrom do tablicy strTo.
//Funkcja kopiuje znak po znaku od poczatku, az do konca tablicy lub znaku '\0', ktory tez kopiuje.
//Funkcja zwraca wskaznik na strTo.

char *strncpy (char* strTo, const char* strFrom, size_t size);
//Funkcja kopiuje co najwyzej size znakow z tekstu w tablicy strFrom do tablicy strTo. 
//Funkcja kopiuje znak po znaku od poczatku, az skopiuje size znakow lub napotka znak '\0',
//wtedy za reszte znakow do skopiowania wstawia '\0'.

void *memcpy (void* dest, const void* src, size_t size);
//Funkcja kopiuje size bajtow z obiektu source do obiektu dest.

int strlen (char *str);
//Funkcja oblicza dlugosc lancucha str. 
//Jej dzialanie polega na zliczaniu znakow az do napotkania 0 (znaku '\0'). 
//W przypadku lancuchow nie zakonczonych 0 jej dzialanie jest nieokreslone.

void * memset ( void * buffer, int c, size_t num );
//Wypelnia kolejne bajty w pamieci ustalona wartoscia.
//buffer - adres poczatkowy
//c - wpisywana wartosc (dla napisu - numer znaku)
//num - ile bajtów (uwaga! bajtów) zapisac

char *strcat (char* strTo, const char* strFrom);
//Funkcja dopisuje tekst z tablicy strFrom na koniec tekstu w tablicy strTo.
//Uwaga! Tablica strTo powinna byc dostatecznie duza, 
//aby pomiescic dodany tekst z strFrom, poniewaz moze dojsc do przepelnienia bufora. 

char *string;
  *string = strdup("Hello World");
//Funkcja strdup() zwraca wskaznik do nowego lancucha, ktory stanowi kopie lancucha s. 
//Pamiec dla nowego lancucha jest przydzielana za pomoca malloc() 
//i moze byc zwolniona za pomoca free().

char* strstr (const char* str1, const char* str2);
// Funkcja szuka pierwszego wyst¹pienia ³añcucha znakowego str2 w ³añcuchu str1


uint8_t gamma_correction[] = {
    0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,
    5,  5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  8,  8,  8,  8,  9,  9,
    9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 17, 17,
    18, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 26, 26, 27, 28, 29, 30,
    31, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 49,
    50, 51, 53, 54, 55, 57, 58, 60, 61, 63, 64, 66, 68, 69, 71, 73, 74, 76,
    78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98,101,103,105,107,110,112,115,
    117,120,122,125,128,131,133,136,139,142,145,148,151,154,157,161,164,167,
    171,174,177,181,185,188,192,196,200,203,207,211,216,220,224,228,232,237,
    241,246,250,255
  };