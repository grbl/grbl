#ifndef print_h
#define print_h

void printNewline(void);
void printString(const char *s);
void printPgmString(const char *s);
void printInteger(long n);
void printHex(unsigned long n);
void printOctal(unsigned long n);
void printBinary(unsigned long n);
void printIntegerInBase(unsigned long n, unsigned long base);
void printFloat(double n);

#endif