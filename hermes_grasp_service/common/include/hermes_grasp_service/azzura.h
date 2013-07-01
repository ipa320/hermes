 #ifndef I2H_AZZURA_VR2
 #define I2H_AZZURA_VR2


#include <iostream>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <stdlib.h>     /* atoi */
#include "azzura.h"

#define CYLLOW_C 0x60
#define CYLMED_C 0x61
#define CYLHIGH_C 0x62

#define TRILOW_C 0x64
#define TRI2LOW_C 0x71
#define TRIMED_C 0x65
#define TRIHIGH_C 0x66

#define BILOW_C 0x67
#define BI2LOW_C 0x69

#define LATHIGH_C 0x63

#define OPEN_ALL 0x4C
#define STOP_ALL 0x4C


#define FIC 0x42
#define FAC 0x46

#define CYLINDER   1
#define BIFINGER   2
#define BIFINGER2  3
#define TRIFINGER  4
#define TRIFINGER2 5
#define LATERAL    6
#define OPENALL    7
#define STOPALL    8
#define FIRSTCALIBRATION 9
#define FASTCALIBRATION 10
#define STRONGGRIP      11 
#define PEINETA         12



#endif /* I2H_AZZURA_VR2 */
