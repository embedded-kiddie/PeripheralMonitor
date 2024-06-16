/*
 * Peripheral Monitor
 *
 * This program monitors Arduino UNO R4 Minima/WiFi peripheral registers
 * though the external serial I/F (Serial1) at D0(RX)/D1(TX).
 *
 * Copyright (c) 2024 Kingsman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#ifndef _PERIPHERAL_MONITOR_
#define _PERIPHERAL_MONITOR_

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "Serial.h"

#undef  DEBUG
#define DEBUG   0

/*----------------------------------------------------------------------
 * Peripheral types
 *----------------------------------------------------------------------*/
typedef enum {
  PERIPHERAL_PORTS, // PORT0 〜 PORT9 (default)
  PERIPHERAL_PORT,  // PORT0 〜 PORT9
  PERIPHERAL_PFS,   // PmnPFS (P000 〜 P915)
  PERIPHERAL_PINS,  // D0 〜 D19 (A0 〜 A5)
  PERIPHERAL_AGT,   // AGT0 〜 AGT1
  PERIPHERAL_CTSU,  // CTSU
} PeripheralType_t;

/*----------------------------------------------------------------------
 * Printf() buffer size
 *----------------------------------------------------------------------*/
#define PRINTF_BUF_SIZE   1024 // Max 986 at CTSU caption

/*----------------------------------------------------------------------
 * Port/Pin related definition
 *----------------------------------------------------------------------*/
// PmnPFS : P000 〜 P915
#define MAX_PFS_PORT_N    9
#define MAX_PFS_PIN_N     15

// A0 〜 A5, D0 〜 D19
#define MAX_ANALOG_PIN_N  5
#define MAX_DIGITAL_PIN_N 19
#define ANALOG_TO_DIGITAL 14

/*----------------------------------------------------------------------
 * Arduino digital pin No. to Port/Pin No. using digitalPinToBspPin()
 *----------------------------------------------------------------------*/
#define PmnToPfsPort(pmn)     ((pmn) / 100)   // for R_PFS->PORT[m].PIN[n]
#define PmnToPfsPin(pmn)      ((pmn) % 100)   // for R_PFS->PORT[m].PIN[n]
#define BspPinToPfsPort(bsp)  ((bsp) >> 8)    // Arduino pin No -> PORT[m]
#define BspPinToPfsPin(bsp)   ((bsp) & 0xFF)  // Arduino pin No ->  PIN[n]
#define digitalPinToPmn(p)    (BspPinToPfsPort(digitalPinToBspPin(p)) * 100 + BspPinToPfsPin(digitalPinToBspPin(p)))

/*----------------------------------------------------------------------
 * List of commands
 *----------------------------------------------------------------------*/
static const char * cmd_list[] = {
  "ports", "port", "pins", "p", "agt", "a", "d", "ctsu", "?", "help", NULL
};

/*----------------------------------------------------------------------
 * Peripheral declaration: PORT0 〜 PORT9
 *----------------------------------------------------------------------*/
static const R_PORT0_Type * p_port[] = {
  R_PORT0, R_PORT1, R_PORT2, R_PORT3, R_PORT4, R_PORT5, R_PORT6, R_PORT7, R_PORT8, R_PORT9
};

/*----------------------------------------------------------------------
 * Peripheral declaration: AGT0 〜 AGT1
 *----------------------------------------------------------------------*/
static const R_AGT0_Type * p_agt[] = {
  R_AGT0, R_AGT1
};

/*----------------------------------------------------------------------
 * ANSI escape code: PORT0 〜 PORT9
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_PORTS = "\e[2J\
\e[1;1HPORTS\
\
\e[2;13HPORT0 PORT1 PORT2 PORT3 PORT4\
\
\e[3;1HPCNTR1\
\e[4;2H- PDR[16]\
\e[5;2H- PODR[16]\
\
\e[7;1HPCNTR2\
\e[8;2H- PIDR[16]\
\e[9;2H- EIDR[16]\
\
\e[11;1HPCNTR3\
\e[12;2H- POSR[16]\
\e[13;2H- PORR[16]\
\
\e[15;1HPCNTR4\
\e[16;2H- EOSR[16]\
\e[17;2H- EORR[16]\
 ";

static const char * FMT_REGISTER_PORTS = "\
\e[?25l\
\e[4;%dH%%04X\
\e[5;%dH%%04X\
\e[8;%dH%%04X\
\e[9;%dH%%04X\
\e[12;%dH%%04X\
\e[13;%dH%%04X\
\e[16;%dH%%04X\
\e[17;%dH%%04X\
";

/*----------------------------------------------------------------------
 * ANSI escape code: PORT0 〜 PORT9
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_PORT = "\e[2J\
\e[1;1HPORT\
\
\e[3;22HFEDC BA98 7654 3210\
\e[3;1HPCNTR1\
\e[4;2H- PDR[16]\
\e[5;2H- PODR[16]\
\
\e[7;22HFEDC BA98 7654 3210\
\e[7;1HPCNTR2\
\e[8;2H- PIDR[16]\
\e[9;2H- EIDR[16]\
\
\e[11;22HFEDC BA98 7654 3210\
\e[11;1HPCNTR3\
\e[12;2H- POSR[16]\
\e[13;2H- PORR[16]\
\
\e[15;22HFEDC BA98 7654 3210\
\e[15;1HPCNTR4\
\e[16;2H- EOSR[16]\
\e[17;2H- EORR[16]\
 ";

static const char * FMT_REGISTER_PORT = "\e[?25l\
\e[1;5H%d\
\
\e[4;14H0x%04X  %s\
\e[5;14H0x%04X  %s\
\
\e[8;14H0x%04X  %s\
\e[9;14H0x%04X  %s\
\
\e[12;14H0x%04X  %s\
\e[13;14H0x%04X  %s\
\
\e[16;14H0x%04X  %s\
\e[17;14H0x%04X  %s\
";

/*----------------------------------------------------------------------
 * ANSI escape code: PmnPFS (P000 〜 P915)
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_PFS = "\e[2J\
\e[1;1HP\
\
\e[3;1HPmnPFS\
\e[4;2H- PODR[1]\
\e[5;2H- PIDR[1]\
\e[6;2H- PDR[1]\
\e[7;2H- PCR[1]\
\e[8;2H- PIM[1]\
\e[9;2H- NCODR[1]\
\e[10;2H- DSCR[2]\
\e[11;2H- EOFR[2]\
\e[12;2H- ISEL[1]\
\e[13;2H- ASEL[1]\
\e[14;2H- PMR[1]\
\e[15;2H- PSEL[5]\
 ";

static const char * FMT_REGISTER_PFS = "\e[?25l\
\
\e[1;2H%1d%02d\
\e[4;14H%d\
\e[5;14H%d\
\e[6;14H%d\
\e[7;14H%d\
\e[8;14H%d\
\e[9;14H%d\
\e[10;14H%d\
\e[11;14H%d\
\e[12;14H%d\
\e[13;14H%d\
\e[14;14H%d\
\e[15;14H%d (%s)\
";

/*----------------------------------------------------------------------
 * ANSI escape code: A0 〜 A5, D0 〜 D19
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_PINS = "\e[2J\
\e[1;1HPINS\
\
\e[3;13HD0 D1 D2 D3 D4 D5 D6 D7 D8 D9 Da Db Dc Dd A0 A1 A2 A3 A4 A5\
\
\e[3;1HPmnPFS\
\e[4;2H- PODR[1]\
\e[5;2H- PIDR[1]\
\e[6;2H- PDR[1]\
\e[7;2H- PCR[1]\
\e[8;2H- PIM[1]\
\e[9;2H- NCODR[1]\
\e[10;2H- DSCR[2]\
\e[11;2H- EOFR[2]\
\e[12;2H- ISEL[1]\
\e[13;2H- ASEL[1]\
\e[14;2H- PMR[1]\
\e[15;2H- PSEL[5]\
 ";

static const char * FMT_REGISTER_PINS = "\e[?25l\
\e[4;%dH%%d\
\e[5;%dH%%d\
\e[6;%dH%%d\
\e[7;%dH%%d\
\e[8;%dH%%d\
\e[9;%dH%%d\
\e[10;%dH%%d\
\e[11;%dH%%d\
\e[12;%dH%%d\
\e[13;%dH%%d\
\e[14;%dH%%d\
\e[15;%dH%%d\
";

/*----------------------------------------------------------------------
 * ANSI escape code: AGT0 〜 AGT1
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_AGT = "\e[2J\
\e[1;1HAGT\
\
\e[3;1HAGT\
\e[4;2H- AGT[16]\
\
\e[6;1HAGTCMA\
\e[7;2H- AGTCMA[16]\
\
\e[9;1HAGTCMB\
\e[10;2H- AGTCMB[16]\
\
\e[12;1HAGTCR\
\e[13;2H- TSTART[1]\
\e[14;2H- TCSTF[1]\
\e[15;2H- TSTOP[1]\
\e[16;2H- TEDGF[1]\
\e[17;2H- TUNDF[1]\
\e[18;2H- TCMAF[1]\
\e[19;2H- TCMBF[1]\
\
\e[3;25HAGTMR1\
\e[4;26H- TMOD[3]\
\e[5;26H- TEDGPL[1]\
\e[6;26H- TCK[3]\
\
\e[8;25HAGTMR2\
\e[9;26H- CKS[3]\
\e[10;26H- LPM[1]\
\
\e[12;25HAGTIOC\
\e[13;26H- TEDGSEL[1]\
\e[14;26H- TOE[1]\
\e[15;26H- TIPF[2]\
\e[16;26H- TIOGT[2]\
\
\e[18;25HAGTISR\
\e[19;26H- EEPS[1]\
\
\e[3;45HAGTCMSR\
\e[4;46H- TCMEA[1]\
\e[5;46H- TOEA[1]\
\e[6;46H- TOPOLA[1]\
\e[7;46H- TCMEB[1]\
\e[8;46H- TOEB[1]\
\e[9;46H- TOPOLB[1]\
\
\e[11;45HAGTIOSEL\
\e[12;46H- SEL[2]\
\e[13;46H- TIES[1]\
 ";

static const char * FMT_REGISTER_AGT = "\e[?25l\
\e[1;4H%d\
\e[4;15H%5d\
\e[7;15H%5d\
\e[10;15H%5d\
\
\e[13;15H%d\
\e[14;15H%d\
\e[15;15H%d\
\e[16;15H%d\
\e[17;15H%d\
\e[18;15H%d\
\e[19;15H%d\
\
\e[4;40H%d\
\e[5;40H%d\
\e[6;40H%d\
\
\e[9;40H%d\
\e[10;40H%d\
\
\e[13;40H%d\
\e[14;40H%d\
\e[15;40H%d\
\e[16;40H%d\
\
\e[19;40H%d\
\
\e[4;60H%d\
\e[5;60H%d\
\e[6;60H%d\
\e[7;60H%d\
\e[8;60H%d\
\e[9;60H%d\
\
\e[12;60H%d\
\e[13;60H%d\
";

/*----------------------------------------------------------------------
 * Capacitive Touch Sensing Unit escape code: CTSU
 *----------------------------------------------------------------------*/
static const char * FMT_CAPTION_CTSU = "\e[2J\
\e[1;1HCTSU\
\e[3;1HCR0\
\e[4;2H- STRT[1]\
\e[5;2H- CAP[1]\
\e[6;2H- SNZ[1]\
\e[7;2H- INIT[1]\
\
\e[9;1HCR1\
\e[10;2H- PON[1]\
\e[11;2H- CSW[1]\
\e[12;2H- ATUNE0[1]\
\e[13;2H- ATUNE1[1]\
\e[14;2H- CLK[2]\
\e[15;2H- MD[2]\
\
\e[17;1HSDPRS\
\e[18;2H- PRRATIO[4]\
\e[19;2H- PRMODE[2]\
\e[20;2H- SOFF[1]\
\
\e[22;1HSST\
\e[23;2H- SST[8]\
\
\e[3;21HMCH0/1\
\e[4;22H- MCH0[6]\
\e[5;22H- MCH1[6]\
\
\e[7;21HCHAC0/1/2/3/4\
\e[8;22H- CHAC0[8]\
\e[9;22H- CHAC1[8]\
\e[10;22H- CHAC2[8]\
\e[11;22H- CHAC3[8]\
\e[12;22H- CHAC4[4]\
\
\e[14;21HCHTRC0/1/2/3/4\
\e[15;22H- CHTRC0[8]\
\e[16;22H- CHTRC1[8]\
\e[17;22H- CHTRC2[8]\
\e[18;22H- CHTRC3[8]\
\e[19;22H- CHTRC4[8]\
\
\e[21;21HDCLKC\
\e[22;22H- SSMOD[2]\
\e[23;22H- SSCNT[2]\
\
\e[3;42HST\
\e[4;43H- STC[3]\
\e[5;43H- DTSR[1]\
\e[6;43H- SOVF[1]\
\e[7;43H- ROVF[1]\
\e[8;43H- PS[1]\
\
\e[10;42HSSC\
\e[11;43H- SSDIV[4]\
\
\e[13;42HSO0\
\e[14;43H- SO[10]\
\e[15;43H- SNUM[6]\
\
\e[17;42HSO1\
\e[18;43H- RICOA[8]\
\e[19;43H- SDPA[5]\
\e[20;43H- ICOG[2]\
\
\e[3;61HERRS\
\e[4;62H- SPMD[2]\
\e[5;62H- TSOD[1]\
\e[6;62H- DRV[1]\
\e[7;62H- CLKSEL1[1]\
\e[8;62H- TSOC[1]\
\e[9;62H- ICOMP[1]\
\
\e[11;61HSC/RC\
\e[12;62H- SC[16]    n/a\
\e[13;62H- RC[16]    n/a\
 ";

static const char * FMT_REGISTER_CTSU = "\e[?25l\
\e[4;17H%d\
\e[5;17H%d\
\e[6;17H%d\
\e[7;17H%d\
\
\e[10;17H%d\
\e[11;17H%d\
\e[12;17H%d\
\e[13;17H%d\
\e[14;15H0x%X\
\e[15;15H0x%X\
\
\e[18;15H0x%X\
\e[19;15H0x%X\
\e[20;17H%d\
\
\e[23;14H0x%02X\
\
\e[4;35H0x%02X\
\e[5;35H0x%02X\
\
\e[8;35H0x%02X\
\e[9;35H0x%02X\
\e[10;35H0x%02X\
\e[11;35H0x%02X\
\e[12;35H0x%02X\
\
\e[15;35H0x%02X\
\e[16;35H0x%02X\
\e[17;35H0x%02X\
\e[18;35H0x%02X\
\e[19;35H0x%02X\
\
\e[22;36H0x%X\
\e[23;36H0x%X\
\
\e[4;55H0x%X\
\e[5;57H%d\
\e[6;57H%d\
\e[7;57H%d\
\e[8;57H%d\
\
\e[11;55H0x%X\
\
\e[14;54H%4d\
\e[15;54H0x%02X\
\
\e[18;54H0x%02X\
\e[19;54H0x%02X\
\e[20;55H0x%X\
\
\e[4;74H0x%X\
\e[5;76H%d\
\e[6;76H%d\
\e[7;76H%d\
\e[8;76H%d\
\e[9;76H%d\
";

/*--------------------------------------------------------------------------------------------
 * Class definition
 *-------------------------------------------------------------------------------------------*/
class PeripheralMonitor {
  /*----------------------------------------------------------------------------------*/
  private:
  /*----------------------------------------------------------------------------------*/

  PeripheralType_t peripheral = PERIPHERAL_PORTS;
  int num_agt = 0, num_port = 0, num_pfs = 0, num_pin = 0;

  /*------------------------------------------------------------
   * Output formatted strings to serial I/F
   *------------------------------------------------------------*/
  int printf(const char* fmt, ...) {
    int len = 0;
    char buf[PRINTF_BUF_SIZE];
#if DEBUG
    uint32_t start = micros(), end, lap1, lap2;
#endif
  
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    len = vsnprintf(buf, PRINTF_BUF_SIZE, fmt, arg_ptr);
    va_end(arg_ptr);

#if DEBUG
    lap1 = (lap2 = micros()) - start;
#endif

    // output to the serial console through the 'Serial'
    len = Serial1.write((uint8_t*)buf, (size_t)len); 

#if DEBUG && 0
    Serial.println("availableForWrite = " + String(Serial1.availableForWrite()));
#endif

#if DEBUG
    lap2 = (end = micros()) - lap2;
    Serial.println(
      "len: "        + String(len ) + 
      ", vsprintf: " + String(lap1) + 
      " + Serial1: " + String(lap2) + 
      " = "          + String(end - start)
    );
#endif

    return len;
  }

  /*------------------------------------------------------------
   * Convert decimal number to binary number
   *------------------------------------------------------------*/
  char* dec2bin(char* bin, int bit, uint16_t dec) {
    *bin = 0;
    for (--bit; bit >= 0; --bit) {
      strcat(bin, dec & (1 << bit) ? "1" : "0");
      if (bit && bit % 4 == 0) {
        strcat(bin, " ");
      }
    }
    return bin;
  }

  /*------------------------------------------------------------
   *　Show heartbeat indicator
   *------------------------------------------------------------*/
  __attribute__((always_inline)) void heartbeat(void) {
    static uint8_t n;
    printf("\e[1;7H%c ", "-\\|/"[n++ % 4]);
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: PORT0 〜 PORT9
   *------------------------------------------------------------*/
  void setup_ports(int arg) {
    peripheral = PERIPHERAL_PORTS;
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: PORT0 〜 PORT9
   *------------------------------------------------------------*/
  void setup_port(int port) {
    if (0 <= port && port <= MAX_PFS_PORT_N) {
      peripheral = PERIPHERAL_PORT;
      num_port = port;
      return;
    }
    Serial.println("PORT: out of range.");
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: PmnPFS (P000 〜 P915)
   *------------------------------------------------------------*/
  void setup_pfs(int m, int n) {
    if (0 <= m && m <= MAX_PFS_PORT_N && 0 <= n && n <= MAX_PFS_PIN_N) {
      peripheral = PERIPHERAL_PFS;
      num_pfs = m;
      num_pin = n;
      return;
    }
    Serial.println("PFS: out of range.");
  }

  void setup_pfs(int pmn) {
    setup_pfs(PmnToPfsPort(pmn), PmnToPfsPin(pmn));
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: A0 〜 A5, D0 〜 D19
   *------------------------------------------------------------*/
  void setup_pins(int arg) {
    peripheral = PERIPHERAL_PINS;
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: D0 〜 D19
   *------------------------------------------------------------*/
  void setup_digital(int pin) {
    if (0 <= pin && pin <= MAX_DIGITAL_PIN_N) {
      pin = digitalPinToBspPin(pin);
      setup_pfs(BspPinToPfsPort(pin), BspPinToPfsPin(pin));
      return;
    }
    Serial.println("Digital: out of range.");
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: A0 〜 A5
   *------------------------------------------------------------*/
  void setup_analog(int pin) {
    if (0 <= pin && pin <= MAX_ANALOG_PIN_N) {
      setup_digital(pin + ANALOG_TO_DIGITAL);
      return;
    }
    Serial.println("Analog: out of range.");
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: AGT0 〜 AGT1
   *------------------------------------------------------------*/
  void setup_agt(int num) {
    if (0 <= num && num < sizeof(p_agt) / sizeof(p_agt[0])) {
      peripheral = PERIPHERAL_AGT;
      num_agt = num;
      return;
    }
    Serial.println("AGT: out of range.");
  }

  /*------------------------------------------------------------
   * Setup peripheral registers: CTSU
   *------------------------------------------------------------*/
  void setup_ctsu(int arg) {
    peripheral = PERIPHERAL_CTSU;
    return;
  }

  void show_help(int i) {
    String cmd = "";
    for (i = 0; cmd_list[i] != NULL; i++) {
      cmd += String(cmd_list[i]) + (cmd_list[i+1] != NULL ? ", " : "");
    }
    Serial.println("Possible commands: " + cmd);
  }

  /*------------------------------------------------------------
   * Show peripheral registers: A0 〜 A5, D0 〜 D19
   *------------------------------------------------------------*/
  void show_ports(void) {
    char fmt[128];
    int i, c;
    const R_PORT0_Type * p;

    // PORT0  PORT1  PORT2  PORT3  PORT4
    for (i = 0; i <= 4; i++) {
      c = i * 6 + 14;
      p = p_port[i];
      sprintf(fmt, FMT_REGISTER_PORTS, c, c, c, c, c, c, c, c);
      printf(fmt, p->PDR, p->PODR, p->PIDR, p->EIDR, p->POSR, p->PORR, p->EOSR, p->EORR);
    }
  }

  /*------------------------------------------------------------
   * Show peripheral registers: PORT0 〜 PORT9
   *------------------------------------------------------------*/
  void show_port(int port) {
    if (0 <= port && port <= MAX_PFS_PORT_N) {
      char buf[8][64];
      /*volatile*/ const R_PORT0_Type P_PORT = *p_port[port];
      printf(FMT_REGISTER_PORT,
        /* PORT No            */ port,

        /* Port Control Register 1 */
        /* uint32_t PDR  : 16 */ P_PORT.PDR , dec2bin(buf[0], 16, P_PORT.PDR ), // Pmn Direction
        /* uint32_t PODR : 16 */ P_PORT.PODR, dec2bin(buf[1], 16, P_PORT.PODR), // Pmn Output Data

        /* Port Control Register 2 */
        /* uint32_t PIDR : 16 */ P_PORT.PIDR, dec2bin(buf[2], 16, P_PORT.PIDR), // Pmn Input Data
        /* uint32_t EIDR : 16 */ P_PORT.EIDR, dec2bin(buf[3], 16, P_PORT.EIDR), // Pmn Event Input Data

        /* Port Control Register 3 */
        /* uint32_t POSR : 16 */ P_PORT.POSR, dec2bin(buf[4], 16, P_PORT.POSR), // Pmn Output Set
        /* uint32_t PORR : 16 */ P_PORT.PORR, dec2bin(buf[5], 16, P_PORT.PORR), // Pmn Output Reset

        /* Port Control Register 4 */
        /* uint32_t EOSR : 16 */ P_PORT.EOSR, dec2bin(buf[6], 16, P_PORT.EOSR), // Pmn Event Output Set
        /* uint32_t EORR : 16 */ P_PORT.EORR, dec2bin(buf[7], 16, P_PORT.EORR)  // Pmn Event Output Reset
      );
    }
  }

  /*------------------------------------------------------------
   * Show peripheral registers: PmnPFS (P000 〜 P915)
   *------------------------------------------------------------*/
  void show_pfs(int m, int n) {
    if (0 <= m && m <= MAX_PFS_PORT_N && 0 <= n && n <= MAX_PFS_PIN_N) {
      char buf[8];
      volatile const R_PFS_PORT_PIN_Type * P_PFS = &R_PFS->PORT[m].PIN[n];
      printf(FMT_REGISTER_PFS,
        /* PORT No and PIN No */ m, n,
        /* Port mn Pin Function Select Register */
        /* uint32_t PODR  : 1 */ P_PFS->PmnPFS_b.PODR,  // Port Output Data
        /* uint32_t PIDR  : 1 */ P_PFS->PmnPFS_b.PIDR,  // Port Input Data
        /* uint32_t PDR   : 1 */ P_PFS->PmnPFS_b.PDR,   // Port Direction
        /* uint32_t PCR   : 1 */ P_PFS->PmnPFS_b.PCR,   // Pull-up Control
        /* uint32_t PIM   : 1 */ P_PFS->PmnPFS_b.PIM,   // Port Input Mode Control
        /* uint32_t NCODR : 1 */ P_PFS->PmnPFS_b.NCODR, // N-Channel Open Drain Control
        /* uint32_t DSCR  : 2 */ P_PFS->PmnPFS_b.DSCR,  // Drive Strength Control Register
        /* uint32_t EOFR  : 2 */ P_PFS->PmnPFS_b.EOFR,  // Event on Falling/Rising 
        /* uint32_t ISEL  : 1 */ P_PFS->PmnPFS_b.ISEL,  // IRQ input enable
        /* uint32_t ASEL  : 1 */ P_PFS->PmnPFS_b.ASEL,  // Analog Input enable
        /* uint32_t PMR   : 1 */ P_PFS->PmnPFS_b.PMR,   // Port Mode Control
        /* uint32_t PSEL  : 5 */ P_PFS->PmnPFS_b.PSEL,  // Port Function Select
        dec2bin(buf, 5, P_PFS->PmnPFS_b.PSEL)
      );
    }
  }

  /*------------------------------------------------------------
   * Show peripheral registers: A0 〜 A5, D0 〜 D19
   *------------------------------------------------------------*/
  void show_pins(void) {
    char fmt[128];
    volatile const R_PFS_PORT_PIN_Type *p;

    // D0 〜 D19
    for (int i = 0; i <= MAX_DIGITAL_PIN_N ; i++) {
      int c = digitalPinToBspPin(i);
      p = &R_PFS->PORT[BspPinToPfsPort(c)].PIN[BspPinToPfsPin(c)];
      c = i * 3 + 14;
      sprintf(fmt, FMT_REGISTER_PINS, c, c, c, c, c, c, c, c, c, c, c, c);
      printf(fmt, p->PmnPFS_b.PODR, p->PmnPFS_b.PIDR, p->PmnPFS_b.PDR, p->PmnPFS_b.PCR, p->PmnPFS_b.PIM, p->PmnPFS_b.NCODR, p->PmnPFS_b.DSCR, p->PmnPFS_b.EOFR, p->PmnPFS_b.ISEL, p->PmnPFS_b.ASEL, p->PmnPFS_b.PMR, p->PmnPFS_b.PSEL);
    }
  }

  /*------------------------------------------------------------
   * Show peripheral registers: AGT0 〜 ARG1
   *------------------------------------------------------------*/
  void show_agt(int num) {
    if (0 <= num && num < sizeof(p_agt) / sizeof(p_agt[0])) {
      /*volatile*/ const R_AGT0_Type P_AGT = *p_agt[num];
      printf(FMT_REGISTER_AGT,
        /* AGT No             */ num,
        /* uint16_t AGT       */ P_AGT.AGT,              // 16bit counter and reload register
        /* uint16_t AGTCMA    */ P_AGT.AGTCMA,           // AGT Compare Match A Register
        /* uint16_t AGTCMB    */ P_AGT.AGTCMB,           // AGT Compare Match B Register

        /* AGT Control Register */
        /* uint8_t TSTART : 1 */ P_AGT.AGTCR_b.TSTART,   // AGT count start
        /* uint8_t TCSTF  : 1 */ P_AGT.AGTCR_b.TCSTF,    // AGT count status flag
        /* uint8_t TSTOP  : 1 */ P_AGT.AGTCR_b.TSTOP,    // AGT count forced stop
        /* uint8_t TEDGF  : 1 */ P_AGT.AGTCR_b.TEDGF,    // Active edge judgment flag
        /* uint8_t TUNDF  : 1 */ P_AGT.AGTCR_b.TUNDF,    // Underflow flag
        /* uint8_t TCMAF  : 1 */ P_AGT.AGTCR_b.TCMAF,    // Compare match A flag
        /* uint8_t TCMBF  : 1 */ P_AGT.AGTCR_b.TCMBF,    // Compare match B flag

        /* AGT Mode Register 1 */
        /* uint8_t TMOD   : 3 */ P_AGT.AGTMR1_b.TMOD,    // Operating mode
        /* uint8_t TEDGPL : 1 */ P_AGT.AGTMR1_b.TEDGPL,  // Edge polarity
        /* uint8_t TCK    : 3 */ P_AGT.AGTMR1_b.TCK,     // Count source

        /* AGT Mode Register 2 */
        /* uint8_t CKS    : 3 */ P_AGT.AGTMR2_b.CKS,     // AGTLCLK/AGTSCLK count source clock frequency division ratio
        /* uint8_t LPM    : 1 */ P_AGT.AGTMR2_b.LPM,     // Low Power Mode

        /* AGT I/O Control Register */
        /* uint8_t TEDGSEL: 1 */ P_AGT.AGTIOC_b.TEDGSEL, // I/O polarity switchFunction varies depending on the operating mode
        /* uint8_t TOE    : 1 */ P_AGT.AGTIOC_b.TOE,     // AGTOn output enable
        /* uint8_t TIPF   : 2 */ P_AGT.AGTIOC_b.TIPF,    // Input filter
        /* uint8_t TIOGT  : 2 */ P_AGT.AGTIOC_b.TIOGT,   // Count control

        /* AGT Event Pin Select Register */
        /* uint8_t EEPS   : 1 */ P_AGT.AGTISR_b.EEPS,    // AGTEE polarty selection

        /* AGT Compare Match Function Select Register */
        /* uint8_t TCMEA  : 1 */ P_AGT.AGTCMSR_b.TCMEA,  // Compare match A register enable
        /* uint8_t TOEA   : 1 */ P_AGT.AGTCMSR_b.TOEA,   // AGTOA output enable
        /* uint8_t TOPOLA : 1 */ P_AGT.AGTCMSR_b.TOPOLA, // AGTOA polarity select
        /* uint8_t TCMEB  : 1 */ P_AGT.AGTCMSR_b.TCMEB,  // Compare match B register enable
        /* uint8_t TOEB   : 1 */ P_AGT.AGTCMSR_b.TOEB,   // AGTOB output enable
        /* uint8_t TOPOLB : 1 */ P_AGT.AGTCMSR_b.TOEB,   // AGTOB polarity select

        /* AGT Pin Select Register */
        /* uint8_t SEL    : 2 */ P_AGT.AGTIOSEL_b.SEL,   // AGTIO pin select
        /* uint8_t TIES   : 1 */ P_AGT.AGTIOSEL_b.TIES   // AGTIO input enable
      );
    }
  }

  /*------------------------------------------------------------
   * Show peripheral registers: CTSU
   *------------------------------------------------------------*/
  void show_ctsu(int arg) {
//  Serial.println("CTSU: " + String(strlen(FMT_CAPTION_CTSU)));
    printf(FMT_REGISTER_CTSU,
      /* CTSUCR1: CTSU Control Register 0 */
      R_CTSU->CTSUCR0_b.CTSUSTRT,       // Measurement Operation Start
      R_CTSU->CTSUCR0_b.CTSUCAP,        // Measurement Operation Start Trigger Select
      R_CTSU->CTSUCR0_b.CTSUSNZ,        // Wait State Power-Saving Enable
//    R_CTSU->CTSUCR0_b.CTSUIOC,        // Transmit Pin Control
      R_CTSU->CTSUCR0_b.CTSUINIT,       // Control Block Initialization 

      /* CTSUCR1: CTSU Control Register 1 */
      R_CTSU->CTSUCR1_b.CTSUPON,        // Power Supply Enable
      R_CTSU->CTSUCR1_b.CTSUCSW,        // LPF Capacitance Charging Control
      R_CTSU->CTSUCR1_b.CTSUATUNE0,     // Power Supply Operating Mode Setting
      R_CTSU->CTSUCR1_b.CTSUATUNE1,     // Power Supply Capacity Adjustment
      R_CTSU->CTSUCR1_b.CTSUCLK,        // Operating Clock Select
      R_CTSU->CTSUCR1_b.CTSUMD,         // Measurement Mode Select

      /* CTSUSDPRS: CTSU Synchronous Noise Reduction Setting Register */
      R_CTSU->CTSUSDPRS_b.CTSUPRRATIO,  // Measurement Time and Pulse Count Adjustment
      R_CTSU->CTSUSDPRS_b.CTSUPRMODE,   // Base Period and Pulse Count Setting
      R_CTSU->CTSUSDPRS_b.CTSUSOFF,     // High-Pass Noise Reduction Function Off Setting

      /* CTSUSST: CTSU Sensor Stabilization Wait Control Register */
      R_CTSU->CTSUSST,                  // Sensor Stabilization Wait ControlNOTE: The value

      /* CTSUMCH0/1: CTSU Measurement Channel Register 0/1 */
      R_CTSU->CTSUMCH0,                 // Measurement Channel 0
      R_CTSU->CTSUMCH1,                 // Measurement Channel 1

      /* CTSUCHAC0/1/2/3/4: CTSU Channel Enable Control Register */
      R_CTSU->CTSUCHAC[0],              // Channel Enable Control
      R_CTSU->CTSUCHAC[1],              // Channel Enable Control
      R_CTSU->CTSUCHAC[2],              // Channel Enable Control
      R_CTSU->CTSUCHAC[3],              // Channel Enable Control
      R_CTSU->CTSUCHAC[4],              // Channel Enable Control

      /* CTSUCHTRC0/1/2/3/4: CTSU Channel Transmit/Receive Control Register */
      R_CTSU->CTSUCHTRC[0],             // Channel Transmit/Receive Control
      R_CTSU->CTSUCHTRC[1],             // Channel Transmit/Receive Control
      R_CTSU->CTSUCHTRC[2],             // Channel Transmit/Receive Control
      R_CTSU->CTSUCHTRC[3],             // Channel Transmit/Receive Control
      R_CTSU->CTSUCHTRC[4],             // Channel Transmit/Receive Control

      /* CTSUDCLKC: CTSU High-Pass Noise Reduction Control Register */
      R_CTSU->CTSUDCLKC_b.CTSUSSMOD,    // Diffusion Clock Mode Select
      R_CTSU->CTSUDCLKC_b.CTSUSSCNT,    // Diffusion Clock Mode Control

      /* CTSUST: CTSU Status Register */
      R_CTSU->CTSUST_b.CTSUSTC,         // Measurement Status Counter
      R_CTSU->CTSUST_b.CTSUDTSR,        // Data Transfer Status Flag
      R_CTSU->CTSUST_b.CTSUSOVF,        // Sensor Counter Overflow Flag
      R_CTSU->CTSUST_b.CTSUROVF,        // Reference Counter Overflow Flag
      R_CTSU->CTSUST_b.CTSUPS,          // Mutual Capacitance Status Flag

      /* CTSUSSC: CTSU High-Pass Noise Reduction Spectrum Diffusion ontrol Register */
      R_CTSU->CTSUSSC_b.CTSUSSDIV,      // Spectrum Diffusion Frequency Division Setting

      /* CTSUSO0: CTSU Sensor Offset Register 0 */
      R_CTSU->CTSUSO0_b.CTSUSO,         // Sensor Offset Adjustment
      R_CTSU->CTSUSO0_b.CTSUSNUM,       // Measurement Count Setting

      /* CTSUSO1: CTSU Sensor Offset Register 1 */
      R_CTSU->CTSUSO1_b.CTSURICOA,      // Reference ICO Current Adjustment
      R_CTSU->CTSUSO1_b.CTSUSDPA,       // SettingOperating clock divided by (CTSUSDPA + 1) x 2
      R_CTSU->CTSUSO1_b.CTSUICOG,       // ICO Gain Adjustment

      /* CTSUERRS: CTSU Error Status Register */
      R_CTSU->CTSUERRS_b.CTSUSPMD,      // Calibration Mode
      R_CTSU->CTSUERRS_b.CTSUTSOD,      // TS Pin Fixed Output
      R_CTSU->CTSUERRS_b.CTSUDRV,       // Calibration Setting 1
      R_CTSU->CTSUERRS_b.CTSUCLKSEL1,   // Calibration Setting 3
      R_CTSU->CTSUERRS_b.CTSUTSOC,      // Calibration Setting 2
      R_CTSU->CTSUERRS_b.CTSUICOMP      // TSCAP Voltage Error Monitor
    );
  }

  /*----------------------------------------------------------------------------------*/
  public:
  /*----------------------------------------------------------------------------------*/

  /*----------------------------------------------------------------------------------*/
  void setup_register(PeripheralType_t type = PERIPHERAL_PORTS, int arg = 0) {
  /*----------------------------------------------------------------------------------*/
    switch (peripheral = type) {
      case PERIPHERAL_PORTS: setup_ports(arg); break;
      case PERIPHERAL_PORT:  setup_port(arg); break;
      case PERIPHERAL_PFS:   setup_pfs(arg); break;
      case PERIPHERAL_PINS:  setup_pins(arg); break;
      case PERIPHERAL_AGT:   setup_agt(arg); break;
      case PERIPHERAL_CTSU:  setup_ctsu(arg); break;
      default: break;
    }
  }

  /*----------------------------------------------------------------------------------*/
  void show_caption(void) {
  /*----------------------------------------------------------------------------------*/
    switch (peripheral) {
      case PERIPHERAL_PORTS:  printf(FMT_CAPTION_PORTS);  break;  // PORT0 〜 PORT9
      case PERIPHERAL_PORT:   printf(FMT_CAPTION_PORT);   break;  // PORT0 〜 PORT9
      case PERIPHERAL_PFS:    printf(FMT_CAPTION_PFS);    break;  // PmnPFS (P000 〜 P915)
      case PERIPHERAL_PINS:   printf(FMT_CAPTION_PINS);   break;  // A0 〜 A5, D0 〜 D19
      case PERIPHERAL_AGT:    printf(FMT_CAPTION_AGT);    break;  // AGT0 〜 ARG1
      case PERIPHERAL_CTSU:   printf(FMT_CAPTION_CTSU);   break;  // CTSU
    }
  }

  /*----------------------------------------------------------------------------------*/
  void show_register(void) {
  /*----------------------------------------------------------------------------------*/
    heartbeat();
    switch (peripheral) {
      case PERIPHERAL_PORTS:  show_ports();               break;  // PORT0 〜 PORT9
      case PERIPHERAL_PORT:   show_port(num_port);        break;  // PORT0 〜 PORT9
      case PERIPHERAL_PFS:    show_pfs(num_pfs, num_pin); break;  // PmnPFS (P000 〜 P915)
      case PERIPHERAL_PINS:   show_pins();                break;  // A0 〜 A5, D0 〜 D19
      case PERIPHERAL_AGT:    show_agt(num_agt);          break;  // AGT0 〜 ARG1
      case PERIPHERAL_CTSU:   show_ctsu(num_agt);         break;  // ctsu
    }
  }

  /*----------------------------------------------------------------------------------*/
  void scan_command(void) {
  /*----------------------------------------------------------------------------------*/
    if (Serial.available()) {
      String cmd = Serial.readString();
      cmd.trim();
      cmd.toLowerCase();

      // "ports", "port", "pins", "p", "agt", "a", "d", "ctsu", "?", "help", NULL
      int num = -1, arg = -1;
      for (int i = 0; cmd_list[i] != NULL; i++) {
        if (cmd.startsWith(cmd_list[i])) {
          num = i;
          arg = cmd.substring(strlen(cmd_list[i])).toInt();
          break;
        }
      }

      switch (num) {
        case 0: setup_ports(arg);   break;  // PORT0 〜 PORT9
        case 1: setup_port(arg);    break;  // PORT0 〜 PORT9
        case 2: setup_pins(arg);    break;  // A0 〜 A5, D0 〜 D19
        case 3: setup_pfs(arg);     break;  // PmnPFS (P000 〜 P915)
        case 4: setup_agt(arg);     break;  // AGT0 〜 ARG1
        case 5: setup_analog(arg);  break;  // A0 〜 A5
        case 6: setup_digital(arg); break;  // D0 〜 D19
        case 7: setup_ctsu(arg);    break;  // CTSU
        case 8:
        case 9: show_help(arg); break;
        default: break;
      }
      show_caption();
    }
  }

  /*----------------------------------------------------------------------------------*/
  bool begin(int baud_rate, PeripheralType_t type = PERIPHERAL_PORTS, int arg = 0) {
  /*----------------------------------------------------------------------------------*/
    Serial1.begin(baud_rate, SERIAL_8N1);
    if (Serial1) {
      setup_register(type, arg);
      show_caption();
      return true;
    }
    return false;
  }
};

#endif  // _PERIPHERAL_MONITOR_
