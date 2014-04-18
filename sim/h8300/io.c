/*
 H8 simulator Internal Peripheral Support
*/

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <string.h>
#define _XOPEN_SOURCE
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "sim-main.h"
#undef CSIZE
#include <termios.h>

#define MAX_SCI_CH 3

#define SMR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+0])
#define BRR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+1])
#define SCR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+2])
#define TDR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+3])
#define SSR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+4])
#define RDR(ch) (STATE_CPU(sd, 0)->eightbit[sci_base[ch]+5])

#define TCR8(ch)   (STATE_CPU(sd, 0)->eightbit[timer8_base[ch] + 0])
#define TCSR8(ch)  (STATE_CPU(sd, 0)->eightbit[timer8_base[ch] + 2])
#define TCORA8(ch) (STATE_CPU(sd, 0)->eightbit[timer8_base[ch] + 4])
#define TCORB8(ch) (STATE_CPU(sd, 0)->eightbit[timer8_base[ch] + 6])
#define TCNT8(ch)  (STATE_CPU(sd, 0)->eightbit[timer8_base[ch] + 8])

#define TSTR16 (STATE_CPU(sd, 0)->eightbit[0x60])
#define TISRA16  (STATE_CPU(sd, 0)->eightbit[0x64])
#define TISRB16  (STATE_CPU(sd, 0)->eightbit[0x65])
#define TISRC16  (STATE_CPU(sd, 0)->eightbit[0x66])
#define TCR16(ch) (STATE_CPU(sd, 0)->eightbit[0x68 + (ch) * 8])
#define TCNT16H(ch) (STATE_CPU(sd, 0)->eightbit[0x6a + (ch) * 8])
#define TCNT16L(ch) (STATE_CPU(sd, 0)->eightbit[0x6b + (ch) * 8])
#define GRA16H(ch) (STATE_CPU(sd, 0)->eightbit[0x6c + (ch) * 8])
#define GRA16L(ch) (STATE_CPU(sd, 0)->eightbit[0x6d + (ch) * 8])
#define GRB16H(ch) (STATE_CPU(sd, 0)->eightbit[0x6e + (ch) * 8])
#define GRB16L(ch) (STATE_CPU(sd, 0)->eightbit[0x6f + (ch) * 8])

#define TPU_CH 6
#define TPU_TSTR (STATE_CPU(sd, 0)->eightbit[0xc0])
#define TPU_TCR(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 0])
#define TPU_TSR(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 5])
#define TPU_TCNTH(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 6])
#define TPU_TCNTL(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 7])
#define TPU_GRAH(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 8])
#define TPU_GRAL(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 9])
#define TPU_GRBH(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 10])
#define TPU_GRBL(ch) (STATE_CPU(sd, 0)->memory[tpubase[ch] + 11])

#define IPRA_H8300H 0xfee018
#define IPRB_H8300H 0xfee019

#define IPRA_H8300S 0xfffe00

struct int_list_t {
  int vector;
  unsigned int  isr_adr;
  unsigned char isr_mask;
  unsigned int  ier_adr;
  unsigned char ier_mask;
};

static const unsigned char *sci_base;
static unsigned char ssr[MAX_SCI_CH];
static const unsigned char *timer8_base;
static const struct int_list_t *int_table;

static const unsigned char h8300h_timer8_base[] = {0x80,0x81,0x90,0x91,0};
static const unsigned char h8300s_timer8_base[] = {0xb0,0xb1,0};
static const unsigned int tpubase[] = {0xffffd0,0xffffe0,0xfffff0,
				       0xfffe80,0xfffe90,0xfffea0};
static const unsigned char h8300h_sci_base[] = {0xb0,0xb8,0xc0};
static const unsigned char h8300s_sci_base[] = {0x78,0x80,0x88};
static const unsigned char h8300sx_sci_base[] = {0x80,0x88,0x60};

extern int h8300hmode;
extern int h8300smode;
extern int h8300sxmode;

static const struct int_list_t h8300h_int_table[]= {
  {24,0xffff64,0x01,0xffff64,0x10},	/* IMIA0 */
  {25,0xffff65,0x01,0xffff65,0x10},	/* IMIB0 */
  {26,0xffff66,0x01,0xffff66,0x10},	/* OVI0  */
  {28,0xffff64,0x02,0xffff64,0x20},	/* IMIA1 */
  {29,0xffff65,0x02,0xffff65,0x20},	/* IMIB1 */
  {30,0xffff66,0x02,0xffff66,0x20},	/* OVI1  */
  {32,0xffff64,0x04,0xffff64,0x40},	/* IMIA2 */
  {33,0xffff65,0x04,0xffff65,0x40},	/* IMIB2 */
  {34,0xffff66,0x04,0xffff66,0x40},	/* OVI2  */
  {36,0xffff82,0x40,0xffff80,0x40},	/* CMIA0 */
  {37,0xffff82,0x80,0xffff80,0x80},	/* CMIB0 */
  {38,0xffff83,0x40,0xffff81,0x40},	/* CMIA1 */
  {38,0xffff83,0x80,0xffff81,0x40},	/* CMIB1 */
  {39,0xffff82,0x20,0xffff80,0x20},	/* TOVI0 */
  {39,0xffff83,0x20,0xffff81,0x20},	/* TOVI1 */
  {40,0xffff92,0x40,0xffff90,0x40},	/* CMIA2 */
  {41,0xffff92,0x80,0xffff90,0x80},	/* CMIB2 */
  {42,0xffff93,0x40,0xffff91,0x40},	/* CMIA3 */
  {42,0xffff93,0x80,0xffff91,0x40},	/* CMIB3 */
  {43,0xffff92,0x20,0xffff90,0x20},	/* TOVI2 */
  {43,0xffff93,0x20,0xffff91,0x20},	/* TOVI3 */
  {52,0xffffb4,0x38,0xffffb2,0x40},	/* ERI0 */
  {53,0xffffb4,0x40,0xffffb2,0x40},	/* RXI0 */
  {54,0xffffb4,0x80,0xffffb2,0x80},	/* TXI0 */
  {55,0xffffb4,0x04,0xffffb2,0x04},	/* TEI0 */
  {56,0xffffbc,0x38,0xffffba,0x40},	/* ERI1 */
  {57,0xffffbc,0x40,0xffffba,0x40},	/* RXI1 */
  {58,0xffffbc,0x80,0xffffba,0x80},	/* TXI1 */
  {59,0xffffbc,0x04,0xffffba,0x04},	/* TEI1 */
  {60,0xffffc4,0x38,0xffffc2,0x40},	/* ERI2 */
  {61,0xffffc4,0x40,0xffffc2,0x40},	/* RXI2 */
  {62,0xffffc4,0x80,0xffffc2,0x80},	/* TXI2 */
  {63,0xffffc4,0x04,0xffffc2,0x04},	/* TEI2 */
  {-1,0,0,0,0}
};

static const struct int_list_t h8300s_int_table[]= {
  {40,0xffffd5,0x01,0xffffd4,0x01},	/* TGI0A */
  {41,0xffffd5,0x02,0xffffd4,0x02},	/* TGI0B */
  {43,0xffffd5,0x10,0xffffd4,0x10},	/* TGI0V */
  {48,0xffffe5,0x01,0xffffe4,0x01},	/* TGI1A */
  {49,0xffffe5,0x01,0xffffe4,0x02},	/* TGI1B */
  {50,0xffffe5,0x10,0xffffe4,0x10},	/* TGI1V */
  {52,0xfffff5,0x01,0xfffff4,0x01},	/* TGI2A */
  {53,0xfffff5,0x02,0xfffff4,0x02},	/* TGI2B */
  {54,0xfffff5,0x10,0xfffff4,0x10},	/* TGI2V */
  {56,0xfffe85,0x01,0xfffe84,0x01},	/* TGI3A */
  {57,0xfffe85,0x02,0xfffe84,0x02},	/* TGI3B */
  {60,0xfffe85,0x10,0xfffe84,0x10},	/* TGI3V */
  {64,0xfffe95,0x01,0xfffe94,0x01},	/* TGI4A */
  {65,0xfffe95,0x02,0xfffe94,0x02},	/* TGI4B */
  {66,0xfffe95,0x10,0xfffe94,0x10},	/* TGI4V */
  {68,0xfffea5,0x01,0xfffea4,0x01},	/* TGI5A */
  {69,0xfffea5,0x02,0xfffea4,0x02},	/* TGI5B */
  {70,0xfffea5,0x10,0xfffea4,0x10},	/* TGI5V */
  {72,0xffffb2,0x40,0xffffb0,0x40},	/* CMIA0 */
  {73,0xffffb2,0x80,0xffffb0,0x80},	/* CMIB0 */
  {74,0xffffb2,0x20,0xffffb0,0x20},	/* CMIA1 */
  {76,0xffffb3,0x40,0xffffb1,0x40},	/* CMIB1 */
  {77,0xffffb3,0x80,0xffffb1,0x40},	/* TOVI0 */
  {78,0xffffb3,0x20,0xffffb1,0x20},	/* TOVI1 */
  {88,0xffff7c,0x38,0xffff7a,0x40},	/* ERI0 */
  {89,0xffff7c,0x40,0xffff7a,0x40},	/* RXI0 */
  {90,0xffff7c,0x80,0xffff7a,0x80},	/* TXI0 */
  {91,0xffff7c,0x04,0xffff7a,0x04}, 	/* TEI0 */
  {92,0xffff84,0x38,0xffff82,0x40},	/* ERI1 */
  {93,0xffff84,0x40,0xffff82,0x40},	/* RXI1 */
  {94,0xffff84,0x80,0xffff82,0x80},	/* TXI1 */
  {95,0xffff84,0x04,0xffff82,0x04}, 	/* TEI1 */
  {96,0xffff8c,0x38,0xffff8a,0x40},	/* ERI2 */
  {97,0xffff8c,0x40,0xffff8a,0x40},	/* RXI2 */
  {98,0xffff8c,0x80,0xffff8a,0x80},	/* TXI2 */
  {99,0xffff8c,0x04,0xffff8a,0x04}, 	/* TEI2 */
  {-1,0,0,0,0}
};
static const struct int_list_t h8300sx_int_table[]= {
  {88,0xffffd5,0x01,0xffffd4,0x01},	/* TGI0A */
  {89,0xffffd5,0x02,0xffffd4,0x02},	/* TGI0B */
  {90,0xffffd5,0x01,0xffffd4,0x01},	/* TGI0C */
  {91,0xffffd5,0x02,0xffffd4,0x02},	/* TGI0D */
  {93,0xffffe5,0x01,0xffffe4,0x01},	/* TGI1A */
  {94,0xffffe5,0x01,0xffffe4,0x02},	/* TGI1B */
  {95,0xffffe5,0x10,0xffffe4,0x10},	/* TGI1V */
  {96,0xffffe5,0x10,0xffffe4,0x10},	/* TGI1U */
  {97,0xfffff5,0x01,0xfffff4,0x01},	/* TGI2A */
  {98,0xfffff5,0x02,0xfffff4,0x02},	/* TGI2B */
  {99,0xfffff5,0x10,0xfffff4,0x10},	/* TGI2V */
  {100,0xfffff5,0x10,0xfffff4,0x10},	/* TGI2U */
  {101,0xfffe85,0x01,0xfffe84,0x01},	/* TGI3A */
  {102,0xfffe85,0x02,0xfffe84,0x02},	/* TGI3B */
  {103,0xfffe85,0x01,0xfffe84,0x01},	/* TGI3A */
  {104,0xfffe85,0x02,0xfffe84,0x02},	/* TGI3B */
  {105,0xfffe85,0x10,0xfffe84,0x10},	/* TGI3V */
  {106,0xfffe95,0x01,0xfffe94,0x01},	/* TGI4A */
  {107,0xfffe95,0x02,0xfffe94,0x02},	/* TGI4B */
  {108,0xfffe95,0x10,0xfffe94,0x10},	/* TGI4V */
  {109,0xfffe95,0x10,0xfffe94,0x10},	/* TGI4U */
  {110,0xfffea5,0x01,0xfffea4,0x01},	/* TGI5A */
  {111,0xfffea5,0x02,0xfffea4,0x02},	/* TGI5B */
  {112,0xfffea5,0x10,0xfffea4,0x10},	/* TGI5V */
  {113,0xfffea5,0x10,0xfffea4,0x10},	/* TGI5V */
  {116,0xffffb2,0x40,0xffffb0,0x40},	/* CMIA0 */
  {117,0xffffb2,0x80,0xffffb0,0x80},	/* CMIB0 */
  {118,0xffffb3,0x80,0xffffb1,0x40},	/* OVI0 */
  {119,0xffffb2,0x20,0xffffb0,0x20},	/* CMIA1 */
  {120,0xffffb3,0x40,0xffffb1,0x40},	/* CMIB1 */
  {121,0xffffb3,0x20,0xffffb1,0x20},	/* OVI1 */
  {144,0xffff7c,0x38,0xffff7a,0x40},	/* ERI0 */
  {145,0xffff7c,0x40,0xffff7a,0x40},	/* RXI0 */
  {146,0xffff7c,0x80,0xffff7a,0x80},	/* TXI0 */
  {147,0xffff7c,0x04,0xffff7a,0x04}, 	/* TEI0 */
  {148,0xffff84,0x38,0xffff82,0x40},	/* ERI1 */
  {149,0xffff84,0x40,0xffff82,0x40},	/* RXI1 */
  {150,0xffff84,0x80,0xffff82,0x80},	/* TXI1 */
  {151,0xffff84,0x04,0xffff82,0x04}, 	/* TEI1 */
  {152,0xffff8c,0x38,0xffff8a,0x40},	/* ERI2 */
  {153,0xffff8c,0x40,0xffff8a,0x40},	/* RXI2 */
  {154,0xffff8c,0x80,0xffff8a,0x80},	/* TXI2 */
  {155,0xffff8c,0x04,0xffff8a,0x04}, 	/* TEI2 */
  {-1,0,0,0,0}
};

void 
timer8(SIM_DESC sd, unsigned int cycles_diff)
{
  static int prescale[3]={8,64,8192};
  const int prescale_div[3]={8,64,8192};
  static unsigned char tcsr[4]={0x00,0x00,0x00,0x00};
  int tm, cnt, pcnt, cor;
  for (pcnt = 0; pcnt < 3; pcnt++)
    {
      prescale[pcnt] -= cycles_diff;
      
      if (prescale[pcnt]<=0) 
	{
	  /* input time pulse */
	  for(tm=0; timer8_base[tm] != 0; tm++)
	    {
	      if ((TCR8(tm) & 0x07) == 0)
		continue;
	      /* internal TCSR status clear */
	      tcsr[tm] &= (TCSR8(tm) & 0xf0);
	      
	      if ((TCR8(tm & 2) & 0x7) == 0x04)
		{
		  /* 16bit mode */
		  if (tm & 1)
		    continue;
		  tcsr[tm+1] &= (TCSR8(tm+1) & 0xf0);
		  cnt = TCNT8(tm) << 8 | TCNT8(tm+1);
		  cnt++;
		  if (cnt >= 0x10000)
		    {
		      tcsr[tm] |= 0x20;
		      cnt = 0;
		    }
		  TCNT8(tm) = cnt >> 8;
		  TCNT8(tm-1) = cnt & 0xff;
		  /* TCORA compare match check */
		  cor = TCORA8(tm) << 8 | TCORA8(tm+1);
		  if (cnt >= cor)
		    {
		      tcsr[tm]|=0x40;
		      if ((TCR8(tm) & 0x18) == 0x08)
			cnt = 0;
		    }
		  if ((cnt & 0xff) >= (cor & 0xff))
		    tcsr[tm+1]|=0x40;
		  /* TCORB compare match check */
		  cor = TCORB8(tm) << 8 | TCORB8(tm+1);
		  if (cnt >= cor)
		    {
		      tcsr[tm]|=0x80;
		      if ((TCR8(tm) & 0x18) == 0x10)
			cnt = 0;
		    }
		  if ((cnt & 0xff) >= (cor & 0xff))
		    tcsr[tm+1]|=0x80;
		  TCNT8(tm) = cnt >> 8;
		  TCNT8(tm+1) = cnt & 0xff;
		  /* update TSCR */
		  TCSR8(tm) &= 0x1f;
		  TCSR8(tm) |= (tcsr[tm] & 0xe0);
		  TCSR8(tm+1) &= 0x1f;
		  TCSR8(tm+1) |= (tcsr[tm+1] & 0xe0);
		}
	      else
		{
		  /* 8bit mode */
		  /* update counter */
		  if ((TCR8(tm) & 0x07) == (pcnt+1))
		    {
		      cnt = ++TCNT8(tm);
		      if (cnt>=0x100)
			{
			  tcsr[tm] |= 0x20;
			  cnt = 0;
			}
		    }
		  /* TCORA compare match check*/
		  if (cnt >= TCORA8(tm))
		    {
		      tcsr[tm]|=0x40;
		      if ((TCR8(tm) & 0x18) == 0x08)
			cnt = 0;
		    }
		  /* TCORB compare match check*/
		  if (cnt >= TCORB8(tm))
		    {
		      tcsr[tm]|=0x80;
		      if ((TCR8(tm) & 0x18) == 0x10)
			cnt = 0;
		    }
		  TCNT8(tm) = cnt;
		  /* update TSCR */
		  TCSR8(tm) &= 0x1f;
		  TCSR8(tm) |= (tcsr[tm] & 0xe0);
		}
	    }
	  prescale[pcnt]+=prescale_div[pcnt];
	}
    }
}

static void 
h8300sx_timer16(SIM_DESC sd, unsigned int cycles_diff)
{
  static int prescale[4]={1,4,16,64};
  const int prescale_div[4]={1,4,16,64};
  static int tsr[TPU_CH];
  int tm, cnt, pcnt, gr, pulse;
  for (pcnt = 0; pcnt < 4; pcnt++) {
    prescale[pcnt] -= cycles_diff;
    pulse = -prescale[pcnt] / prescale_div[cnt];
    if (prescale[pcnt]<=0) 
    {
      /* input time pulse */
      for(tm=0; tm < TPU_CH; tm++) {

	/* Timer enable check */
	if (!(TPU_TSTR & (1 << tm)))
	  continue;

	/* internal TCSR status clear */
	tsr[tm] &= TPU_TSR(tm);
	/* update counter */
        if ((TPU_TCR(tm) & 0x07) == pcnt)
          {
            cnt = ((TPU_TCNTH(tm) << 8) | TPU_TCNTL(tm));
	    cnt += pulse;

	    /* CNT overflow check */
            if (cnt>=0x10000)
              {
                tsr[tm] |= 0x10;
                cnt = 0;
              }

	    /* GRA compare match check*/
	    gr = (TPU_GRAH(tm) << 8) | TPU_GRAL(tm);
	    if (cnt >= gr)
	      {
                tsr[tm] |= 0x1;
		if ((TPU_TCR(tm) & 0x60) == 0x20)
		  cnt = 0;
	      }

	    /* GRB compare match check*/
	    gr = (TPU_GRBH(tm) << 8) | TPU_GRBL(tm);
	    if (cnt >= gr)
	      {
                tsr[tm] |= 0x2;
		if ((TPU_TCR(tm) & 0x60) == 0x20)
		  cnt = 0;
	      }

	    /* update TCNT */
            TPU_TCNTH(tm) = (cnt >> 8);
	    TPU_TCNTL(tm) = cnt & 0xff;
          }

      }
      prescale[pcnt]+=prescale_div[pcnt];
      /* update TSR */
      TPU_TSR(tm) |= tsr[tm];
    }
  }
}

static void 
h8300s_timer16(SIM_DESC sd, unsigned int cycles_diff)
{
  static int prescale[4]={1,4,16,64};
  const int prescale_div[4]={1,4,16,64};
  static int tsr[TPU_CH];
  int tm, cnt, pcnt, gr, pulse;
  for (pcnt = 0; pcnt < 4; pcnt++) {
    prescale[pcnt] -= cycles_diff;
    pulse = -prescale[pcnt] / prescale_div[pcnt];
    if (prescale[pcnt]<=0) 
    {
      /* input time pulse */
      for(tm=0; tm < TPU_CH; tm++) {

	/* Timer enable check */
	if (!(TPU_TSTR & (1 << tm)))
	  continue;

	/* internal TCSR status clear */
	tsr[tm] &= TPU_TSR(tm);
	/* update counter */
        if ((TPU_TCR(tm) & 0x07) == pcnt)
          {
            cnt = ((TPU_TCNTH(tm) << 8) | TPU_TCNTL(tm));
	    cnt += pulse;

	    /* CNT overflow check */
            if (cnt>=0x10000)
              {
                tsr[tm] |= 0x10;
                cnt = 0;
              }

	    /* GRA compare match check*/
	    gr = (TPU_GRAH(tm) << 8) | TPU_GRAL(tm);
	    if (cnt >= gr)
	      {
                tsr[tm] |= 0x1;
		if ((TPU_TCR(tm) & 0x60) == 0x20)
		  cnt = 0;
	      }

	    /* GRB compare match check*/
	    gr = (TPU_GRBH(tm) << 8) | TPU_GRBL(tm);
	    if (cnt >= gr)
	      {
                tsr[tm] |= 0x2;
		if ((TPU_TCR(tm) & 0x60) == 0x20)
		  cnt = 0;
	      }

	    /* update TCNT */
            TPU_TCNTH(tm) = (cnt >> 8);
	    TPU_TCNTL(tm) = cnt & 0xff;
          }

      }
      prescale[pcnt]+=prescale_div[pcnt];
      /* update TSR */
      TPU_TSR(tm) |= tsr[tm];
    }
  }
}

static void 
h8300h_timer16(SIM_DESC sd, unsigned int cycles_diff)
{
  static int prescale[4]={1,2,4,8};
  const int prescale_div[4]={1,2,4,8};
  static int tisra, tisrb, tisrc;
  int tm, cnt, pcnt, gr, pulse;
  for (pcnt = 0; pcnt < 4; pcnt++) {
    prescale[pcnt] -= cycles_diff;
    pulse = -prescale[pcnt] / prescale_div[pcnt];
    if (prescale[pcnt]<=0) 
    {
      /* input time pulse */
      for(tm=0; tm < 3; tm++) {

	/* Timer enable check */
	if (!(TSTR16 & (1 << tm)))
	  continue;

	/* internal TCSR status clear */
	tisra &= (0x07 & (TISRA16 & (1 << tm)));
	tisrb &= (0x07 & (TISRB16 & (1 << tm)));
	tisrc &= (0x07 & (TISRC16 & (1 << tm)));
	/* update counter */
        if ((TCR16(tm) & 0x07) == pcnt)
          {
            cnt = ((TCNT16H(tm) << 8) | TCNT16L(tm));
	    cnt += pulse;

	    /* CNT overflow check */
            if (cnt>=0x10000)
              {
                tisrc |= (1 << tm);
                cnt = 0;
              }

	    /* GRA compare match check*/
	    gr = (GRA16H(tm) << 8) | GRA16L(tm);
	    if (cnt >= gr)
	      {
                tisra |= (1 << tm);
		if ((TCR16(tm) & 0x60) == 0x20)
		  cnt = 0;
	      }

	    /* GRB compare match check*/
	    gr = (GRB16H(tm) << 8) | GRB16L(tm);
	    if (cnt >= gr)
	      {
                tisrb |= (1 << tm);
		if ((TCR16(tm) & 0x60) == 0x40)
		  cnt = 0;
	      }
	    /* update TCNT */
            TCNT16H(tm) = (cnt >> 8);
	    TCNT16L(tm) = cnt & 0xff;
          }

      }
      prescale[pcnt]+=prescale_div[pcnt];
      /* update TSCR */
      TISRA16 &= 0x70;
      TISRA16 |= tisra;
      TISRB16 &= 0x70;
      TISRB16 |= tisrb;
      TISRC16 &= 0x70;
      TISRC16 |= tisrc;
    }
  }
}

static struct {
  int fd;
  int socket;
  int iac;
  unsigned char cmd;
  struct sockaddr_in local;
  struct sockaddr_in remote;
  struct termios old_attr;
} sci_port[MAX_SCI_CH];

enum {PORT_NONE, PORT_PTY,PORT_NET};
static int sci_port_type = PORT_NONE;

static unsigned int 
sci_complete_time(SIM_DESC sd, int ch)
{
  int length;
  int div[]={1,4,16,64};
  length = (SMR(ch) & 0x40)?7:8;
  length += (SMR(ch) & 0x20)?1:0;
  length += (SMR(ch) & 0x08)?1:0;
  length += 2;
  return length * 32 * div[SMR(ch) & 0x03] * BRR(ch);
}

static void 
sci_send_data(int ch, int txd)
{
  char dt = txd;
  if (sci_port[ch].fd >= 0) {
    if (write(sci_port[ch].fd, &dt, 1) > 0)
      fsync(sci_port[ch].fd);
    else
      if (errno != EAGAIN)
	sci_port[ch].fd = -1;
  }
}

static void
telnet_escape(int ch, char rd)
{
  unsigned char cmd = sci_port[ch].cmd;
  unsigned char rep[3];
  switch(sci_port[ch].iac)
    {
    case 1:
      sci_port[ch].cmd = rd;
      sci_port[ch].iac++;
      break;
    case 2:
      if ((rd == 1 || rd == 3) && cmd == 0xfd)
	{
	  sci_port[ch].iac = 0;
	  return;
	}
      else if (rd == 1 || rd == 3)
	{
	  if (cmd == 0xfb) 
	    cmd = 0xfd;
	  else if (cmd == 0xfd)
	    cmd = 0xfb;
	} 
      else
	{
	  if (cmd == 0xfb) 
	    cmd = 0xfe;
	  else if (cmd == 0xfd)
	  cmd = 0xfc;
	}
      rep[0] = 0xff;
      rep[1] = cmd;
      rep[2] = rd;
      write(sci_port[ch].fd, rep, sizeof(rep));
      sci_port[ch].iac = 0;
      break;
    }
}

static void
telnet_request(int fd)
{
  static unsigned char req[6] = {0xff, 0xfb, 0x03, 0xff, 0xfb, 0x01};
  write(fd, req, sizeof(req));
}


int
sci_rcv_data(int ch, int *rxd)
{
  unsigned char rd;
  if (sci_port[ch].fd >= 0)
    {
      if( read(sci_port[ch].fd, &rd , 1) > 0 )
	{
	  if (sci_port_type == PORT_NET)
	    {
	      if (sci_port[ch].iac > 0)
		{
		  telnet_escape(ch, rd);
		  return 0;
		}
	      else
		if (rd == 0xff)
		  {
		    sci_port[ch].iac = 1;
		    return 0;
		  }
	    }
	  *rxd = rd;
	  return 1;
	}
      else
	{
	  if (errno == EAGAIN)
	    {
	      return 0;
	    }
	  else
	    {
	      close(sci_port[ch].fd);
	      sci_port[ch].fd = -1;
	    }
	}
    }
  return 0;
}

static int net_accept(void)
{
  int ch;
  for (ch = 0; ch < MAX_SCI_CH; ch++)
    {
      if(sci_port[ch].fd == -1)
	{
	  int connectfd;
	  socklen_t rem_size = sizeof(sci_port[ch].remote);
	  connectfd = accept(sci_port[ch].socket, 
			     (struct sockaddr *)&sci_port[ch].remote, 
			     &rem_size);
	  if (connectfd > 0)
	    {
	      unsigned char rd;
	      int flag;
	      sci_port[ch].fd = connectfd;
	      telnet_request(connectfd);
	      sci_port[ch].iac = 0;
	      flag = fcntl(sci_port[ch].fd, F_GETFL, 0);
	      fcntl(sci_port[ch].fd, F_SETFL, flag | O_NONBLOCK);
	      
	      while ( read(sci_port[ch].fd, &rd , 1) > 0 )
		{
		  if (sci_port[ch].iac > 0)
		    {
		      telnet_escape(ch, rd);
		      return 1;
		    }
		  else
		    if (rd == 0xff)
		      {
			sci_port[ch].iac = 1;
			return 1;
		      }
		}
	    }
	}
    }
  return 0;
}

static void
sci(SIM_DESC sd, unsigned int cycles_diff)
{
  static int tx_end_time[MAX_SCI_CH];
  static int rx_end_time[MAX_SCI_CH];
  static int txstate = 0;
  int data;
  int ch;

  if (sci_port_type == PORT_NET && net_accept())
    return;

  for (ch = 0; ch < MAX_SCI_CH; ch++)
    {
      /* clear internal ssr */
      ssr[ch] &= SSR(ch);

      /* Tx request */
      if((SCR(ch) & 0x20) && !(ssr[ch] & 0x80) && (txstate == 0))
	{
	  sci_send_data(ch,TDR(ch));
	  ssr[ch] &= ~0x04;
	  /* TSR shift time */
	  tx_end_time[ch] = 1;
	  txstate = 1;
	}
      tx_end_time[ch] -= cycles_diff;
      /* Tx complete check */
      if(((ssr[ch] & 0x84) != 0x84) && (tx_end_time[ch] <= 0))
	if (!(ssr[ch] & 0x80))
	  {
	    ssr[ch] |= 0x80;
	    tx_end_time[ch] = sci_complete_time(sd, ch);
	    txstate = 0;
	  } 
	else
	  ssr[ch] |= 0x04; /* All data transmit done */
      rx_end_time[ch] -= cycles_diff;
      /* Rx check */
      if (rx_end_time[ch] <= 0)
	/* RSR free & Rx Enabled */
	if ((SCR(ch) & 0x10) && sci_rcv_data(ch, &data))
	  {
	    /* Rx Overrun */
	    if(ssr[ch] & 0x40)
	      ssr[ch] |= 0x20;
	    else
	  /* Rx ok */
	      {
		RDR(ch)=data;
		ssr[ch] |= 0x40;
	      }
	    /* RSR shift time */
	    rx_end_time[ch] = sci_complete_time(sd, ch);
	  }

      /* update SSR */
      SSR(ch) = ssr[ch];
    }
}

static int
get_priority(SIM_DESC sd, int vec)
{
  const static int ipr_bit[] = {
    -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1,  7,  6,  5,  5,
     4,  4,  4,  4,  3,  3,  3,  3,
     2,  2,  2,  2,  1,  1,  1,  1,
     0,  0,  0,  0, 15, 15, 15, 15,
    14, 14, 14, 14, 13, 13, 13, 13,
    -1, -1, -1, -1, 11, 11, 11, 11,
    10, 10, 10, 10,  9,  9,  9,  9,
  };
  const static unsigned char ipr_table[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* 0 - 7 */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* 8 - 15 */
    0x03, 0x02, 0x01, 0x00, 0x13, 0x12, 0x11, 0x10, /* 16 - 23 */
    0x23, 0x22, 0x21, 0x20, 0x33, 0x32, 0x31, 0x30, /* 24 - 31 */
    0x43, 0x42, 0x41, 0x40, 0x53, 0x53, 0x52, 0x52, /* 32 - 39 */
    0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, /* 40 - 47 */
    0x50, 0x50, 0x50, 0x50, 0x63, 0x63, 0x63, 0x63, /* 48 - 55 */
    0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, /* 56 - 63 */
    0x61, 0x61, 0x61, 0x61, 0x60, 0x60, 0x60, 0x60, /* 64 - 71 */
    0x73, 0x73, 0x73, 0x73, 0x72, 0x72, 0x72, 0x72, /* 72 - 79 */
    0x71, 0x71, 0x71, 0x71, 0x70, 0x83, 0x82, 0x81, /* 80 - 87 */
    0x80, 0x80, 0x80, 0x80, 0x93, 0x93, 0x93, 0x93, /* 88 - 95 */
    0x92, 0x92, 0x92, 0x92, 0x91, 0x91, 0x91, 0x91, /* 96 - 103 */
    0x90, 0x90, 0x90, 0x90, 0xa3, 0xa3, 0xa3, 0xa3, /* 104 - 111 */
    0xa2, 0xa2, 0xa2, 0xa2, 0xa1, 0xa1, 0xa1, 0xa1, /* 112 - 119 */
    0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, /* 120 - 127 */
  };


  if (h8300smode)
    {
      unsigned short ipr;
      int pos;
      if ((pos = ipr_table[vec]) == 0xff)
	return 0;
      ipr = (STATE_CPU(sd, 0)->memory[IPRA_H8300S + ((pos & 0xf0) >> 3)] << 8) |
	    (STATE_CPU(sd, 0)->memory[IPRA_H8300S + ((pos & 0xf0) >> 3) + 1]);
      return  vec + ((ipr >> ((pos & 0x0f) * 4)) & 7) * 0x100;
    }
  else if (h8300hmode)
    {
      int b;
      unsigned char ipr;
      if ((b = ipr_bit[vec]) < 0)
	return 0;
      ipr = (b < 8)?STATE_CPU(sd, 0)->memory[IPRA_H8300H]:
	            STATE_CPU(sd, 0)->memory[IPRB_H8300H];
      b = 1 << (b & 7);
      if (ipr & b)
	return vec + 0x100;
      else
	return vec;
    }
}

static int 
intcont(SIM_DESC sd)
{
  int irqno;
  for (irqno=0; int_table[irqno].vector > 0; irqno++)
    {
      if((STATE_CPU(sd, 0)->memory[int_table[irqno].ier_adr] & 
	  int_table[irqno].ier_mask) &&
	 (STATE_CPU(sd, 0)->memory[int_table[irqno].isr_adr] & 
	  int_table[irqno].isr_mask))
	return get_priority(sd, int_table[irqno].vector);
    }
  return 0;
}

int 
iosimulation(SIM_DESC sd, int cycles)
{
  static unsigned int prev_cycles = 0;
  unsigned int cycles_diff;
  cycles_diff = (cycles < prev_cycles)?cycles:(cycles - prev_cycles);
  prev_cycles = cycles;
  timer8(sd, cycles_diff);
  if (h8300smode)
    h8300s_timer16(sd, cycles_diff);
  else if (h8300hmode)
    h8300h_timer16(sd, cycles_diff);
  sci(sd, cycles_diff);
  return intcont(sd);
}

void init_ioregs(SIM_DESC sd)
{
  struct INITTABLE {
    unsigned char addr;
    unsigned char data;
  };
  const struct INITTABLE h8300h_reg_ini[] = {
    0x80,0x00,
    0x81,0x00,
    0x82,0x00,
    0x83,0x00,
    0x84,0xff,
    0x85,0xff,
    0x86,0xff,
    0x87,0xff,
    0x88,0x00,
    0x89,0x00,
    0x90,0x00,
    0x91,0x00,
    0x92,0x00,
    0x93,0x00,
    0x94,0xff,
    0x95,0xff,
    0x96,0xff,
    0x97,0xff,
    0x98,0x00,
    0x99,0x00,
    0xb0,0x00,
    0xb1,0xff,
    0xb2,0x00,
    0xb3,0xff,
    0xb4,0x84,
    0xb8,0x00,
    0xb9,0xff,
    0xba,0x00,
    0xbb,0xff,
    0xbc,0x84,
    0xc0,0x00,
    0xc1,0xff,
    0xc2,0x00,
    0xc3,0xff,
    0xc4,0x84,
  };
  const struct INITTABLE h8300s_reg_ini[] = {
    0xb0,0x00,
    0xb1,0x00,
    0xb2,0x00,
    0xb3,0x00,
    0xb4,0xff,
    0xb5,0xff,
    0xb6,0xff,
    0xb7,0xff,
    0xb8,0x00,
    0xb9,0x00,
    0x78,0x00,
    0x79,0xff,
    0x7a,0x00,
    0x7b,0xff,
    0x7c,0x84,
    0x80,0x00,
    0x81,0xff,
    0x82,0x00,
    0x83,0xff,
    0x84,0x84,
    0x88,0x00,
    0x89,0xff,
    0x8a,0x00,
    0x8b,0xff,
    0x8c,0x84,
  };
  const struct INITTABLE h8300sx_reg_ini[] = {
    0xb0,0x00,
    0xb1,0x00,
    0xb2,0x00,
    0xb3,0x00,
    0xb4,0xff,
    0xb5,0xff,
    0xb6,0xff,
    0xb7,0xff,
    0xb8,0x00,
    0xb9,0x00,
    0x80,0x00,
    0x81,0xff,
    0x82,0x00,
    0x83,0xff,
    0x84,0x84,
    0x88,0x00,
    0x89,0xff,
    0x8a,0x00,
    0x8b,0xff,
    0x8c,0x84,
    0x60,0x00,
    0x61,0xff,
    0x62,0x00,
    0x63,0xff,
    0x64,0x84,
  };
  int c;
  if (h8300sxmode) {
    sci_base = h8300sx_sci_base;
    timer8_base = h8300s_timer8_base;
    int_table = h8300sx_int_table;
    for(c=0;c<sizeof(h8300sx_reg_ini)/sizeof(struct INITTABLE);c++)
      STATE_CPU(sd, 0)->eightbit[h8300sx_reg_ini[c].addr]=h8300sx_reg_ini[c].data;
  }
  else if (h8300smode) {
    sci_base = h8300s_sci_base;
    timer8_base = h8300s_timer8_base;
    int_table = h8300s_int_table;
    for(c=0;c<sizeof(h8300s_reg_ini)/sizeof(struct INITTABLE);c++)
      STATE_CPU(sd, 0)->eightbit[h8300s_reg_ini[c].addr]=h8300s_reg_ini[c].data;
  }
  else if (h8300hmode) {
    sci_base = h8300h_sci_base;
    timer8_base = h8300h_timer8_base;
    int_table = h8300h_int_table;
    for(c=0;c<sizeof(h8300h_reg_ini)/sizeof(struct INITTABLE);c++)
      STATE_CPU(sd, 0)->eightbit[h8300h_reg_ini[c].addr]=h8300h_reg_ini[c].data;
  }
  for(c = 0; c< MAX_SCI_CH; c++)
    ssr[c] = 0x84;
}

static char *openpty(int ch)
{
  const char nm[]="0123456789ABCDEF";
  static char ptyname[16];
  int c1,c2,fd;
  struct termios attr;
  fd = open("/dev/ptmx",O_RDWR|O_NONBLOCK);
  if(fd >= 0) {
    grantpt(fd);
    unlockpt(fd);
    ptsname_r(fd, ptyname, sizeof(ptyname));
  } else {
    for(c1='a';c1<='z';c1++)
      for(c2=0;c2<sizeof(nm)-1;c2++) {
	sprintf(ptyname,"/dev/pty%c%c",c1,nm[c2]);
	fd=open(ptyname,O_RDWR|O_NONBLOCK);
	if(fd != -1)
	  break ;
      }
    ptyname[5]='t';
  }
  if (fd >= 0) {
    sci_port[ch].fd = fd;
    tcgetattr(fd, &attr);
    memcpy(&sci_port[ch].old_attr, &attr, sizeof(struct termios));
    attr.c_lflag &= ~ICANON;
    attr.c_cc[VMIN] = 0;
    attr.c_cc[VTIME] =0;
    tcsetattr(fd, TCSAFLUSH, &attr);
    return ptyname;
  } else {
    sci_port[ch].fd = -1;
    return NULL;
  }
}

void sci_open_pty(struct host_callback_struct *callback)
{
  int ch;
  int max_ch;
  char *pty;
  for (ch = 0; ch < MAX_SCI_CH; ch++)
    {
      pty = openpty(ch);
      if (pty)
	(*callback->printf_filtered) (callback, "SCI%d = %s\n",ch ,pty);
    }
}

void sci_open_net(struct host_callback_struct *callback, int port)
{
  int c;
  int flag;
  int socketfd;
  sci_port_type = PORT_NET;
  for (c = 0; c < MAX_SCI_CH; c++) {
    memset(&sci_port[c].local, 0, sizeof(sci_port[c].local));
    sci_port[c].local.sin_family = AF_INET;
    sci_port[c].local.sin_addr.s_addr = htonl(INADDR_ANY);
    sci_port[c].local.sin_port = htons(port + c);
    sci_port[c].fd = -1;
    socketfd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketfd >= 0)
      {
	bind(socketfd, (struct sockaddr *)&sci_port[c].local, sizeof(sci_port[c].local));
	flag = fcntl(socketfd, F_GETFL, 0);
	fcntl(socketfd, F_SETFL, flag | O_NONBLOCK);
	listen(socketfd, 1);
	sci_port[c].socket = socketfd;
	(*callback->printf_filtered) (callback, "SCI%d = %d\n",c ,port+c);
      }
  }
}

void sci_close(void)
{
  int ch;
  if (sci_port_type == PORT_NONE)
    return;
  for (ch = 0; ch < MAX_SCI_CH; ch++) {
    if(sci_port[ch].fd != -1) {
      if (sci_port_type == PORT_PTY)
	tcsetattr(sci_port[ch].fd, TCSAFLUSH, &sci_port[ch].old_attr);
      close(sci_port[ch].fd);
      if (sci_port_type == PORT_NET)
	close(sci_port[ch].socket);
    }
  }
}
