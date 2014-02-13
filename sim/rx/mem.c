/* mem.c --- memory for RX simulator.

Copyright (C) 2005-2014 Free Software Foundation, Inc.
Contributed by Red Hat, Inc.

This file is part of the GNU simulators.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* This slows down the simulator and we get some false negatives from
   gcc, like when it uses a long-sized hole to hold a byte-sized
   variable, knowing that it doesn't care about the other bits.  But,
   if you need to track down a read-from-unitialized bug, set this to
   1.  */
#define RDCHECK 0

#include "config.h"
#include <stdio.h>
#define __USE_XOPEN
#define __USE_GNU
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <termios.h>

#include "opcode/rx.h"
#include "mem.h"
#include "cpu.h"
#include "syscalls.h"
#include "misc.h"
#include "err.h"
#include "gdb/callback.h"

#define L1_BITS  (10)
#define L2_BITS  (10)
#define OFF_BITS PAGE_BITS

#define L1_LEN  (1 << L1_BITS)
#define L2_LEN  (1 << L2_BITS)
#define OFF_LEN (1 << OFF_BITS)

static unsigned char **pt[L1_LEN];
static unsigned char **ptr[L1_LEN];
static RX_Opcode_Decoded ***ptdc[L1_LEN];

/* [ get=0/put=1 ][ byte size ] */
static unsigned int mem_counters[2][5];

#define COUNT(isput,bytes)                                      \
  if (verbose && enable_counting) mem_counters[isput][bytes]++

enum mem_access_type {MEM_RL,MEM_RQ,MEM_RW,MEM_RB,MEM_WL,MEM_WQ,MEM_WW,MEM_WB};

struct memlog {
  unsigned long mpc;
  unsigned long addr;
  unsigned long data;
  enum mem_access_type type;
};

static struct memlog *memlog_buffer;
static unsigned long memlogsize;
static unsigned long memlogtail;

void
init_mem (void)
{
  int i, j;

  for (i = 0; i < L1_LEN; i++)
    if (pt[i])
      {
	for (j = 0; j < L2_LEN; j++)
	  if (pt[i][j])
	    free (pt[i][j]);
	free (pt[i]);
      }
  memset (pt, 0, sizeof (pt));
  memset (ptr, 0, sizeof (ptr));
  memset (mem_counters, 0, sizeof (mem_counters));

  if (memlog_buffer)
    free(memlog_buffer);
  memlog_buffer = malloc(65536);
  memlogsize = 65536 / sizeof(struct memlog);
  memlogtail = 0;
}

unsigned char *
rx_mem_ptr (unsigned long address, enum mem_ptr_action action)
{
  int pt1 = (address >> (L2_BITS + OFF_BITS)) & ((1 << L1_BITS) - 1);
  int pt2 = (address >> OFF_BITS) & ((1 << L2_BITS) - 1);
  int pto = address & ((1 << OFF_BITS) - 1);

  if (address == 0)
    execution_error (SIM_ERR_NULL_POINTER_DEREFERENCE, 0);

  if (pt[pt1] == 0)
    {
      pt[pt1] = (unsigned char **) calloc (L2_LEN, sizeof (char **));
      ptr[pt1] = (unsigned char **) calloc (L2_LEN, sizeof (char **));
      ptdc[pt1] = (RX_Opcode_Decoded ***) calloc (L2_LEN, sizeof (RX_Opcode_Decoded ***));
    }
  if (pt[pt1][pt2] == 0)
    {
      if (action == MPA_READING)
	execution_error (SIM_ERR_READ_UNWRITTEN_PAGES, address);

      pt[pt1][pt2] = (unsigned char *) calloc (OFF_LEN, 1);
      ptr[pt1][pt2] = (unsigned char *) calloc (OFF_LEN, 1);
      ptdc[pt1][pt2] = (RX_Opcode_Decoded **) calloc (OFF_LEN, sizeof(RX_Opcode_Decoded *));
    }
  else if (action == MPA_READING
	   && ptr[pt1][pt2][pto] == MC_UNINIT)
    execution_error (SIM_ERR_READ_UNWRITTEN_BYTES, address);

  if (action == MPA_WRITING)
    {
      int pto_dc;
      if (ptr[pt1][pt2][pto] == MC_PUSHED_PC)
	execution_error (SIM_ERR_CORRUPT_STACK, address);
      ptr[pt1][pt2][pto] = MC_DATA;

      /* The instruction decoder doesn't store it's decoded instructions
         at word swapped addresses.  Therefore, when clearing the decode
	 cache, we have to account for that here.  */
      pto_dc = pto ^ (rx_big_endian ? 3 : 0);
      if (ptdc[pt1][pt2][pto_dc])
	{
	  free (ptdc[pt1][pt2][pto_dc]);
	  ptdc[pt1][pt2][pto_dc] = NULL;
	}
    }

  if (action == MPA_CONTENT_TYPE)
    return (unsigned char *) (ptr[pt1][pt2] + pto);

  if (action == MPA_DECODE_CACHE)
    return (unsigned char *) (ptdc[pt1][pt2] + pto);

  return pt[pt1][pt2] + pto;
}

RX_Opcode_Decoded **
rx_mem_decode_cache (unsigned long address)
{
  return (RX_Opcode_Decoded **) rx_mem_ptr (address, MPA_DECODE_CACHE);
}

static inline int
is_reserved_address (unsigned int address)
{
  return (address >= 0x00020000 && address < 0x00080000)
    ||   (address >= 0x00100000 && address < 0x01000000)
    ||   (address >= 0x08000000 && address < 0xff000000);
}

static void
used (int rstart, int i, int j)
{
  int rend = i << (L2_BITS + OFF_BITS);
  rend += j << OFF_BITS;
  if (rstart == 0xe0000 && rend == 0xe1000)
    return;
  printf ("mem:   %08x - %08x (%dk bytes)\n", rstart, rend - 1,
	  (rend - rstart) / 1024);
}

static char *
mcs (int isput, int bytes)
{
  return comma (mem_counters[isput][bytes]);
}

void
mem_usage_stats ()
{
  int i, j;
  int rstart = 0;
  int pending = 0;

  for (i = 0; i < L1_LEN; i++)
    if (pt[i])
      {
	for (j = 0; j < L2_LEN; j++)
	  if (pt[i][j])
	    {
	      if (!pending)
		{
		  pending = 1;
		  rstart = (i << (L2_BITS + OFF_BITS)) + (j << OFF_BITS);
		}
	    }
	  else if (pending)
	    {
	      pending = 0;
	      used (rstart, i, j);
	    }
      }
    else
      {
	if (pending)
	  {
	    pending = 0;
	    used (rstart, i, 0);
	  }
      }
  /*       mem foo: 123456789012 123456789012 123456789012 123456789012
            123456789012 */
  printf ("                 byte        short        3byte         long"
          "       opcode\n");
  if (verbose > 1)
    {
      /* Only use comma separated numbers when being very verbose.
	 Comma separated numbers are hard to parse in awk scripts.  */
      printf ("mem get: %12s %12s %12s %12s %12s\n", mcs (0, 1), mcs (0, 2),
	      mcs (0, 3), mcs (0, 4), mcs (0, 0));
      printf ("mem put: %12s %12s %12s %12s\n", mcs (1, 1), mcs (1, 2),
	      mcs (1, 3), mcs (1, 4));
    }
  else
    {
      printf ("mem get: %12u %12u %12u %12u %12u\n",
	      mem_counters[0][1], mem_counters[0][2],
	      mem_counters[0][3], mem_counters[0][4],
	      mem_counters[0][0]);
      printf ("mem put: %12u %12u %12u %12u\n",
	      mem_counters [1][1], mem_counters [1][2],
	      mem_counters [1][3], mem_counters [1][4]);
    }
}

unsigned long
mem_usage_cycles (void)
{
  unsigned long rv = mem_counters[0][0];
  rv += mem_counters[0][1] * 1;
  rv += mem_counters[0][2] * 2;
  rv += mem_counters[0][3] * 3;
  rv += mem_counters[0][4] * 4;
  rv += mem_counters[1][1] * 1;
  rv += mem_counters[1][2] * 2;
  rv += mem_counters[1][3] * 3;
  rv += mem_counters[1][4] * 4;
  return rv;
}

static void 
add_memlog(unsigned long mpc, unsigned long addr, enum mem_access_type type, unsigned long data)
{
  if (memlogsize == memlogtail) {
	  memlog_buffer = (struct memlog *)realloc(memlog_buffer, 
						  (memlogsize * sizeof(struct memlog) + 65536));
	  memlogsize += 65536 / sizeof(struct memlog);
  }
  if (memlog_buffer) {
    memlog_buffer[memlogtail].mpc  = mpc;
    memlog_buffer[memlogtail].addr = addr;
    memlog_buffer[memlogtail].type = type;
    memlog_buffer[memlogtail].data = data;
    memlogtail++;
  }
}

struct mem_access_flag
{
  unsigned long address;
  int flag;
};

static struct mem_access_flag mem_access_flags[] = {
  { .address = 0x00088243, },
  { .address = 0x00088245, },
  { .address = 0x0008824b, },
  { .address = 0x0008824d, },
  { .address = 0x00088253, },
  { .address = 0x00088255, },
  { .address = 0x0008825b, },
  { .address = 0x0008825d, },
  { .address = 0x00088263, },
  { .address = 0x00088265, },
  { .address = 0x0008826b, },
  { .address = 0x0008826d, },
  { .address = 0x00088273, },
  { .address = 0x00088275, },
};

enum {mem_none, mem_r, mem_w};

static inline void 
set_rw_flag(unsigned long address, int flag)
{
  int i;
  if (address < 0x00080000 || address >= 0x00100000)
	  return ;
  for (i = 0; i < sizeof(mem_access_flags)/sizeof(struct mem_access_flag); i++)
    {
      if (address == mem_access_flags[i].address)
	{
	  mem_access_flags[i].flag = flag;
	  break;
	}
    }
}

static inline int
get_rw_flag(unsigned long address)
{
  int i;
  for (i = 0; i < sizeof(mem_access_flags)/sizeof(struct mem_access_flag); i++)
    {
      if (address == mem_access_flags[i].address)
	{
	  return mem_access_flags[i].flag;
	}
    }
  return mem_none;
}

void
mem_put_byte (unsigned int address, unsigned char value)
{
  unsigned char *m;

  m = rx_mem_ptr (address, MPA_WRITING);
  if (is_reserved_address (address))
    generate_access_exception ();
  else
    {
      *m = value;
      set_rw_flag(address, mem_w);
    }
}

void
mem_put_qi (int address, unsigned char value, unsigned long mpc)
{
  if (trace)
    add_memlog(mpc, address, MEM_WB, value & 0xff);
  mem_put_byte (address, value & 0xff);
  COUNT (1, 1);
}

void
mem_put_hi (int address, unsigned short value, unsigned long mpc)
{
  if (trace)
    add_memlog(mpc, address, MEM_WW, value & 0xffff);
  if (rx_big_endian)
    {
      mem_put_byte (address, value >> 8);
      mem_put_byte (address + 1, value & 0xff);
    }
  else
    {
      mem_put_byte (address, value & 0xff);
      mem_put_byte (address + 1, value >> 8);
    }
  COUNT (1, 2);
}

void
mem_put_psi (int address, unsigned long value, unsigned long mpc)
{
  if (trace)
    add_memlog(mpc, address, MEM_WQ, value & 0xffffff);
  if (rx_big_endian)
    {
      mem_put_byte (address, value >> 16);
      mem_put_byte (address + 1, (value >> 8) & 0xff);
      mem_put_byte (address + 2, value & 0xff);
    }
  else
    {
      mem_put_byte (address, value & 0xff);
      mem_put_byte (address + 1, (value >> 8) & 0xff);
      mem_put_byte (address + 2, value >> 16);
    }
  COUNT (1, 3);
}

void
mem_put_si (int address, unsigned long value, unsigned long mpc)
{
  if (trace)
    add_memlog(mpc, address, MEM_WL, value & 0xffffffff);
  if (rx_big_endian)
    {
      mem_put_byte (address + 0, (value >> 24) & 0xff);
      mem_put_byte (address + 1, (value >> 16) & 0xff);
      mem_put_byte (address + 2, (value >> 8) & 0xff);
      mem_put_byte (address + 3, value & 0xff);
    }
  else
    {
      mem_put_byte (address + 0, value & 0xff);
      mem_put_byte (address + 1, (value >> 8) & 0xff);
      mem_put_byte (address + 2, (value >> 16) & 0xff);
      mem_put_byte (address + 3, (value >> 24) & 0xff);
    }
  COUNT (1, 4);
}

void
mem_put_blk (int address, void *bufptr, int nbytes)
{
  if (enable_counting)
    mem_counters[1][1] += nbytes;
  while (nbytes--)
    mem_put_byte (address++, *(unsigned char *) bufptr++);
}

unsigned char
mem_get_pc (int address)
{
  unsigned char *m = rx_mem_ptr (address, MPA_READING);
  COUNT (0, 0);
  return *m;
}

static unsigned char
mem_get_byte (unsigned int address)
{
  unsigned char *m;

  m = rx_mem_ptr (address, MPA_READING);
  if (is_reserved_address (address))
    generate_access_exception ();
  else
    set_rw_flag(address, mem_r);
  return *m;
}

unsigned char
mem_get_qi (int address, unsigned long mpc)
{
  unsigned char rv;
  rv = mem_get_byte (address);
  COUNT (0, 1);
  if (trace)
    add_memlog(mpc, address, MEM_RB, rv);
  return rv;
}

unsigned short
mem_get_hi (int address, unsigned long mpc)
{
  unsigned short rv;
  if (rx_big_endian)
    {
      rv = mem_get_byte (address) << 8;
      rv |= mem_get_byte (address + 1);
    }
  else
    {
      rv = mem_get_byte (address);
      rv |= mem_get_byte (address + 1) << 8;
    }
  COUNT (0, 2);
  if (trace)
    add_memlog(mpc, address, MEM_RW, rv);
  return rv;
}

unsigned long
mem_get_psi (int address, unsigned long mpc)
{
  unsigned long rv;
  if (rx_big_endian)
    {
      rv = mem_get_byte (address + 2);
      rv |= mem_get_byte (address + 1) << 8;
      rv |= mem_get_byte (address) << 16;
    }
  else
    {
      rv = mem_get_byte (address);
      rv |= mem_get_byte (address + 1) << 8;
      rv |= mem_get_byte (address + 2) << 16;
    }
  COUNT (0, 3);
  if (trace)
    add_memlog(mpc, address, MEM_RQ, rv);
  return rv;
}

unsigned long
mem_get_si (int address, unsigned long mpc)
{
  unsigned long rv;
  if (rx_big_endian)
    {
      rv = mem_get_byte (address + 3);
      rv |= mem_get_byte (address + 2) << 8;
      rv |= mem_get_byte (address + 1) << 16;
      rv |= mem_get_byte (address) << 24;
    }
  else
    {
      rv = mem_get_byte (address);
      rv |= mem_get_byte (address + 1) << 8;
      rv |= mem_get_byte (address + 2) << 16;
      rv |= mem_get_byte (address + 3) << 24;
    }
  COUNT (0, 4);
  if (trace)
    add_memlog(mpc, address, MEM_RL, rv);
  return rv;
}

void
mem_get_blk (int address, void *bufptr, int nbytes)
{
  if (enable_counting)
    mem_counters[0][1] += nbytes;
  while (nbytes--)
    *(char *) bufptr++ = mem_get_byte (address++);
}

int
sign_ext (int v, int bits)
{
  if (bits < 32)
    {
      v &= (1 << bits) - 1;
      if (v & (1 << (bits - 1)))
	v -= (1 << bits);
    }
  return v;
}

void
mem_set_content_type (int address, enum mem_content_type type)
{
  unsigned char *mt = rx_mem_ptr (address, MPA_CONTENT_TYPE);
  *mt = type;
}

void
mem_set_content_range (int start_address, int end_address, enum mem_content_type type)
{
  while (start_address < end_address)
    {
      int sz, ofs;
      unsigned char *mt;

      sz = end_address - start_address;
      ofs = start_address % L1_LEN;
      if (sz + ofs > L1_LEN)
	sz = L1_LEN - ofs;

      mt = rx_mem_ptr (start_address, MPA_CONTENT_TYPE);
      memset (mt, type, sz);

      start_address += sz;
    }
}

enum mem_content_type
mem_get_content_type (int address)
{
  unsigned char *mt = rx_mem_ptr (address, MPA_CONTENT_TYPE);
  return *mt;
}

#define IRADR(ir) (0x00087000 + (ir))

#define TCR8(ch)   (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 0])
#define TCSR8(ch)  (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 2])
#define TCORA8(ch) (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 4])
#define TCORB8(ch) (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 6])
#define TCNT8(ch)  (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 8])
#define TCCR8(ch)  (tmrbase[(((ch) / 2) * 0x10 + (ch) % 2 ) + 10])

#define TMRI_CMA (1<<2)
#define TMRI_CMB (1<<1)
#define TMRI_OVF (1<<0)

static unsigned char get_tmrisr(int ch)
{
  unsigned char *irptr = rx_mem_ptr(IRADR(174 + ch * 3), MPA_READING);
  unsigned char isr;
  unsigned int i;
  for (i = 0; i < 3; i++)
    {
      isr <<= 1;
      if (*irptr++)
	isr |= 1;
    }
  return isr;
}

static void set_tmrisr(int ch, unsigned char isr)
{
  unsigned char *irptr = rx_mem_ptr(IRADR(174 + ch * 3), MPA_WRITING);
  unsigned int i;
  for (i = 0; i < 3; i++)
    {
      *irptr++ = (isr & (1<<2))?1:0;
      isr <<= 1;
    }
}

static void 
tmr_update(int ch, unsigned int cycles_diff)
{
  static int prescale[]={1, 2, 8, 32, 64, 1024, 8192};
  const int prescale_div[]={1, 2, 8, 32, 64, 1024, 8192};
  static unsigned char isr[4]={0x00,0x00,0x00,0x00};
  int tm, cnt, pcnt, cor;
  unsigned char *tmrbase = rx_mem_ptr(0x00088200, MPA_WRITING);

  for (pcnt = 0; pcnt < 7; pcnt++)
    {
      prescale[pcnt] -= cycles_diff;
      
      if (prescale[pcnt]<=0) 
	{

	  /* input time pulse */
	  for(tm=0; tm < ch; tm++)
	    {
	      if ((TCCR8(tm) & 0x01f) == 0)	/* disable */
		continue;
	      /* internal isr clear */
	      isr[tm] &= get_tmrisr(tm);
	      
	      if ((TCCR8(tm & 2) & 0x18) == 0x18)
		{
		  /* 16bit mode */
		  if (tm & 1)
		    continue;
		  if ((TCCR8(tm + 1) & 0x07) != (pcnt - 1))
		    continue;
		  isr[tm + 1] &= get_tmrisr(tm + 1);
		  cnt = TCNT8(tm) << 8 | TCNT8(tm+1);
		  cnt++;
		  if (cnt >= 0x10000)
		    {
		      isr[tm] |= TMRI_OVF;
		      cnt = 0;
		    }
		  TCNT8(tm) = cnt >> 8;
		  TCNT8(tm-1) = cnt & 0xff;
		  /* TCORA compare match check */
		  cor = TCORA8(tm) << 8 | TCORA8(tm+1);
		  if (cnt >= cor)
		    {
		      isr[tm] |= TMRI_CMA;
		      if ((TCR8(tm) & 0x18) == 0x08)
			cnt = 0;
		    }
		  if ((cnt & 0xff) >= (cor & 0xff))
		    isr[tm+1] |= TMRI_CMA;
		  /* TCORB compare match check */
		  cor = TCORB8(tm) << 8 | TCORB8(tm+1);
		  if (cnt >= cor)
		    {
		      isr[tm] |= TMRI_CMB;
		      if ((TCR8(tm) & 0x18) == 0x10)
			cnt = 0;
		    }
		  if ((cnt & 0xff) >= (cor & 0xff))
		    isr[tm] |= TMRI_CMB;
		  TCNT8(tm) = cnt >> 8;
		  TCNT8(tm+1) = cnt & 0xff;
		  set_tmrisr(tm, isr[tm]);
		  set_tmrisr(tm + 1, isr[tm + 1]);
		}
	      else
		{
		  /* 8bit mode */
		  cnt = ++TCNT8(tm);
		  if (cnt>=0x100)
		    {
		      isr[tm] |= TMRI_OVF;
		      cnt = 0;
		    }
		  /* TCORA compare match check*/
		  if (cnt >= TCORA8(tm))
		    {
		      isr[tm] |= TMRI_CMA;
		      if ((TCR8(tm) & 0x18) == 0x08)
			cnt = 0;
		    }
		  /* TCORB compare match check*/
		  if (cnt >= TCORB8(tm))
		    {
		      isr[tm] |= TMRI_CMB;
		      if ((TCR8(tm) & 0x18) == 0x10)
			cnt = 0;
		    }
		  TCNT8(tm) = cnt;
		  set_tmrisr(tm, isr[tm]);
		}
	    }
	  prescale[pcnt]+=prescale_div[pcnt];
	}
    }
}

#define CMSTR(ch) (cmtbase[((ch) / 2 * 0x10 / 2)])
#define CMCR(ch)  (cmtbase[((ch) / 2 * 0x10 + ((ch) % 2 * 0x06) + 0x02) / 2])
#define CMCNT(ch) (cmtbase[((ch) / 2 * 0x10 + ((ch) % 2 * 0x06) + 0x04) / 2])
#define CMCOR(ch) (cmtbase[((ch) / 2 * 0x10 + ((ch) % 2 * 0x06) + 0x06) / 2])

void 
cmt_update(int cycles_diff)
{
  static int prescale[]={8, 32, 128, 512};
  const int prescale_div[]={8, 32, 128, 512};
  unsigned short *cmtbase = (unsigned short *)rx_mem_ptr(0x00088000, 
							 MPA_WRITING);
  int pcnt, tm, cnt;

  for (pcnt = 0; pcnt < 4; pcnt++)
    {
      prescale[pcnt] -= cycles_diff;
      
      if (prescale[pcnt]<=0) 
	{
	  /* input time pulse */
	  for(tm=0; tm < 4; tm++)
	    {
	      if (!(CMSTR(tm) & (1 << (tm % 2))))
		continue;
	      if ((CMCR(tm) & 3) != pcnt)
		continue;

	      cnt = ++CMCNT(tm);
	      if (cnt >= CMCOR(tm))
		{
		  if (CMCR(tm) & 0x40)
		    *rx_mem_ptr(IRADR(28 +tm), MPA_WRITING) = 1;
		  cnt = 0;
		}
	      CMCNT(tm) = cnt;
	    }
	  prescale[pcnt]+=prescale_div[pcnt];
	}
    }
}

#define SMR(ch) (scibase[(ch) * 8 + 0])
#define BRR(ch) (scibase[(ch) * 8 + 1])
#define SCR(ch) (scibase[(ch) * 8 + 2])
#define TDR(ch) (scibase[(ch) * 8 + 3])
#define SSR(ch) (scibase[(ch) * 8 + 4])
#define RDR(ch) (scibase[(ch) * 8 + 5])

#define SCI_ERI (1<<3)
#define SCI_RXI (1<<2)
#define SCI_TXI (1<<1)
#define SCI_TEI (1<<0)

#define MAX_SCI_CH 3

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

static unsigned char get_sciir(int ch)
{
  unsigned char *irptr = rx_mem_ptr(IRADR(214 + ch * 4), MPA_WRITING);
  unsigned char isr = 0;
  unsigned int i;
  for (i = 0; i < 4; i++)
    {
      isr <<= 1;
      if (*irptr++)
	isr |= 1;
    }
  return isr;
}

static void set_sciir(int ch, unsigned char isr)
{
  unsigned char *irptr = rx_mem_ptr(IRADR(214 + ch * 4), MPA_WRITING);
  unsigned int i;
  for (i = 0; i < 4; i++)
    {
      *irptr++ = (isr & (1<<3))?1:0;
      isr <<= 1;
    }
}

static unsigned int 
sci_complete_time(unsigned char *scibase, int ch)
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
  if (sci_port[ch].fd > 0) {
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
  if (sci_port[ch].fd > 0)
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

void
sci(unsigned int cycles_diff)
{
  static struct {
    int tx_end_time;
    int rx_end_time;
    int txstate;
    unsigned char isr;
    unsigned char ssr;
  } sci_state[MAX_SCI_CH] = {
    { .ssr = 0x84 },
    { .ssr = 0x84 },
    { .ssr = 0x84 },
  }, *st;
  unsigned char *scibase;
  int data;
  int ch;

  if (sci_port_type == PORT_NET && net_accept())
    return;

  scibase = rx_mem_ptr(0x00088240, MPA_WRITING);

  for (ch = 0; ch < MAX_SCI_CH; ch++)
    {
      st = &sci_state[ch];
      /* clear internal ssr */
      st->ssr &= SSR(ch);
      st->isr &= get_sciir(ch);
      if (!(SCR(ch) & 0x80))
	st->isr &= ~SCI_TXI;
      if (!(SCR(ch) & 0x40))
	st->isr &= ~(SCI_RXI|SCI_ERI);
      if (!(SCR(ch) & 0x4))
	st->isr &= ~SCI_TEI;

      /* Tx request */
      if((SCR(ch) & 0x20) && 
	 (get_rw_flag(0x00088243 + ch * 8) == mem_w) && (st->txstate == 0))
	{
	  sci_send_data(ch,TDR(ch));
	  st->isr &= ~SCI_TEI;
	  st->ssr &= ~0x04;
	  /* TSR shift time */
	  st->tx_end_time = 1;
	  st->txstate = 1;
	  set_rw_flag(0x00088243 + ch * 8, mem_none);
	}
      st->tx_end_time -= cycles_diff;
      /* Tx complete check */
      if(((st->isr & (SCI_TXI|SCI_TEI)) != (SCI_TXI|SCI_TEI)) &&
	 (st->tx_end_time <= 0))
	{
	  if (!(st->isr & SCI_TXI))
	    {
	      st->isr |= SCI_TXI;
	      st->ssr |= 0x80;
	      st->tx_end_time = sci_complete_time(scibase, ch);
	      st->txstate = 0;
	    } 
	  else {
	    st->isr |= SCI_TEI; /* All data transmit done */
	    st->ssr |= 0x04;
	  }
	}
      st->rx_end_time -= cycles_diff;
      /* Rx check */
      if (st->rx_end_time <= 0)
	/* RSR free & Rx Enabled */
	if ((SCR(ch) & 0x10) && sci_rcv_data(ch, &data))
	  {
	    /* Rx Overrun */
	    if(get_rw_flag(0x00088245 + ch * 8) == mem_none)
	      {
		st->isr |= SCI_ERI;
		set_rw_flag(0x00088245 + ch * 8, mem_none);
		st->ssr |= 0x20;
	      }
	    else
	  /* Rx ok */
	      {
		RDR(ch)=data;
		st->isr |= SCI_RXI;
		set_rw_flag(0x00088245 + ch * 8, mem_none);
		st->ssr |= 0x40;
	      }
	    /* RSR shift time */
	    st->rx_end_time = sci_complete_time(scibase, ch);
	  }

      /* update SSR */
      SSR(ch) = st->ssr & 0xfc; 
      set_sciir(ch, st->isr);
    }
}

static const unsigned char iprmap[] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x00, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0x02,
  0xff, 0xff, 0xff, 0xff, 0x04, 0x05, 0x06, 0x07,

  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
  0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

  0x40, 0xff, 0x44, 0x45, 0x46, 0x47, 0xff, 0xff,
  0x4c, 0x4c, 0x4c, 0x4c, 0x4d, 0xff, 0xff, 0x4e,
  0x4e, 0xff, 0xff, 0x4f, 0x4f, 0x50, 0x50, 0xff,
  0x51, 0x51, 0x52, 0x52, 0x52, 0x52, 0x53, 0x54,

  0x54, 0xff, 0xff, 0x55, 0x55, 0x56, 0x56, 0xff,
  0x57, 0x57, 0x58, 0x58, 0x58, 0x58, 0x59, 0xff,
  0xff, 0x5a, 0x5a, 0xff, 0xff, 0x5b, 0x5b, 0x5c,
  0x5c, 0xff, 0x5d, 0x5d, 0x5e, 0x5e, 0x5e, 0x5e,

  0x5f, 0x60, 0x60, 0xff, 0xff, 0x61, 0x61, 0x62,
  0x62, 0xff, 0x63, 0x63, 0xff, 0xff, 0x68, 0x68,
  0x68, 0x69, 0x69, 0x69, 0x6a, 0x6a, 0x6a, 0x6b,
  0x6b, 0x6b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x70, 0x71,
  0x72, 0x73, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x80,
  0x80, 0x80, 0x81, 0x81, 0x81, 0x81, 0x82, 0x82,

  0x82, 0x82, 0x83, 0x83, 0x83, 0x83, 0x84, 0x84,
  0x84, 0x84, 0x85, 0x85, 0x85, 0x85, 0x86, 0x86,
  0x86, 0x86, 0xff, 0xff, 0xff, 0xff, 0x88, 0x89,
  0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0xff, 0xff,
};

int 
icu(int pri)
{
  unsigned char *ir = rx_mem_ptr(0x00087000, MPA_WRITING);
  unsigned char *ien = rx_mem_ptr(0x00087200, MPA_WRITING);
  unsigned char *ipr = rx_mem_ptr(0x00087300, MPA_WRITING);
  int irq;
  int maxpri = 0;
  int ackirq = -1;
  for (irq = 255; irq >= 0; irq--)
    {
      if (iprmap[irq] != 0xff && ir[irq] && 
	  (ien[irq >> 3] & (1 << (irq &7))) &&
	  ipr[iprmap[irq]] > pri && ipr[iprmap[irq]] >= maxpri)
	{
	  ackirq = irq;
	  maxpri = ipr[iprmap[irq]];
	}
    }
  return ackirq;
}

int io_simulation(cpupri)
{
  static unsigned int prev_icycle = 0;
  static unsigned int prev_pcycle = 0;
  static unsigned int pcycles = 0;
  unsigned int pcycle_diff = 0;
  unsigned int icycle_diff = 0;

  icycle_diff = (rx_cycles - prev_icycle);
  prev_icycle = rx_cycles;
  pcycles += icycle_diff / 2;
  pcycle_diff = pcycles - prev_pcycle;
  if (pcycle_diff == 0)
    return -1;
  prev_pcycle = pcycles;
  tmr_update(3, pcycle_diff);
  cmt_update(pcycle_diff);
  sci(pcycle_diff);
  return icu(cpupri);
}

void init_io(void)
{
  struct INITTABLE {
    unsigned long addr;
    unsigned int size;
    unsigned int data;
  };
  enum {byte, word, lword};
  static const struct INITTABLE rx_regs_init[] = {
    {0x00080010,lword,0x67ffffff},
    {0x00080014,lword,0xffffffff},
    {0x00080018,lword,0xffff0000},
    {0x00088006,word,0xffff},
    {0x0008800c,word,0xffff},
    {0x00088016,word,0xffff},
    {0x0008801c,word,0xffff},
    {0x00088203,byte,0x10},
    {0x00088204,word,0xffff},
    {0x00088206,word,0xffff},
    {0x00088213,byte,0x10},
    {0x00088214,word,0xffff},
    {0x00088216,word,0xffff},
    {0x00088241,byte,0xff},
    {0x00088244,byte,0x84},
    {0x00088245,byte,0x00},
    {0x00088246,byte,0xf2},
    {0x00088249,byte,0xff},
    {0x0008824c,byte,0x84},
    {0x0008824d,byte,0x00},
    {0x0008824e,byte,0xf2},
    {0x00088251,byte,0xff},
    {0x00088254,byte,0x84},
    {0x00088255,byte,0x00},
    {0x00088256,byte,0xf2},
    {0x00088259,byte,0xff},
    {0x0008825c,byte,0x84},
    {0x0008825d,byte,0x00},
    {0x0008825e,byte,0xf2},
    {0x00088261,byte,0xff},
    {0x00088264,byte,0x84},
    {0x00088265,byte,0x00},
    {0x00088266,byte,0xf2},
    {0x00088269,byte,0xff},
    {0x0008826c,byte,0x84},
    {0x0008826d,byte,0x00},
    {0x0008826e,byte,0xf2},
    {0x00088271,byte,0xff},
    {0x00088274,byte,0x84},
    {0x00088275,byte,0x00},
    {0x00088276,byte,0xf2},
  };

  int c;
  for (c = 0x00087000; c < 0x00087390; c++)
    mem_put_qi(c, 0x00, 0);
  for (c = 0x00088000; c < 0x00088020; c += 2)
    mem_put_hi(c, 0x0000, 0);
  for(c=0;c<sizeof(rx_regs_init)/sizeof(struct INITTABLE);c++)
    {
      switch(rx_regs_init[c].size)
	{
	case byte:
	  mem_put_qi(rx_regs_init[c].addr,rx_regs_init[c].data, 0);
	  break;
	case word:
	  mem_put_hi(rx_regs_init[c].addr,rx_regs_init[c].data, 0);
	  break;
	case lword:
	  mem_put_si(rx_regs_init[c].addr,rx_regs_init[c].data, 0);
	  break;
	}
    }
  for (c = 0; c < MAX_SCI_CH; c++)
    mem_get_qi(0x00088245 + (c * 8), 0);
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
    tcgetattr(sci_port[ch].fd,&attr);
    memcpy(&sci_port[ch].old_attr,&attr,sizeof(struct termios));
    attr.c_lflag&=~ICANON;
    attr.c_cc[VMIN]=0;
    attr.c_cc[VTIME]=0;
    tcsetattr(sci_port[ch].fd,TCSAFLUSH,&attr);
    return ptyname;
  } else {
    sci_port[ch].fd = -1;
    return NULL;
  }
}

void sci_open_pty(struct host_callback_struct *callback)
{
  int ch;
  char *pty;
  sci_port_type = PORT_PTY;
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

void
show_memmap(struct host_callback_struct *callback)
{
  int l1, l2, off;
  char buf[65];
  for (l1 = 0; l1 < L1_LEN; l1++)
    {
      if (ptr[l1])
	{
	  for (l2 = 0; l2 < L2_LEN; l2++)
	    {
	      if (ptr[l1][l2])
		{
		  for (off = 0; off < OFF_LEN; off += 64)
		    {
		      int col = 0;
		      for (col = 0; col < 64; col++)
			buf[col] = (ptr[l1][l2][off] == MC_UNINIT)?'U':'D';
		      buf[col] = '\0';
		      (*callback->printf_filtered) (callback, "%08x: %s\n",
						    (l1 << (32 - L1_BITS)) |
						    (l2 << OFF_BITS) |
						    off, buf);
		    }
		}
	    }
	}
    }
}

static const char *memtype_str[]={"RL","RP","RW","RB","WL","WP","WW","WB"};

void show_memlog(struct host_callback_struct *callback, int lines)
{
  unsigned long idx;
  idx = memlogtail - lines;
  for (; lines > 0; --lines)
    {
      (*callback->printf_filtered) (callback,
					"0x%08x 0x%08x %s %08x\n", 
					memlog_buffer[idx].mpc,
					memlog_buffer[idx].addr,
					memtype_str[memlog_buffer[idx].type],
					memlog_buffer[idx].data);
      idx++;
    }
  (*callback->printf_filtered) (callback, "\n");
}

void save_memlog(struct host_callback_struct *callback, char *filename)
{
  FILE *fp;
  unsigned long idx;
  fp = fopen(filename, "w");
  if (!fp) {
      (*callback->printf_filtered) (callback, 
					"save-history: file open failed.\n");
      return ;
  }
  for (idx = 0; idx < memlogtail; idx++)
   fprintf(fp, "0x%08lx 0x%08lx %s %08lx\n", 
	   memlog_buffer[idx].mpc,
	   memlog_buffer[idx].addr,
	   memtype_str[memlog_buffer[idx].type],
	   memlog_buffer[idx].data);
  fclose(fp);
}

