# This shell script emits a C file. -*- C -*-
# It does some substitutions.
fragment <<EOF
/* This file is is generated by a shell script.  DO NOT EDIT! */

/* Handle embedded relocs for m68k.
   Copyright (C) 2000-2014 Free Software Foundation, Inc.
   Written by Michael Sokolov <msokolov@ivan.Harhan.ORG>, based on generic.em
   by Steve Chamberlain <steve@cygnus.com>, embedded relocs code based on
   mipsecoff.em by Ian Lance Taylor <ian@cygnus.com> (now removed).

   This file is part of the GNU Binutils.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston,
   MA 02110-1301, USA.  */

#define TARGET_IS_${EMULATION_NAME}

#include "sysdep.h"
#include "bfd.h"
#include "bfdlink.h"

#include "ld.h"
#include "ldmain.h"
#include "ldexp.h"
#include "ldlang.h"
#include "ldfile.h"
#include "ldemul.h"
#include "ldmisc.h"

static void check_sections (bfd *, asection *, void *);

static void
gld${EMULATION_NAME}_before_parse (void)
{
#ifndef TARGET_			/* I.e., if not generic.  */
  ldfile_set_output_arch ("`echo ${ARCH}`", bfd_arch_unknown);
#endif /* not TARGET_ */
}

/* This function is run after all the input files have been opened.
   We create a .emreloc section for each input file with a non zero
   .data section.  The BFD backend will fill in these sections with
   magic numbers which can be used to relocate the data section at run
   time.  */

static void
gld${EMULATION_NAME}_after_open (void)
{
  bfd *abfd;

  after_open_default ();

  if (! command_line.embedded_relocs
      || link_info.relocatable)
    return;

  for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      asection *datasec;

      /* As first-order business, make sure that each input BFD is COFF. It
         better be, as we are directly calling a COFF backend function.  */
      if (bfd_get_flavour (abfd) != bfd_target_coff_flavour)
	einfo ("%F%B: all input objects must be COFF for --embedded-relocs\n");

      datasec = bfd_get_section_by_name (abfd, ".data");

      /* Note that we assume that the reloc_count field has already
         been set up.  We could call bfd_get_reloc_upper_bound, but
         that returns the size of a memory buffer rather than a reloc
         count.  We do not want to call bfd_canonicalize_reloc,
         because although it would always work it would force us to
         read in the relocs into BFD canonical form, which would waste
         a significant amount of time and memory.  */
      if (datasec != NULL && datasec->reloc_count > 0)
	{
	  asection *relsec;

	  relsec = bfd_make_section_with_flags (abfd, ".emreloc",
						(SEC_ALLOC
						 | SEC_LOAD
						 | SEC_HAS_CONTENTS
						 | SEC_IN_MEMORY));
	  if (relsec == NULL
	      || ! bfd_set_section_alignment (abfd, relsec, 2)
	      || ! bfd_set_section_size (abfd, relsec,
					 datasec->reloc_count * 12))
	    einfo ("%F%B: can not create .emreloc section: %E\n");
	}

      /* Double check that all other data sections are empty, as is
         required for embedded PIC code.  */
      bfd_map_over_sections (abfd, check_sections, datasec);
    }
}

/* Check that of the data sections, only the .data section has
   relocs.  This is called via bfd_map_over_sections.  */

static void
check_sections (bfd *abfd, asection *sec, void *datasec)
{
  if ((bfd_get_section_flags (abfd, sec) & SEC_DATA)
      && sec != datasec
      && sec->reloc_count != 0)
    einfo ("%B%X: section %s has relocs; can not use --embedded-relocs\n",
	   abfd, bfd_get_section_name (abfd, sec));
}

/* This function is called after the section sizes and offsets have
   been set.  If we are generating embedded relocs, it calls a special
   BFD backend routine to do the work.  */

static void
gld${EMULATION_NAME}_after_allocation (void)
{
  bfd *abfd;

  if (! command_line.embedded_relocs
      || link_info.relocatable)
    return;

  for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      asection *datasec, *relsec;
      char *errmsg;

      datasec = bfd_get_section_by_name (abfd, ".data");

      if (datasec == NULL || datasec->reloc_count == 0)
	continue;

      relsec = bfd_get_section_by_name (abfd, ".emreloc");
      ASSERT (relsec != NULL);

      if (! bfd_m68k_coff_create_embedded_relocs (abfd, &link_info,
						   datasec, relsec,
						   &errmsg))
	{
	  if (errmsg == NULL)
	    einfo ("%B%X: can not create runtime reloc information: %E\n",
		   abfd);
	  else
	    einfo ("%X%B: can not create runtime reloc information: %s\n",
		   abfd, errmsg);
	}
    }
}

static char *
gld${EMULATION_NAME}_get_script (int *isfile)
EOF

if test x"$COMPILE_IN" = xyes
then
# Scripts compiled in.

# sed commands to quote an ld script as a C string.
sc="-f stringify.sed"

fragment <<EOF
{
  *isfile = 0;

  if (link_info.relocatable && config.build_constructors)
    return
EOF
sed $sc ldscripts/${EMULATION_NAME}.xu                 >> e${EMULATION_NAME}.c
echo '  ; else if (link_info.relocatable) return'     >> e${EMULATION_NAME}.c
sed $sc ldscripts/${EMULATION_NAME}.xr                 >> e${EMULATION_NAME}.c
echo '  ; else if (!config.text_read_only) return'     >> e${EMULATION_NAME}.c
sed $sc ldscripts/${EMULATION_NAME}.xbn                >> e${EMULATION_NAME}.c
echo '  ; else if (!config.magic_demand_paged) return' >> e${EMULATION_NAME}.c
sed $sc ldscripts/${EMULATION_NAME}.xn                 >> e${EMULATION_NAME}.c
echo '  ; else return'                                 >> e${EMULATION_NAME}.c
sed $sc ldscripts/${EMULATION_NAME}.x                  >> e${EMULATION_NAME}.c
echo '; }'                                             >> e${EMULATION_NAME}.c

else
# Scripts read from the filesystem.

fragment <<EOF
{
  *isfile = 1;

  if (link_info.relocatable && config.build_constructors)
    return "ldscripts/${EMULATION_NAME}.xu";
  else if (link_info.relocatable)
    return "ldscripts/${EMULATION_NAME}.xr";
  else if (!config.text_read_only)
    return "ldscripts/${EMULATION_NAME}.xbn";
  else if (!config.magic_demand_paged)
    return "ldscripts/${EMULATION_NAME}.xn";
  else
    return "ldscripts/${EMULATION_NAME}.x";
}
EOF

fi

fragment <<EOF

struct ld_emulation_xfer_struct ld_${EMULATION_NAME}_emulation =
{
  gld${EMULATION_NAME}_before_parse,
  syslib_default,
  hll_default,
  after_parse_default,
  gld${EMULATION_NAME}_after_open,
  gld${EMULATION_NAME}_after_allocation,
  set_output_arch_default,
  ldemul_default_target,
  before_allocation_default,
  gld${EMULATION_NAME}_get_script,
  "${EMULATION_NAME}",
  "${OUTPUT_FORMAT}",
  finish_default,
  NULL,	/* create output section statements */
  NULL,	/* open dynamic archive */
  NULL,	/* place orphan */
  NULL,	/* set symbols */
  NULL,	/* parse args */
  NULL,	/* add_options */
  NULL,	/* handle_option */
  NULL,	/* unrecognized file */
  NULL,	/* list options */
  NULL,	/* recognized file */
  NULL,	/* find_potential_libraries */
  NULL,	/* new_vers_pattern */
  NULL	/* extra_map_file_text */
};
EOF
