/* This testcase is part of GDB, the GNU debugger.

   Copyright 2013-2015 Free Software Foundation, Inc.

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

#if 0
# define HEADER 1
#endif
#if 1
# undef HEADER
# define HEADER 2
void header_two_func (void) {}
#endif
#if 0
/* #include "header.h" does not work, why?  */
# include <header.h>
#endif
