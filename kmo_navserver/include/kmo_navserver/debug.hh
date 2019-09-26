/*
sensorstorage - a framework for storing and replaying sensor data

Copyright (C) 2007 jacobs_robotics

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Head developer: Kaustubh Pathak, k.pathak@jacobs-university.de

Paper mail:
Jacobs University Bremen gGmbH
Kaustubh Pathak
Research I
Postfach 750561
D-28725 Bremen

*/

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>

#ifdef __GNUC__
/* #define LOC  fprintf(stderr, "[in %s@line %d] %s:", \ */
/*                               __FILE__, __LINE__, __PRETTY_FUNCTION__); */
#define LOC  fprintf(stderr, "[in %s@line %d] %s: ", \
		     __FILE__, __LINE__, __FUNCTION__);
#else
#define LOC  fprintf(stderr, "[in %s@line %d] ", __FILE__, __LINE__); 
#endif //__GNUC__


#ifndef NO_ERR
#define ERR(...) do{LOC fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");} while(0)
#else
#define ERR(...) ;
#endif

#ifdef DEBUG
#define DBG(...) do{LOC fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");} while(0)
#define DBG_FUNC(...) do{__VA_ARGS__} while(0)
#else
#define DBG(...) do{;}while(0)
#define DBG_FUNC(...) do{;} while(0)
#endif //DEBUG

#define TEST(...) do{LOC fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n");}\
 while(0)

#endif //__DEBUG_H__
