/**
 *
 * Standard includes
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: VTW8PSR $
 * $Date: 2012-07-24 13:44:52 +0200 (Di, 24 Jul 2012) $
 * $Revision: 14167 $
 *
 */

#ifndef _STD_INCLUDES_H_
#define _STD_INCLUDES_H_

#include <connectionlib.h>
using namespace connectionlib;

struct tAADC_Maneuver
{
    int nId;
    std::string action;
};

struct tSector
{
    int id;
    std::vector<tAADC_Maneuver> lstManeuvers;
};


#endif

