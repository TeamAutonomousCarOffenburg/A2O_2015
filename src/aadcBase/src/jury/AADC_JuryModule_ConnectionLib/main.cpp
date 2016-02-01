/**
 *
 * Implementing a very simple example to show the creation and general usage
 * of a system in terms of the connection library.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: VTW8PSR $
 * $Date: 2012-08-10 14:53:29 +0200 (Fr, 10 Aug 2012) $
 * $Revision: 14649 $
 *
 */

#include <math.h>
#include <string.h> //memset under Linux
#include <iostream>
#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif
#include "stdafx.h"     //necessary includes and use namespace connectionlib

#include "./coder_description.h"
#include "cJuryModule.h"
#include <QApplication>






int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    cJuryModule oJuryModule;

    oJuryModule.show();
    return a.exec();
    
}
