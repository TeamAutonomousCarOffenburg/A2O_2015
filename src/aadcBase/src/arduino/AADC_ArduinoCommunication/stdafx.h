/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-19 13:12:07#$ $Rev:: 35171   $
**********************************************************************/

#ifndef _STDAFX_HEADER
#define _STDAFX_HEADER


#ifndef YOURHEADER_INCLUSION_GUARD
#define YOURHEADER_INCLUSION_GUARD

    #ifdef WIN32
    #    ifdef GetObject
    #        define MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED
    #    endif
    #    undef GetObject
    #endif

    #ifndef WIN32
        #include <sys/ioctl.h>
        #include <unistd.h>     // UNIX standard function definitions
        #include <termios.h>    // POSIX terminal control definitions
        #include <strings.h>    //linuxlib for bzero
        #include <fcntl.h>      // File control definitions
        #define BAUDRATE B115200
        #define _POSIX_SOURCE 1
    #else
        #define WIN32_LEAN_AND_MEAN
        #include <windows.h>
        #include <windowsx.h>
        #include <commctrl.h>
        #include <stdlib.h>    //lib for exit command
        #define SPM_BYTESTOREAD        WM_USER + 0x11
    #endif

    #ifdef WIN32
    #    if defined(MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED)
    #        undef MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED
    #        define GetObject GetObjectA
    #    endif
    #endif

#endif //YOURHEADER_INCLUSION_GUARD

// ADTF includes
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

// AADC includes
#include "arduinoProtocol.h"

#endif // _STDAFX_HEADER

