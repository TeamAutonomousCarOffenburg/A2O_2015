/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-20 16:21:28#$ $Rev:: 35283   $
**********************************************************************/

#include "stdafx.h"
#include "cRawSerialDevice.h"

#ifndef WIN32
tResult Serial::cRawSerialDevice::SetRawMode()
{
    struct termios tTermiosOptions;
    if (m_nDev<0) RETURN_ERROR(ERR_DEVICE_NOT_READY);
    // get the options from the serial interface
    if (tcgetattr(m_nDev, &tTermiosOptions) == -1) {        
        RETURN_ERROR(ERR_DEVICE_IO);  
    }
    //set serial port to raw mode
    cfmakeraw(&tTermiosOptions);  
    tTermiosOptions.c_cflag |= CLOCAL;
    tTermiosOptions.c_cflag |= B115200 ;    
    
    tTermiosOptions.c_cc[VTIME] = 0;
    tTermiosOptions.c_cc[VMIN] = 7; // blocking until 7 charse are received i.e. length of header of arduino frame
    // send setting to serial port
    if (tcsetattr(m_nDev, TCSANOW, &tTermiosOptions) == -1) 
        RETURN_ERROR(ERR_DEVICE_IO); 
    
    RETURN_NOERROR;
}
#endif

tUInt32 Serial::cRawSerialDevice::BytesAvailable()
{    
    int bytesAvailable = 0;
#ifndef     WIN32
    if (m_nDev==-1) RETURN_ERROR(ERR_DEVICE_NOT_READY);
    ioctl(m_nDev, FIONREAD, &bytesAvailable);
#else
    DWORD status;
    COMSTAT comStat;
    ClearCommError((HWND)(m_hDevice),&status, &comStat);
    bytesAvailable = (tInt32)comStat.cbInQue;
#endif
    return static_cast<tUInt32>(bytesAvailable);
}



