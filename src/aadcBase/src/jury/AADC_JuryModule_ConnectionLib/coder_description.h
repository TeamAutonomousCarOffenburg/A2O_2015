/**
 *
 * Struct Definitions for the ADTF Message Bus to decode Messages
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: WEF27UG $
 * $Date: 2013-04-23 10:26:52 +0200 (Di, 23 Apr 2013) $
 * $Revision: 20263 $
 *
 * @remarks
 *
 */

#ifndef _HEADER_CODER_DESC_AS_CODE_
#define _HEADER_CODER_DESC_AS_CODE_

#define ADTF_DEFAULT_DDL_BEGIN \
"<?xml version=\"1.0\" encoding=\"iso-8859-1\" standalone=\"no\"?>" \
"<adtf:ddl xmlns:adtf=\"adtf\">" \
    "<header>" \
        "<language_version>2.00</language_version>" \
        "<author> AUDI Electronics Venture GmbH</author>" \
        "<date_creation>07.04.2010</date_creation>" \
        "<date_change> 07.04.2010</date_change>" \
        "<description> ADTF Common Description File</description>" \
    "</header>" \
    "<units>" \
        "<baseunit description=\"Fundamental unit for length\" name=\"Metre\" symbol=\"m\" />" \
        "<baseunit description=\"Fundamental unit for mass\" name=\"Kilogram\" symbol=\"kg\" />" \
        "<baseunit description=\"Fundamental unit for time\" name=\"Second\" symbol=\"s\" />" \
        "<baseunit description=\"Fundamental unit for electric current\" name=\"Ampere\" symbol=\"A\" />" \
        "<baseunit description=\"Fundamental unit for thermodynamic temperature\" name=\"Kelvin\" symbol=\"K\" />" \
        "<baseunit description=\"Fundamental unit for amount of substance\" name=\"Mole\" symbol=\"mol\" />" \
        "<baseunit description=\"Fundamental unit for luminous intensity\" name=\"Candela\" symbol=\"cd\" />" \
        "<baseunit description=\"Non-SI standard unit for angle\" name=\"Degree\" symbol=\"deg\" />" \
        "<baseunit description=\"Non-SI standard unit for angle\" name=\"Radiant\" symbol=\"rad\" />" \
        "<baseunit description=\"No SI, but needed for own unit definitions\" name=\"Unitless\" symbol=\"\" />" \
        "<baseunit description=\"No SI, but needed for no unit definitions\" name=\"nou\" symbol=\"\" />" \
        "<prefixes name=\"yotta\" power=\"24\" symbol=\"Y\" />" \
        "<prefixes name=\"zetta\" power=\"21\" symbol=\"Z\" />" \
        "<prefixes name=\"exa\" power=\"18\" symbol=\"E\" />" \
        "<prefixes name=\"peta\" power=\"15\" symbol=\"P\" />" \
        "<prefixes name=\"tera\" power=\"12\" symbol=\"T\" />" \
        "<prefixes name=\"giga\" power=\"9\" symbol=\"G\" />" \
        "<prefixes name=\"mega\" power=\"6\" symbol=\"M\" />" \
        "<prefixes name=\"kilo\" power=\"3\" symbol=\"k\" />" \
        "<prefixes name=\"hecto\" power=\"2\" symbol=\"h\" />" \
        "<prefixes name=\"deca\" power=\"1\" symbol=\"da\" />" \
        "<prefixes name=\"deci\" power=\"-1\" symbol=\"d\" />" \
        "<prefixes name=\"centi\" power=\"-2\" symbol=\"c\" />" \
        "<prefixes name=\"milli\" power=\"-3\" symbol=\"m\" />" \
        "<prefixes name=\"micro\" power=\"-6\" symbol=\"u\" />" \
        "<prefixes name=\"nano\" power=\"-9\" symbol=\"n\" />" \
        "<prefixes name=\"pico\" power=\"-12\" symbol=\"p\" />" \
        "<prefixes name=\"femto\" power=\"-15\" symbol=\"f\" />" \
        "<prefixes name=\"atto\" power=\"-18\" symbol=\"a\" />" \
        "<prefixes name=\"zepto\" power=\"-21\" symbol=\"z\" />" \
        "<prefixes name=\"yocto\" power=\"-24\" symbol=\"y\" />" \
    "</units>" \
    "<datatypes>" \
        "<datatype description=\"predefined ADTF tBool datatype\" name=\"tBool\" size=\"8\" />" \
        "<datatype description=\"predefined ADTF tChar datatype\" name=\"tChar\" size=\"8\" />" \
        "<datatype description=\"predefined ADTF tUInt8 datatype\" name=\"tUInt8\" size=\"8\" />" \
        "<datatype description=\"predefined ADTF tInt8 datatype\" name=\"tInt8\" size=\"8\" />" \
        "<datatype description=\"predefined ADTF tUInt16 datatype\" name=\"tUInt16\" size=\"16\" />" \
        "<datatype description=\"predefined ADTF tInt16 datatype\" name=\"tInt16\" size=\"16\" />" \
        "<datatype description=\"predefined ADTF tUInt32 datatype\" name=\"tUInt32\" size=\"32\" />" \
        "<datatype description=\"predefined ADTF tInt32 datatype\" name=\"tInt32\" size=\"32\" />" \
        "<datatype description=\"predefined ADTF tUInt64 datatype\" name=\"tUInt64\" size=\"64\" />" \
        "<datatype description=\"predefined ADTF tInt64 datatype\" name=\"tInt64\" size=\"64\" />" \
        "<datatype description=\"predefined ADTF tFloat32 datatype\" name=\"tFloat32\" size=\"32\" />" \
        "<datatype description=\"predefined ADTF tFloat64 datatype\" name=\"tFloat64\" size=\"64\" />" \
    "</datatypes>" \
    "<enums>" \
        "<enum name=\"tPixelFormat\" type=\"tInt16\">" \
            "<element name=\"PF_16BIT\" value=\"20\" />" \
            "<element name=\"PF_24BIT\" value=\"40\" />" \
            "<element name=\"PF_32BIT\" value=\"50\" />" \
            "<element name=\"PF_8BIT\" value=\"10\" />" \
            "<element name=\"PF_ABGR_4444\" value=\"29\" />" \
            "<element name=\"PF_ABGR_8888\" value=\"52\" />" \
            "<element name=\"PF_ARGB_8888\" value=\"51\" />" \
            "<element name=\"PF_BGRA_8888\" value=\"54\" />" \
            "<element name=\"PF_BGR_555\" value=\"34\" />" \
            "<element name=\"PF_BGR_565\" value=\"35\" />" \
            "<element name=\"PF_BGR_888\" value=\"46\" />" \
            "<element name=\"PF_CUSTOM\" value=\"1000\" />" \
            "<element name=\"PF_GREYSCALE_10\" value=\"21\" />" \
            "<element name=\"PF_GREYSCALE_12\" value=\"22\" />" \
            "<element name=\"PF_GREYSCALE_14\" value=\"23\" />" \
            "<element name=\"PF_GREYSCALE_16\" value=\"24\" />" \
            "<element name=\"PF_GREYSCALE_18\" value=\"41\" />" \
            "<element name=\"PF_GREYSCALE_20\" value=\"42\" />" \
            "<element name=\"PF_GREYSCALE_22\" value=\"43\" />" \
            "<element name=\"PF_GREYSCALE_24\" value=\"44\" />" \
            "<element name=\"PF_GREYSCALE_32\" value=\"55\" />" \
            "<element name=\"PF_GREYSCALE_8\" value=\"11\" />" \
            "<element name=\"PF_GREYSCALE_FLOAT32\" value=\"56\" />" \
            "<element name=\"PF_RGBA_4444\" value=\"28\" />" \
            "<element name=\"PF_RGBA_8888\" value=\"53\" />" \
            "<element name=\"PF_RGB_444\" value=\"25\" />" \
            "<element name=\"PF_RGB_555\" value=\"26\" />" \
            "<element name=\"PF_RGB_565\" value=\"27\" />" \
            "<element name=\"PF_RGB_8\" value=\"12\" />" \
            "<element name=\"PF_RGB_888\" value=\"45\" />" \
            "<element name=\"PF_RIII_10\" value=\"30\" />" \
            "<element name=\"PF_RIII_12\" value=\"31\" />" \
            "<element name=\"PF_RIII_14\" value=\"32\" />" \
            "<element name=\"PF_RIII_16\" value=\"33\" />" \
            "<element name=\"PF_UNKNOWN\" value=\"0\" />" \
            "<element name=\"PF_YUV420P_888\" value=\"60\" />" \
        "</enum>" \
        "<enum name=\"tMediaTypeMajor\" type=\"tUInt32\">" \
            "<element name=\"MEDIA_TYPE_AUDIO\" value=\"2048\" />" \
            "<element name=\"MEDIA_TYPE_CAN\" value=\"512\" />" \
            "<element name=\"MEDIA_TYPE_COMMAND\" value=\"2304\" />" \
            "<element name=\"MEDIA_TYPE_FLEXRAY\" value=\"1280\" />" \
            "<element name=\"MEDIA_TYPE_INFO\" value=\"1536\" />" \
            "<element name=\"MEDIA_TYPE_LIN\" value=\"513\" />" \
            "<element name=\"MEDIA_TYPE_MOST\" value=\"514\" />" \
            "<element name=\"MEDIA_TYPE_NETWORK_DATA\" value=\"8192\" />" \
            "<element name=\"MEDIA_TYPE_REFERENCE\" value=\"4096\" />" \
            "<element name=\"MEDIA_TYPE_STRUCTURED_DATA\" value=\"768\" />" \
            "<element name=\"MEDIA_TYPE_VIDEO\" value=\"256\" />" \
        "</enum>" \
    "</enums>" \
    "<structs>"

#define ADTF_DEFAULT_STRUCTS \
        "<!-- TYPE DESCRIPTIONS-->" \
        "<struct alignment=\"1\" name=\"tMediaTypeInfo\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"ui32MajorType\" type=\"tMediaTypeMajor\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"4\" name=\"ui32SubType\" type=\"tUInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"8\" name=\"ui32Flags\" type=\"tUInt32\" />" \
        "</struct>" \
        "<struct alignment=\"1\" name=\"adtf.core.media_type\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"mediatype\" type=\"tMediaTypeInfo\" />" \
        "</struct>" \
        "<struct alignment=\"1\" name=\"tBitmapFormat\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"nWidth\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"4\" name=\"nHeight\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"8\" name=\"nBitsPerPixel\" type=\"tInt16\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"10\" name=\"nPixelFormat\" type=\"tPixelFormat\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"12\" name=\"nBytesPerLine\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"16\" name=\"nSize\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"20\" name=\"nPaletteSize\" type=\"tInt32\" />" \
        "</struct>" \
        "<struct alignment=\"1\" name=\"tWaveFormat\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"nFormatType\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"4\" name=\"nChannels\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"8\" name=\"nSamplesPerSec\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"12\" name=\"nBitsPerSample\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"16\" name=\"nNumSamples\" type=\"tInt32\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"20\" name=\"nSize\" type=\"tInt32\" />" \
        "</struct>" \
        "<struct alignment=\"1\" name=\"adtf.type.video\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"sMediatype\" type=\"tMediaTypeInfo\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"12\" name=\"sBitmapFormat\" type=\"tBitmapFormat\" />" \
        "</struct>" \
        "<struct alignment=\"1\" name=\"adtf.type.audio\" version=\"1\">" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"sMediatype\" type=\"tMediaTypeInfo\" />" \
            "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"12\" name=\"sWaveFormat\" type=\"tWaveFormat\" />" \
        "</struct>"


#define ADTF_DEFAULT_STREAMS "<stream description=\"video stream\" name=\"audio_stream\" type=\"adtf.type.audio\" />" \
                               "<stream description=\"audio stream\" name=\"video_stream\" type=\"adtf.type.video\" />"


#define ADTF_DEFAULT_DDL_END "</adtf:ddl>"


#define CONNLIB_JURY_STRUCTS \
    "<struct alignment=\"1\" name=\"tJuryEmergencyStop\" version=\"1\">" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"bEmergencyStop\" type=\"tBool\" />" \
    "</struct>" \
    "<struct alignment=\"1\" name=\"tBoolSignalValue\" version=\"1\">" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"ui32ArduinoTimestamp\" type=\"tUInt32\" />" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"4\" name=\"bEmergencyStop\" type=\"tBool\" />" \
    "</struct>" \
    "<struct alignment=\"1\" name=\"tJuryStruct\" version=\"1\">" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"i8ActionID\" type=\"tInt8\" />" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"4\" name=\"i16ManeuverEntry\" type=\"tInt16\" />" \
    "</struct>" \
    "<struct alignment=\"1\" name=\"tManeuverList\" version=\"1\">" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"i32Size\" type=\"tInt32\" />" \
        "<element alignment=\"1\" arraysize=\"i32Size\" byteorder=\"LE\" bytepos=\"4\" name=\"aManeuverList\" type=\"tChar\" />" \
    "</struct>" \
    "<struct alignment=\"1\" name=\"tDriverStruct\" version=\"1\">" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"0\" name=\"i8StateID\" type=\"tInt8\" />" \
        "<element alignment=\"1\" arraysize=\"1\" byteorder=\"LE\" bytepos=\"1\" name=\"i16ManeuverEntry\" type=\"tInt16\" />" \
    "</struct>" 

#define CONNLIB_JURY_STREAMS \
    "<stream name=\"stream_tDriverStruct\" type=\"adtf.core.media_type\" >" \
        "<struct bytepos=\"0\" type=\"tDriverStruct\"/>"\
    "</stream>" \
    "<stream name=\"stream_tJuryStruct\" type=\"adtf.core.media_type\" >" \
        "<struct bytepos=\"0\" type=\"tJuryStruct\"/>"\
    "</stream>" \
    "<stream name=\"stream_tJuryEmergencyStop\" type=\"adtf.core.media_type\" >" \
        "<struct bytepos=\"0\" type=\"tJuryEmergencyStop\"/>"\
    "</stream>" \
    "<stream name=\"stream_tBoolSignalValue\" type=\"adtf.core.media_type\" >" \
        "<struct bytepos=\"0\" type=\"tBoolSignalValue\"/>"\
    "</stream>" \
        "<stream name=\"stream_tManeuverList\" type=\"adtf.core.media_type\" >" \
        "<struct bytepos=\"0\" type=\"tManeuverList\"/>"\
    "</stream>" \



#define CONNLIB_JURY_MODULE_DDL_AS_STRING \
ADTF_DEFAULT_DDL_BEGIN \
ADTF_DEFAULT_STRUCTS \
CONNLIB_JURY_STRUCTS \
"</structs>" \
"<streams>" \
ADTF_DEFAULT_STREAMS \
CONNLIB_JURY_STREAMS \
"</streams>" \
ADTF_DEFAULT_DDL_END

#pragma pack ( push, 1 )
typedef struct 
{
    tBool   bEmergencyStop;
} tJuryEmergencyStop;

typedef struct  
{
    tUInt32 ui32ArduinoTimeStamp;
    tBool   bValue;
} tBoolSignalValue;

typedef struct
{
    tInt8   i8ActionID;
    tInt16  i16ManeuverEntry;
} tJuryStruct;

typedef struct
{
    tInt8   i8StateID;
    tInt8   i16ManeuverEntry;
} tDriverStruct;


#pragma pack ( pop )

#endif //_HEADER_CODER_DESC_AS_CODE_