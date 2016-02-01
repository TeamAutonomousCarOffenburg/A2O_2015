/**
 *
 * @todo
 *
 * @file 
 * Copyright&copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: belkera $
 * $Date: 2011-06-30 16:51:21 +0200 (Do, 30 Jun 2011) $
 * $Revision: 26514 $
 *
 */
#ifndef _TEMPLATE_PROJECT_DATA_H_
#define _TEMPLATE_PROJECT_DATA_H_

// Our example data consists of a single tFloat64 value
typedef tFloat64 tTemplateData;

/*
 * Major-type of our template data type.
 *
 * In order to be able to distinguish our data we define custom media type identifiers.
 * Please note that in this example we do not require any additional metadata, so we use 
 * an "ordinary" OID_ADTF_MEDIA_TYPE media type with the following major- and subtype. If you want to pass on
 * additional meta data please have a look at the demo_versioned_media_type example.
 */ 
#define MEDIA_TYPE_TEMPLATE     0x00040677  

/*
 * Sub-type of our template data type.
 */
#define MEDIA_SUBTYPE_TEMPLATE  0x00040677  

#endif
