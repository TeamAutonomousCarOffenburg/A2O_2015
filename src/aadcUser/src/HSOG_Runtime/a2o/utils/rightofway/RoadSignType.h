#ifndef ROADSIGNTYPE_H
#define ROADSIGNTYPE_H

// Type of road sign
typedef enum RoadSignType
{
    /*! No road sign */
    NoRoadSign,
    
    /*! StVO205 "Vorfahrt gewähren" */
    StVO205RoadSign,
    
    /*! StVO206 "Stoppschild" */
    StVO206RoadSign,
    
    /*! StVO102 "Kreuzung mit Vorfahrt von rechts" */
    StVO102RoadSign,
    
    /*! StVO301 "Vorfahrt" */
    StVO301RoadSign,
    
    /*! StVO20930 "Vorgeschriebene Fahrtrichtung geradeaus" */
    StVO20930RoadSign
    
} RoadSignType_t;

#endif
