#ifndef CARPOSITIONS_H
#define CARPOSITIONS_H

/*! Car positions */
typedef struct CarPositions
{
  /*! Car coming from north */
  bool north;
  
  /*! Car coming from east */
  bool east;
  
  /*! Car coming from west */
  bool west;
  
} CarPositions_t;

#endif