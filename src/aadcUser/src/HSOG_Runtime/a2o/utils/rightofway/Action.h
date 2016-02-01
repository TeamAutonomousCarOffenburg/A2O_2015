#ifndef ACTION_H
#define ACTION_H

/*! Drive action */
typedef enum Action
{
  /*! Drive */
  Drive,
  
  /*! Stop then drive */
  StopThenDrive,
    
  /*! Do not drive (wait some time / e.g. give way) */
  Wait,
  
  /*! Not permitted (e.g. invalid drive destination) */
  NotPermitted
  
} Action_t;

#endif // ACTION_H
