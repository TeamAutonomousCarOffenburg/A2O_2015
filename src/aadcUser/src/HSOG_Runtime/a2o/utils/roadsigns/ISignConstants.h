#pragma once


namespace A2O {

enum SignType {
  /** An unknown or invalid sign. */
  Unknown = 0,
  
  /** StVO 205 "Vorfahrt Gewaehren" */
  GiveWay = 491,

  /** StVO 206 "Stop" */
  Stop = 140,

  /** StVO 314 "Parken" */
  Parking = 484,

  /** StVO 102 "Kreuzung/Einmuendung mit Vorfahrt von rechts" */
  Crossing = 466,

  /** StVO 209-30 "Vorgeschriebene Fahrtrichtung - geradeaus" */
  ForceStraight = 166,

  /** StVO 215 "Kreisverkehr" */
  Roundabout = 46,

  /** StVO 276 "Ueberholverbot fuer alle Fahrzeuge" */
  NoOvertaking = 340,

  /** StVO 350 "Fussgaengerueberweg" */
  PedestrianCrossing = 376,

  /** StVO 220 "Einbahnstrasse" */
  OneWayStreet = 82,

  /** StVO 301 "Vorfahrt" */
  PriorityLane = 371,

  /** StVO 267 "Verbot der Einfahrt*/
  NoEntry = 306
};

}
