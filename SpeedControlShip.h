#ifndef _Speed_Control_Ship_H
#define _Speed_Control_Ship_H

  #ifndef _Control_Command_H
  #define _Control_Command_H
      #define   Astern        1
      #define   Ahead         0 
      #define   FullSpeed     255
      #define   HalfSpeed     125
      #define   SlowSpeed     10
      #define   Stop          0               //Case shutDown exe via other sing
      #define   StandBy       0 
      #define   AsternOutputPin      3        //PIN 9
      #define   ForwardOutputPin     11       //PIN10
      #define            F_EN       6
      #define            R_EN       5
      int ShipCommandProcess(float PotentiometerValue1);
  #endif

 
#endif