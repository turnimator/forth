Forthmobile-june-3v.ino

This file has :

  X("initcompass",setupcomp2, setupcomp()) \
  X("readcompass",readcomp2, PUSH readcomp())  \
  X("initlaser", setupe, setuplaser()) \
  X("readlaser",sotape, PUSH readlaser())  \


you need before using initialize each I2C peripheral
if not they will not work, or crash the system.

oledinit     ( initialize oled) 
initcompass  
initlaser

readcompass will only return  the X magnetometer value 
Y & Z are not needed in my application.

readcompass  ( --X) 
readlaser    ( --dist)

Hope it works for you.

it will work without problems only if you 
have  1 TOF laser , 1 Magnetometer and 1 Oled on the I2c
bus.

 
