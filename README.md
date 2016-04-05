# GeoSol
### Firmware for ARM microcontroller to solve Geodetic problems
This program is created to automatize and speed-up solving of geodetic problems on real objects. 

Three problems will be solved: Direct geodetic problem, Inverse geodetic problem, Polar serif problem

Microcontroller reads data from GPS-module and potentiometers to use it as an input in geodetic problems. Output is viewed on small screen and can be exported to SD-card.

##Hardware  
-<a href="https://developer.mbed.org/platforms/IBMEthernetKit/?cm_mc_uid=15423806122714366318128&cm_mc_sid_50200000=1459858340">ARM® Cortex™-M4 Core Starter Kit</a> 

-<a href="http://wiki.iteadstudio.com/Arduino_GPS_shield">GPS module</a>. 

##Direct geodetic problem
Having coordinate of one point, polar angle from this point to another point and distance between them we are able to find coordinates of second point.

##Inverse geodetic problem 
Having coordinates of two points we can calculate distance and polar angle between them.

##Polar serif problem 
Given three points(P1,P2,P3) on the plane. We know coordinates of first two, polar angle between two lines which connect P1 and P2, P1 and P3 and distance from P1 to P3. Our goal is to find coordinates of point P3.
