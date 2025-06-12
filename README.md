# Crazyflie_traj_planning_w_lighthouse

THIS SCRIPT IS UNTESTED DUE TO TIME CONSTRAINTS OF THE COURSE PROJECT AND EQUIPMENT ISSUES SO IT MAY NOT WORK. USE AT YOUR OWN DISCRETION.

This is a trajectory planning algorithm that uses quintic polynomials to autonomously fly the Bitcraze Crazyflie from a set "home" position to a set "goal" position. These points are set in _main_ of the script. The program uses the [Bitcraze Lighthouse Base Station](https://store.bitcraze.io/products/lighthouse-v2-base-station) to determine the position of the Crazyflie, thus a lighthouse calibration file is needed. This file can be obtained by calibrating the lighthouses in the Crazyflie Client. The software sends the coordinated of the flight path to the Crazyflie using the [Crazyradio 2.0](https://www.bitcraze.io/products/crazyradio-2-0/), the URI of the crazyflie and be configured or obtained in the Crazyflie client. 
