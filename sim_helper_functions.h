/*
# The MIT License (MIT)
#
# Copyright (c) January, 2015, 2020 michael otte
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
*/

#ifndef __SIM_HELPER_FUNCTIONS_H__
#define __SIM_HELPER_FUNCTIONS_H__

#ifndef PI
  #define PI 3.1415926535897
#endif


/*------------------- SIMPLE HELPER FUNCTIONS ---------------------------*/


// returns random number between 0 and 1
float rand_f();


// returns a random float between min and max
float rand_f_between(float min, float max);

// returns the current time in double format
double get_curr_time();





/*------------------- DISTANCE HELPER FUNCTIONS ---------------------------*/

// returns the distance between the center's of two robots
float dist_robots(ROBOT* robot_a, ROBOT* robot_b);

// returns the distance from the robot to the point x_pos y pos
float dist_robot_to_point(ROBOT* robot, float x_pos, float y_pos);








#endif //__SIM_HELPER_FUNCTIONS_H__


