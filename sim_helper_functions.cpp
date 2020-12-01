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

#ifndef __SIM_HELPER_FUNCTIONS_CPP__
#define __SIM_HELPER_FUNCTIONS_CPP__

#ifndef PI
  #define PI 3.1415926535897
#endif


/*------------------- SIMPLE HELPER FUNCTIONS ---------------------------*/


// returns random number between 0 and 1
float rand_f()
{
  if(seeded_the_rand == 0)
  {
    srand(time(NULL));
    seeded_the_rand = 1;
  }

  return (float)rand() / (float)RAND_MAX;
}


// returns a random float between min and max
float rand_f_between(float min, float max)
{
  return min + rand_f() * (max - min);
}

// returns the current time in double format
double get_curr_time()
{
  struct timespec now_time;
  const int gettime_rv = clock_gettime(CLOCK_REALTIME, &now_time);

  return ((double)now_time.tv_sec)+((double)now_time.tv_nsec/1000000000.0);
}

/*------------------- DISTANCE HELPER FUNCTIONS ---------------------------*/

// returns the distance between the center's of two robots
float dist_robots(ROBOT* robot_a, ROBOT* robot_b)
{
  return sqrtf( (robot_a->position[0] - robot_b->position[0])*(robot_a->position[0] - robot_b->position[0]) + (robot_a->position[1] - robot_b->position[1])*(robot_a->position[1] - robot_b->position[1]));
}

// returns the distance from the robot to the point x_pos y pos
float dist_robot_to_point(ROBOT* robot, float x_pos, float y_pos)
{
  return sqrtf( (robot->position[0] - x_pos)*(robot->position[0] - x_pos) + (robot->position[1] - y_pos)*(robot->position[1] - y_pos));
}












#endif //__SIM_HELPER_FUNCTIONS_CPP__


