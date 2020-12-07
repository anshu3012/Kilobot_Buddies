/*
# The MIT License (MIT)
#
# Copyright (c) January, 2015 michael otte
# Copyright (c) April, 2020 michael otte, University of Maryland
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



#ifndef __SIM_DATA_STRUCTURES_CPP__
#define __SIM_DATA_STRUCTURES_CPP__

#include <stdarg.h>
#include <stdio.h>


#include "sim_helper_functions.cpp"
#include "read_bitmap.c"
#include <vector>

/*----------------------- ROBOT DATA STRUCTURE --------------------------*/


// this creates and returns a pointer to a ROBOT struct
ROBOT* make_robot(float x_pose, float y_pose, float theta_pose, float radius, float com_radius, float* color, int id, float x_pose_min, float x_pose_max, float y_pose_min, float y_pose_max)
{
  ROBOT* bot = (ROBOT*)calloc(1, sizeof(ROBOT));
  
  bot->radius = radius;
  bot->com_radius = com_radius;

  bot->position[0] = x_pose;
  bot->position[1] = y_pose;
  bot->position[2] = 0.0f;
 
  bot->orientation[0] = theta_pose;

  bot->speed[0] = 0.0f;
  bot->speed[1] = 0.0f;
  bot->speed[2] = 0.0f;

  bot->d_orientation[0] = 0.0f;

  bot->motors_spun_up = 0;

  bot->color[0] = color[0];
  bot->color[1] = color[1];
  bot->color[2] = color[2];
  
  bot->id = id;
  bot->code_space = None; // set to default later

  bot->num_neighbors = 0;
  bot->neighbors = NULL;

  bot->inter_message_time = 1.0f;
  bot->message_time = 0.0f;
  bot->next_message_send_at = 1.0f;

  bot->sending = 0;
  bot->message_collision = 0;
  bot->last_message_sent = (message_t*)calloc(1, sizeof(message_t));

  init_distance_measurement(&(bot->incomming_distance_measurement));

  bot->pose_min[0] = x_pose_min;
  bot->pose_min[1] = y_pose_min;
  bot->pose_max[0] = x_pose_max;
  bot->pose_max[1] = y_pose_max;

  bot->start_time = get_curr_time();
  bot->time_since_start = 0.0;
  bot->time_since_last_time_update = 0.0;
  bot->need_to_run_loop = 0;

  bot->visual_light_input = 0.0f;

  bot->kilo_uid = (uint16_t)id; 
  bot->kilo_ticks = (uint32_t)0;

  bot->send_message_type = -1;

  bot->kilolib_setup = 0;
  bot->led_color[0] = 0.0f;
  bot->led_color[1] = 0.0f;
  bot->led_color[2] = 0.0f;
  bot->led_color[3] = 1.0f;

  bot->ambient_light = 0;

  bot->kilo_message_rx = message_rx_dummy;
  bot->kilo_message_tx = message_tx_dummy;
  bot->kilo_message_tx_success = message_tx_success_dummy;
  bot->kilo_loop = NULL;

  return bot;
}



// this deallocates all required memory for a robot
void destroy_robot(ROBOT* bot)
{
  if(bot->neighbors != NULL)
  {
    free(bot->neighbors);
  }

  if(bot->last_message_sent != NULL)
  {
    free(bot->last_message_sent);
  }

  if(bot != NULL)
  {
    free(bot);
  }
}



// generates a random robot within the [x, y] position bounds, and with the 
// prescribed radius and comm range, nore that color is randomly chosen
ROBOT* random_robot(float* pose_min, float* pose_max, float radius, float com_radius, int id, float x_pose_min, float x_pose_max, float y_pose_min, float y_pose_max)
{
  float x_pose = rand_f_between(pose_min[0], pose_max[0]);
  float y_pose = rand_f_between(pose_min[1], pose_max[1]);
  //float theta_pose = rand_f_between(0.0f, 2.0f * PI); //uncomment if you need random pose 

  float theta_pose=0.5*PI;


  float rand_color[4];
  int i;
  for(i = 0; i < 3; i++)
  {
    rand_color[i] = rand_f_between(0.5, 1.0);
  }

  return make_robot(x_pose, y_pose, theta_pose, radius, com_radius, rand_color, id, x_pose_min, x_pose_max, y_pose_min, y_pose_max);
}

// generates a robot at the prescribed location, and with the 
// prescribed radius and comm range, nore that color is randomly chosen
ROBOT* new_robot(float x_pose, float y_pose, float theta_pose, float radius, float com_radius, int id, float x_pose_min, float x_pose_max, float y_pose_min, float y_pose_max)
{
  float rand_color[4];
  int i;
  for(i = 0; i < 3; i++)
  {
    rand_color[i] = rand_f_between(0.5, 1.0);
  }

  return make_robot(x_pose, y_pose, theta_pose, radius, com_radius, rand_color, id, x_pose_min, x_pose_max, y_pose_min, y_pose_max);
}


// drive robot
// updates the position of the robot given motor speeds, etc.
void drive_robot_for_duration(ROBOT* bot, float dt)
{
 
  float time_resolution = 0.01;   // time resolution
  int last_loop = 0;
  float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
  float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
  // note that bot->speed is the current ratio of max alowed speed

  if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
  {
    // robot is stopped

    bot->d_orientation[0] = 0.0f;
    return;
  }

  // assuming tank steering is a good enough approximation
  float x = bot->position[0];
  float y = bot->position[1];
  float theta = bot->orientation[0];


  // if robot is moving straight
  if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  {
    float speed = ((bot->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;

    float dx = dt*cos(theta)*speed;
    float dy = dt*sin(theta)*speed;

    bot->position[0] += dx;
    bot->position[1] += dy;

    bot->d_orientation[0] = 0.0f;
    return; 
  }
    
  // otherwise robot is turning

  float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
  float v_r = bot->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

  float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
  float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

  float c_x = x - R*sin(theta); // center of rotation x
  float c_y = y + R*cos(theta); // center of rotation y

  x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
  y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
  theta = theta + omega*dt;

  while(theta > 2.0f*PI)
  {
    theta -= 2.0f*PI;
  }
  while(theta < -2.0f*PI)
  {
    theta += 2.0f*PI;
  }


  bot->position[0] = x;
  bot->position[1] = y;
  bot->orientation[0] = theta;

  bot->d_orientation[0] = omega;
}


void run_full_loop(ROBOT* bot)
{
  // swap this robot into global space
  now_computing_with(bot);

  // do the computation loop for this robot
  (*(bot->kilo_loop))();  // run one loop

  // save any values that have changed 
  done_computing_with(bot);
}

/*----------------------- SWARM DATA STRUCTURE --------------------------*/



// makes a swarm of randomly placed robots
SWARM* make_rand_swarm(int num_robots, float x_pose_min, float x_pose_max, float y_pose_min, float y_pose_max, float robot_radius, float com_radius)
{
  SWARM* swarm = (SWARM*)calloc(1,sizeof(SWARM));
 
  swarm->pose_min[0] = x_pose_min;
  swarm->pose_min[1] = y_pose_min;

  swarm->pose_max[0] = x_pose_max;
  swarm->pose_max[1] = y_pose_max;

  swarm->robot_radius = robot_radius;
  swarm->com_radius = com_radius;

  swarm->num_robots = num_robots;
  swarm->robots = (ROBOT**)calloc(num_robots,sizeof(ROBOT*));
  
  int i; 
  int j;


  float rand_color[4];
  int k;
  for(k = 0; k < 3; k++)
  {
    rand_color[i] = 0.1;
  }

  

  for (int i = 0, j = num_robots/2; i < (num_robots/2) && j < num_robots; i++, j++)
  {
    swarm->robots[i] = random_robot(swarm->pose_min, swarm->pose_max, robot_radius, com_radius, i, x_pose_min, x_pose_max, y_pose_min, y_pose_max);
    swarm->robots[j] = make_robot((swarm->robots[i]->position[0])+2.2*robot_radius, (swarm->robots[i]->position[1]), swarm->robots[i]->orientation[0], robot_radius, com_radius, rand_color, j, x_pose_min, x_pose_max, y_pose_min, y_pose_max);
    swarm->vect.push_back(std::make_pair(swarm->robots[i]->id,swarm->robots[j]->id));
  }

  swarm->next_event = 0.0;
  swarm->total_dropped = 0;
  swarm->total_recieved = 0;
  swarm->total_backoffs = 0;

  return swarm;
}
  

// makes a swarm of robots placed at the particular locations prescribed in the 
// pose vectors
SWARM* make_new_swarm(int num_robots, float x_pose_min, float x_pose_max, float y_pose_min, float y_pose_max,  float* x_poses, float* y_poses, float* theta_poses, float robot_radius, float com_radius)
{
  SWARM* swarm = (SWARM*)calloc(1,sizeof(SWARM));
 
  swarm->pose_min[0] = x_pose_min;
  swarm->pose_min[1] = y_pose_min;

  swarm->pose_max[0] = x_pose_max;
  swarm->pose_max[1] = y_pose_max;

  swarm->robot_radius = robot_radius;
  swarm->com_radius = com_radius;

  swarm->num_robots = num_robots;
  swarm->robots = (ROBOT**)calloc(num_robots,sizeof(ROBOT*));
  
  int i; 
  for(i = 0; i < num_robots; i++)
  {
    swarm->robots[i] = new_robot(x_poses[i], y_poses[i], theta_poses[i], robot_radius, com_radius, i, x_pose_min, x_pose_max, y_pose_min, y_pose_max);
  }

  swarm->next_event = 0.0;

  swarm->total_dropped = 0;
  swarm->total_recieved = 0;
  swarm->total_backoffs = 0;

  return swarm;
}



// this deallocates all required memory for a swarm
void destroy_swarm(SWARM* swarm)
{  
  if(swarm == NULL)
  {
    return;
  }

  int i;
  for(i = 0; i < swarm->num_robots; i++)
  {
    destroy_robot(swarm->robots[i]);
  }

  free(swarm->robots);
  free(swarm);
}




void make_swarm_non_overlapping(SWARM* swarm, int tries_per_robot)
{
  int i,j, collision, num_tries;
  for(i = 0; i < swarm->num_robots; i++)
  {
    collision = 0;
    for(num_tries = 0; num_tries < tries_per_robot; num_tries++)
    {
      for(j = 0; j < swarm->num_robots; j++)
      {
        if(i == j)
        {
          continue;
        }
        float c_dist = dist_robots(swarm->robots[i], swarm->robots[j]);
        if(c_dist < swarm->robots[i]->radius + swarm->robots[j]->radius)
        {
          collision = 1;
          break;
        }
      }
      if(collision == 0)
      {
        // no robots are in collision with i
        break;
      }
      // otherwise, replace the robot at another randomly selected location
      swarm->robots[i]->position[0] = rand_f_between(swarm->pose_min[0], swarm->pose_max[0]);
      swarm->robots[i]->position[1] = rand_f_between(swarm->pose_min[1], swarm->pose_max[1]);
     
    }
  }
}







// returns 1 if any ropbot overlaps this particular robot
int swarm_any_overlapping_bot(SWARM* swarm, ROBOT* robot)
{
  int j;
  for(j = 0; j < swarm->num_robots; j++)
  {
    if(robot->id == j)
    {
      continue;
    }

    float c_dist = dist_robots(robot, swarm->robots[j]);

    if(c_dist < robot->radius + swarm->robots[j]->radius)
    {
      printf("%d and %d overlap\n", robot->id, j);
      return 1;
    }
  }
  return 0;
}

// returns 1 if any of the robots are overlapping
int swarm_any_overlapping(SWARM* swarm)
{
  int i;
  for(i = 0; i < swarm->num_robots; i++)
  {
    if(swarm_any_overlapping_bot(swarm, swarm->robots[i]) == 1)
    {
      return 1;
    }
  }
  return 0;
}


// this uses a particle force simulation push nearby robots away from
// each other until either no robots overlap or max_iterations has been reached
// also pulls far away robot nearer
void swarm_particle_distribute(SWARM* swarm, int max_iterations)
{
  int t,i,j;
  float c_dist, this_force;

  for(t = 0; t < max_iterations; t++)
  {
    if(swarm_any_overlapping(swarm) == 0)
    {
      //return;
    }
    for(i = 0; i < swarm->num_robots; i++)
    {
      if(swarm_any_overlapping_bot(swarm, swarm->robots[i]) == 0 )
      {
        // i is not overlapping any other robot
        continue;
      }

      float total_force_x = 0.0f;
      float total_force_y = 0.0f;
      for(j = 0; j < swarm->num_robots; j++)
      {
        if(i == j)
        {
          continue;
        }

        c_dist = dist_robots(swarm->robots[i], swarm->robots[j]);
        if(c_dist > 1.1f*(swarm->robots[i]->radius + swarm->robots[j]->radius))
        {
          //this_force = -0.1f/(c_dist*c_dist);
          continue;
        }
        else
        {
          this_force = 1.0f/(c_dist*c_dist);
        }

        total_force_x += this_force / c_dist * (swarm->robots[i]->position[0] - swarm->robots[j]->position[0]); 

        total_force_y += this_force / c_dist * (swarm->robots[i]->position[1] - swarm->robots[j]->position[1]); 
      }

      float radius_frac = swarm->robots[i]->radius/10.f;

      float delta_x = total_force_x * radius_frac;
      delta_x = fmin(radius_frac, fmax(-radius_frac, delta_x)); 
      
      float delta_y = total_force_y * radius_frac;
      delta_y = fmin(radius_frac, fmax(-radius_frac, delta_y)); 

      swarm->robots[i]->position[0] += delta_x;
      swarm->robots[i]->position[1] += delta_y;

      // now reset randomly any robots that have been pushed out
      if(swarm->robots[i]->position[0] < swarm->pose_min[0] || swarm->robots[i]->position[0] > swarm->pose_max[0])
      {
        swarm->robots[i]->position[0] = rand_f_between(swarm->pose_min[0], swarm->pose_max[0]);
      }

      if(swarm->robots[i]->position[1] < swarm->pose_min[1] || swarm->robots[i]->position[1] > swarm->pose_max[1])
      {
        swarm->robots[i]->position[1] = rand_f_between(swarm->pose_min[1], swarm->pose_max[1]);
      }

      //printf("%f %f\n", swarm->robots[i]->position[0], swarm->robots[i]->position[1]);
    }
  }

  if(swarm_any_overlapping(swarm) == 1)
  {
    printf("WARNING: unable to remove robot overlaps\n");
  }
  else
  {
    printf("none overlapping \n");
  }
}




// this calculates the communication connectivity graph, and stores
// a list of neighbors at each node. Note, this is the graph
// that has an edge if a particular robot i can hear another robot j
void calculate_connectivity_graph(SWARM* swarm)
{
  int i, j;

 
  for(i = 0; i < swarm->num_robots; i++)
  {
    // first pass, find number of neighbors
    swarm->robots[i]->num_neighbors = 0;

    for(j = 0; j < swarm->num_robots; j++)
    {
      if(i == j)
      {
        continue;
      }
      if(dist_robots(swarm->robots[i], swarm->robots[j]) < swarm->robots[j]->com_radius)
      {
        swarm->robots[i]->num_neighbors++;
      }
    }



    // allocate memory
    if(swarm->robots[i]->neighbors == NULL)
    {
      // NOTE: to facilitate moving robots, we decided to enable this to be recomputed each time
      //       which required allowing up to full connectivity, it seems better just to allocate memory
      //       once for as large as it could ever be and then not worry about deleting and reallocating 
      //       all the time.

      // swarm->robots[i]->neighbors = (ROBOT**)calloc(swarm->robots[i]->num_neighbors, sizeof(ROBOT*));
      swarm->robots[i]->neighbors = (ROBOT**)calloc(NUM_ROBOTS, sizeof(ROBOT*));
    }

    // second pass, set up links
    int n = 0;
    for(j = 0; j < swarm->num_robots; j++)
    {
      if(i == j)
      {
        continue;
      }
      if(dist_robots(swarm->robots[i], swarm->robots[j]) < swarm->robots[j]->com_radius)
      {
        swarm->robots[i]->neighbors[n] = swarm->robots[j];
        n++;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void drive_buddies_for_duration(ROBOT* bot, SWARM* swarm,  float dt)
{
  int i;
  for (i=0; i<(swarm->num_robots/2); i++)
  {
    if ((swarm->vect[i].first==bot->id)||(swarm->vect[i].second==bot->id)) // checking at which index of the vector is our jth robot 
    {
      i=i;
      break;
    }
  }

  if (swarm->vect[i].first==bot->id)
  {
    int q;
    for (int j=0; j<swarm->num_robots;j++)
    {  
      if (swarm->robots[j]->id==swarm->vect[i].second)
      {
        q=j;
        break;
      }   
    }

    float time_resolution = 0.01;   // time resolution
    int last_loop = 0;
    float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
    float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
    // note that bot->speed is the current ratio of max alowed speed

    // if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
    // {
    //   // robot is stopped

    //   bot->d_orientation[0] = 0.0f;
    //   return;
    // }

    if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) && (swarm->robots[q]->speed[0]<= 0.0f && swarm->robots[q]->speed[1]<= 0.0f) || dt <= 0.0f)
    //if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
    {
      // robot is stopped

      bot->d_orientation[0] = 0.0f;
      swarm->robots[q]->d_orientation[0]=0.0f;
      return;
    }

    // assuming tank steering is a good enough approximation
    float x = bot->position[0];
    float y = bot->position[1];
    float theta = bot->orientation[0];
    bot->speed[0]=bot->speed[1];

    float x1 = swarm->robots[q]->position[0];
    float y1 =swarm->robots[q]->position[1];
    float theta1 = swarm->robots[q]->orientation[0];
    swarm->robots[q]->speed[0]=swarm->robots[q]->speed[1];


    // if robot is moving straight
    //if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f  && fabs(swarm->robots[q]->speed[1] - swarm->robots[q]->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
    if(fabs(swarm->robots[q]->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
    {
      float speed = ((swarm->robots[q]->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
      float dx = dt*cos(theta)*speed;
      float dy = dt*sin(theta)*speed;
      bot->position[0] += dx;
      bot->position[1] += dy;
      bot->d_orientation[0] = 0.0f;

      // float speed1 = ((swarm->robots[q]->speed[1] + swarm->robots[q]->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
      // float dx1 = dt*cos(theta)*speed1;
      // float dy1 = dt*sin(theta)*speed1;
      swarm->robots[q]->position[0] += dx;
      swarm->robots[q]->position[1] += dy;
      swarm->robots[q]->d_orientation[0] = 0.0f;
      return; 
    }
      
    // otherwise robot is turning

    float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
    float v_r = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

    // float v_l1 = swarm->robots[q]->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius; 
    // float v_r1 = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*(bot->radius+2*bot->radius);

    float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
    float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

    // float R1 = 2*bot->radius * (v_r1 -2*v_l1)/(v_l1 - v_r1);  // radius of rotation
    // float omega1 = (v_r - v_l)/bot->radius;

    float c_x = x - R*sin(theta); // center of rotation x
    float c_y = y + R*cos(theta); // center of rotation y

    // float c_x1 = x1 - R1*sin(theta1); // center of rotation x
    // float c_y1 = y1 + R1*cos(theta1);

    x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
    y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
    theta = theta + omega*dt;

    x1 = cos(omega*dt)*(x1 - c_x) - sin(omega*dt)*(y1 - c_y) + c_x;
    y1 = sin(omega*dt)*(x1 - c_x) + cos(omega*dt)*(y1 - c_y) + c_y;
    // theta1 = theta1 + omega1*dt;

    while(theta > 2.0f*PI)
    {
      theta -= 2.0f*PI;
    }
    while(theta < -2.0f*PI)
    {
      theta += 2.0f*PI;
    }

    // while(theta1 > 2.0f*PI)
    // {
    //   theta1 -= 2.0f*PI;
    // }
    // while(theta1 < -2.0f*PI)
    // {
    //   theta1 += 2.0f*PI;
    // }


    bot->position[0] = x;
    swarm->robots[q]->position[0]=x1;

    bot->position[1] = y;
    swarm->robots[q]->position[1]=y1;
  
    bot->orientation[0] = theta;
    swarm->robots[q]->orientation[0]=theta;

    bot->d_orientation[0] = omega;
    swarm->robots[q]->d_orientation[0]=omega;
  }
  
  // if (swarm->vect[i].second==bot->id)
  // {
  //   int q;
  //   for (int j=0; j<swarm->num_robots;j++)
  //   {
  //     if (swarm->robots[j]->id==swarm->vect[i].first)
  //     {
  //       q=j;
  //       break;
  //     }
  //   }

  //   float time_resolution = 0.01;   // time resolution
  //   int last_loop = 0;
  //   float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
  //   float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
  //   // note that bot->speed is the current ratio of max alowed speed

  //   //if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) && (swarm->robots[q]->speed[0]<= 0.0f && swarm->robots[q]->speed[1]<= 0.0f) || dt <= 0.0f)
  //   if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
  //   {
  //     // robot is stopped

  //     bot->d_orientation[0] = 0.0f;
  //     //swarm->robots[q]->d_orientation[0]=0.0f;
  //     return;
  //   }

  //   // assuming tank steering is a good enough approximation
  //   float x = bot->position[0];
  //   float y = bot->position[1];
  //   float theta = bot->orientation[0];

  //   // float x1 = swarm->robots[q]->position[0];
  //   // float y1 =swarm->robots[q]->position[1];
  //   // float theta1 = swarm->robots[q]->orientation[0];

    


  //   // if robot is moving straight
  //   //if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f  && fabs(swarm->robots[q]->speed[1] - swarm->robots[q]->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  //   if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  //   {
  //     float speed = ((bot->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
  //     float dx = dt*cos(theta)*speed;
  //     float dy = dt*sin(theta)*speed;
  //     bot->position[0] += dx;
  //     bot->position[1] += dy;
  //     bot->d_orientation[0] = 0.0f;

  //     // float speed1 = ((swarm->robots[q]->speed[1] + swarm->robots[q]->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
  //     // float dx1 = dt*cos(theta)*speed1;
  //     // float dy1 = dt*sin(theta)*speed1;
  //     // swarm->robots[q]->position[0] += dx1;
  //     // swarm->robots[q]->position[1] += dy1;
  //     // swarm->robots[q]->d_orientation[0] = 0.0f;
  //     return; 
  //   }
      
  //   // otherwise robot is turning

  //   float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
  //   float v_r = bot->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

  //   // float v_l1 = swarm->robots[q]->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius; 
  //   // float v_r1 = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*(bot->radius+2*bot->radius);

  //   float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
  //   float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

  //   // float R1 = 2*bot->radius * (v_r1 -2*v_l1)/(v_l1 - v_r1);  // radius of rotation
  //   // float omega1 = (v_r - v_l)/bot->radius;

  //   float c_x = x - R*sin(theta); // center of rotation x
  //   float c_y = y + R*cos(theta); // center of rotation y

  //   // float c_x1 = x1 - R1*sin(theta1); // center of rotation x
  //   // float c_y1 = y1 + R1*cos(theta1);

  //   x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
  //   y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
  //   theta = theta + omega*dt;

  //   // x1 = cos(omega1*dt)*(x1 - c_x1) - sin(omega1*dt)*(y1 - c_y1) + c_x1;
  //   // y1 = sin(omega1*dt)*(x1 - c_x1) + cos(omega1*dt)*(y1 - c_y1) + c_y1;
  //   //theta1 = theta1 + omega1*dt;

  //   while(theta > 2.0f*PI)
  //   {
  //     theta -= 2.0f*PI;
  //   }
  //   while(theta < -2.0f*PI)
  //   {
  //     theta += 2.0f*PI;
  //   }

  //   // while(theta1 > 2.0f*PI)
  //   // {
  //   //   theta1 -= 2.0f*PI;
  //   // }
  //   // while(theta1 < -2.0f*PI)
  //   // {
  //   //   theta1 += 2.0f*PI;
  //   // }


  //   bot->position[0] = x;
  //   //swarm->robots[q]->position[0]=x;

  //   bot->position[1] = y;
  //   swarm->robots[q]->position[1]=y;

  //   bot->orientation[0] = theta;
  //   swarm->robots[q]->orientation[0]=theta;

  //   bot->d_orientation[0] = omega;
  //   //swarm->robots[q]->d_orientation[0]=omega;
    
    
  // }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// drives all robots in the swarm for duration dt
void drive_swarm_for_duration(SWARM* swarm, float dt)
{
  for(int i = 0; i < swarm->num_robots; i++)
  {
    now_computing_with(swarm->robots[i]);
    drive_robot_for_duration(swarm->robots[i], dt);
    done_computing_with(swarm->robots[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive_buddy_swarm_for_duration(float dt, SWARM* swarm)
{
  //for (int i = 0, j = 0; i < swarm->num_robots && j < (swarm->num_robots/2); i++, j++)
  for (int i = 0; i < swarm->num_robots; i++)
    {
        now_computing_with(swarm->robots[i]);
        drive_buddies_for_duration(swarm->robots[i], swarm, dt);
        done_computing_with(swarm->robots[i]);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// performs collision checking for the swarm vs
void collision_check_bouandary(SWARM* swarm)
{
  int j;
  for(j = 0; j < swarm->num_robots; j++)
  {
    if(swarm->robots[j]->position[0] > swarm->pose_max[0] - swarm->robots[j]->radius)
    { 
      swarm->robots[j]->position[0] = swarm->pose_max[0] - swarm->robots[j]->radius;
    }
    else if(swarm->robots[j]->position[0] < swarm->pose_min[0] + swarm->robots[j]->radius)
    { 
      swarm->robots[j]->position[0] = swarm->pose_min[0] + swarm->robots[j]->radius;
    }

    if(swarm->robots[j]->position[1] > swarm->pose_max[1] - swarm->robots[j]->radius)
    { 
      swarm->robots[j]->position[1] = swarm->pose_max[1] - swarm->robots[j]->radius;
    }
    else if(swarm->robots[j]->position[1] < swarm->pose_min[1]  + swarm->robots[j]->radius)
    { 
      swarm->robots[j]->position[1] = swarm->pose_min[1]  + swarm->robots[j]->radius;
    }
  }
}



// this uses a particle force simulation push nearby robots away from
// each other until either no robots overlap or max_iterations has been reached
// also pulls far away robot nearer

void collision_check_robots(SWARM* swarm)
{
  int t,i,j;
 
  float new_x[NUM_ROBOTS];
  float new_y[NUM_ROBOTS];

  for(i = 0; i < swarm->num_robots; i++)
  {
    new_x[i] = swarm->robots[i]->position[0];
    new_y[i] = swarm->robots[i]->position[1];
  }


  for(i = 0; i < swarm->num_robots; i++)
  {
    for(j = i+1; j < swarm->num_robots; j++)
    {
      float c_dist = dist_robots(swarm->robots[i], swarm->robots[j]);
      if(c_dist > swarm->robots[i]->radius + swarm->robots[j]->radius)
      {
        continue;
      }
     
      // note this needs to be adjusted if there are ever robots of different radii 
      // (since c_x will not be equally betwee them)
      float c_x = (swarm->robots[i]->position[0] + swarm->robots[j]->position[0])/2.0f;
      float c_y = (swarm->robots[i]->position[1] + swarm->robots[j]->position[1])/2.0f;
      
      float i_d_x = swarm->robots[i]->position[0] - c_x;
      float i_d_y = swarm->robots[i]->position[1] - c_y;
      float i_d_h = sqrt(i_d_x*i_d_x + i_d_y*i_d_y);

      new_x[i] += (swarm->robots[i]->radius - i_d_h)*(i_d_x/i_d_h);
      new_y[i] += (swarm->robots[i]->radius - i_d_h)*(i_d_y/i_d_h);

      float j_d_x = swarm->robots[j]->position[0] - c_x;
      float j_d_y = swarm->robots[j]->position[1] - c_y;
      float j_d_h = sqrt(j_d_x*j_d_x + j_d_y*j_d_y);

      new_x[j] += (swarm->robots[j]->radius - j_d_h)*(j_d_x/j_d_h);
      new_y[j] += (swarm->robots[j]->radius - j_d_h)*(j_d_y/j_d_h);

    }

  }


  for(i = 0; i < swarm->num_robots; i++)
  {
    swarm->robots[i]->position[0] = new_x[i];
    swarm->robots[i]->position[1] = new_y[i];
  }


  // now reset randomly any robots that have been pushed out
  collision_check_bouandary(swarm);


  if(swarm_any_overlapping(swarm) == 1)
  {
    printf("WARNING: unable to remove robot overlaps\n");
  }
  else
  {
    printf("none overlapping \n");
  }
}



/*---------------- LIGHTPROJECTOR DATA STRUCTURE --------------------*/


// returns a new light projector
LIGHTPROJECTOR* make_lightprojector(int pixel_rows, int pixel_cols)
{
  LIGHTPROJECTOR* lp = (LIGHTPROJECTOR*)calloc(1,sizeof(LIGHTPROJECTOR));
  lp->pixel_rows = pixel_rows;
  lp->pixel_cols = pixel_cols;

  lp->light_data = (float**)calloc(pixel_cols, sizeof(float*));
  int i,j;
  for(i = 0; i < pixel_cols; i++)
  {
    lp->light_data[i] = (float*)calloc(pixel_rows, sizeof(float));
    for(j = 0; j < pixel_rows; j++)
    {
      lp->light_data[i][j] = 0.0f;
    }
    lp->light_data[i][i] = 1.0f;
  }

  return lp;
}

// cleans memory of a light projector
void destroy_lightprojector(LIGHTPROJECTOR* lp)
{
  int i,j;
  for(i = 0; i < lp->pixel_cols; i++)
  {
    free(lp->light_data[i]);
  }
  free(lp->light_data);
  free(lp);
}


// all robots input visual light data
// assumption is that light projector covers min to max allowed bot locations
void update_swarm_visual_light_data(SWARM* swarm, LIGHTPROJECTOR* lp)
{
  int i;
  for(i = 0; i < swarm->num_robots; i++)
  {
    swarm->robots[i]->visual_light_input = get_visual_light_data(swarm->robots[i], lp);
  }
}

// loads the light data from a bitmap file
// uses read_bitmap.c
// note the swap of rows and colls (not an issue for editing the bmp)
// also note that image is converted to inverse gray-scale (i.e., 
// the bitmap is assumed to have the photo-negative of the light data.
LIGHTPROJECTOR* load_projector_from_file(const char *filename)
{
  Bitmap*  B = load_Bitmap_from_file(filename);
  // print_Bitmap_info(B);

  LIGHTPROJECTOR* lp = make_lightprojector((int)B->DBI_Height, (int)B->DBI_Width);

  Image* Im = convert_Bitmap_to_image(B);
  // print_Image(Im);

  int i,j;
  for(i = 0; i < lp->pixel_cols; i++)
  {
    for(j = 0; j < lp->pixel_rows; j++)
    { 
      lp->light_data[i][lp->pixel_rows-j-1] = 1.0f-(Im->Red->A[j][i] + Im->Blue->A[j][i] + Im->Green->A[j][i])/3.0f;
    }
  }

  destroy_Image(Im);
  destroy_Bitmap(B);

  return lp;
}




// loads in all light data
// this uses a lot of the globals defined at the top of the file
void load_all_light_data()
{
  light_patterns = (LIGHTPROJECTOR**)calloc(num_light_patterns, sizeof(LIGHTPROJECTOR*));

  int i;
  for(i = 0; i < num_light_patterns; i++)
  { 
    if(i == 0)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_0);
    }
    else if(i == 1)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_1);
    }
    else if(i == 2)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_2);
    }
  /*  else if(i == 3)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_3);
    }
    else if(i == 4)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_4);
    }
    else if(i == 5)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_5);
    }
    else if(i == 6)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_6);
    }
    else if(i == 7)
    {
      light_patterns[i] = load_projector_from_file(light_data_filename_7);
    } */
  }
}


// cleans up all of the light data
void destroy_all_light_data()
{
  int i;
  for(i = 0; i < num_light_patterns; i++)
  { 
    destroy_lightprojector(light_patterns[i]);
  }
  free(light_patterns);
}











/*--------------------------- UPDATE FUNCTION ---------------------------*/


//void run_update(SWARM* swarm)
//{
///*  // get elapsed time
//  double time_now = display_speed_mult * (get_curr_time() - start_time);


//  #ifdef SAVE_DATA
//  // check if need to save data
//  while(num_data_points < (int)time_now)
//  {
//    record_data(swarm_data, time_now);
//  }
//  #endif



//  // check if time to quit
//  if(time_out <= time_now)
//  {
//    // time to quit, the following calls exit after cleaning up memory
//    cleanup();
//  }


//  // print the training error
//  print_swarm_error(swarm_data);
//  printf("t: %f\n", time_now); 

//  // printf("t: %f  receive: %d  drop: %d  backoffs: %d\n", time_now, swarm_data->total_recieved, swarm_data->total_dropped, swarm_data->total_backoffs);  

//  // check if any robots need to be rebooted
//  if(reboot_robot_flag == 1)
//  {
//    soft_reboot_robot(swarm_data->robots[selected_robot]);
//    printf("robot %d has been re-initialized\n", selected_robot);
//    reboot_robot_flag = 0;
//  }
//*/
//  // do calculations on robots
//  do_swarm_computation(swarm);

//  // do communications that have happened
//  //do_communication(swarm_data, (float)time_now);
//}


// ------------------------- stuff for debugging --------------------------


// creaates a new debug log
DEBUG_LOG* make_new_debug_log()
{ 
  DEBUG_LOG* DL = (DEBUG_LOG*)malloc(sizeof(DEBUG_LOG));
  DL->total_lines = DEBUG_LINES;
  DL->current_line = 0;

  DL->ring_buffer = (char**)calloc(DEBUG_LINES, sizeof(char*));
  for(int i = 0; i < DL->total_lines; i++)
  {
    DL->ring_buffer[i] = (char*)calloc(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE, sizeof(char));
    sprintf(DL->ring_buffer[i], " <br> ");
  }

  DL->log_formatted = (char*)calloc(DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE), sizeof(char));
  
  sprintf(DL->log_formatted, " <br> ");


  DL->displaying_error_message = 0;

  return DL;
}

// cleans up the debug log
void destroy_debug_log(DEBUG_LOG* DL)
{
  if(DL == NULL)
  {
    return;
  }


  for(int i = 0; i < DL->total_lines; i++)
  {
    free(DL->ring_buffer[i]);
  }
  free(DL->ring_buffer);
  free(DL->log_formatted);

  free(DL);
}

// this places the current state of the debug log from the ring_buffer
// into the single log_formatted string so it can be sent to javascript
void format_debug_log(DEBUG_LOG* DL)
{
  if(DL == NULL)
  {
    return;
  }


  int final_line = DL->current_line;
  int first_line = DL->current_line + 1;
  if(first_line >= DL->total_lines)
  {
    first_line = 0;
  }

  int j = 0;
  if(first_line > 0)
  {
    for(int i = first_line; i < DL->total_lines; i++)
    {
      if(j + strlen(DL->ring_buffer[i]) < DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE))
      {
        strcpy(&(DL->log_formatted[j]), DL->ring_buffer[i]);
        j+= strlen(DL->ring_buffer[i]);
      } 
    }
    for(int i = 0; i <= final_line; i++)
    {
      if(j + strlen(DL->ring_buffer[i]) < DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE))
      {
        strcpy(&(DL->log_formatted[j]), DL->ring_buffer[i]);
        j+= strlen(DL->ring_buffer[i]);
      } 
    }
  }
  else if(first_line == 0)
  {
    for(int i = 0; i < DL->total_lines; i++)
    {
      if(j + strlen(DL->ring_buffer[i]) < DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE))
      {
        strcpy(&(DL->log_formatted[j]), DL->ring_buffer[i]);
        j+= strlen(DL->ring_buffer[i]);
      } 
    }

  }
}

// adds the message to the log data
void log_message_raw(int mode, DEBUG_LOG* DL, const char* message)
{
  if(DL == NULL || DL->displaying_error_message == 1)
  {
    return;
  }

  if(mode == 0)
  {
    sprintf(DL->ring_buffer[DL->current_line], "<b>Robot %d [%d]:</b> ", bot_global->id,  bot_global->kilo_uid);
  }
  else
  {
    sprintf(DL->ring_buffer[DL->current_line], "<b>Simulator:</b> ");
  }

  if(strlen(message) > DEBUG_CHARS_PER_LINE)
  {
    strcat(DL->ring_buffer[DL->current_line], "[requested output is too long]");
  }
  else
  {
    strcat(DL->ring_buffer[DL->current_line], message);
  }
  strcat(DL->ring_buffer[DL->current_line], "<br>");

  DL->current_line++;
  if(DL->current_line == DL->total_lines)
  {
    DL->current_line = 0;
  }

}


// adds the message to the log data, but works like sprintf
// (which requires using arg lists, see info about <stdarg.h>
// for more details
void log_message(const char *format, ...) 
{    
   char buffer[DEBUG_CHARS_PER_LINE];

   va_list args;
   va_start(args,format);
   vsprintf(buffer,format,args);
   va_end(args);

   log_message_raw(0, debug_log, (const char*)buffer);
}

// same as above, bur from the simulator
void log_message_from_sim(const char *format, ...) 
{    
   char buffer[DEBUG_CHARS_PER_LINE];

   va_list args;
   va_start(args,format);
   vsprintf(buffer,format,args);
   va_end(args);

   log_message_raw(1, debug_log, (const char*)buffer);
}



// adds the error message to the log data and posts it ASAP
// then disables new messages
void error_message_raw(int mode, DEBUG_LOG* DL, const char* message)
{
  if(DL == NULL)
  {
    return;
  }

  if(mode == 0)
  {
    sprintf(DL->ring_buffer[DL->current_line], "<b>Robot %d [%d] ERROR:</b> ", bot_global->id,  bot_global->kilo_uid);
  }
  else
  {
    sprintf(DL->ring_buffer[DL->current_line], "<b>Simulator ERROR:</b> ");
  }

  if(strlen(message) > DEBUG_CHARS_PER_LINE)
  {
    strcat(DL->ring_buffer[DL->current_line], "[requested output is too long]");
  }
  else
  {
    strcat(DL->ring_buffer[DL->current_line], message);
  }
  strcat(DL->ring_buffer[DL->current_line], "<br>");

  DL->current_line++;
  if(DL->current_line == DL->total_lines)
  {
    DL->current_line = 0;
  }

  DL->displaying_error_message = 1;
/*
  format_debug_log(DL);
  char cst[DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE) + 100];
  sprintf(cst, "print_debugging_log('ERROR<br> <br>%s')", (const char *)(debug_log->log_formatted));
  emscripten_run_script((const char*)cst);
*/
}


// displays an error message
void error_message(const char *format, ...) 
{    
   char buffer[DEBUG_CHARS_PER_LINE];

   va_list args;
   va_start(args,format);
   vsprintf(buffer,format,args);
   va_end(args);

   error_message_raw(0, debug_log, (const char*)buffer);
}


// same as above, but from the simulator
void error_message_from_sim(const char *format, ...) 
{    
   char buffer[DEBUG_CHARS_PER_LINE];

   va_list args;
   va_start(args,format);
   vsprintf(buffer,format,args);
   va_end(args);

   error_message_raw(1, debug_log, (const char*)buffer);
}

/*----------------------- EVENT_NODE (nodeT) DATA STRUCTURE --------------------------*/

// empty event
EVENT_NODE* new_event()
{
  EVENT_NODE* ev = (EVENT_NODE*)malloc(sizeof(EVENT_NODE));
  ev->robot_id = bot_global->id;
  return ev;
}


// LED Change event
EVENT_NODE* new_event_LEDChange(uint8_t color)
{
  EVENT_NODE* ev = new_event();
  ev->type = LEDChange;
  ev->color = color;

  return ev;
}


// MOTOR Change event
EVENT_NODE* new_event_MOTORChange(uint8_t left, uint8_t right)
{
  EVENT_NODE* ev = new_event();
  ev->type = MOTORChange;
  ev->left = left;
  ev->right = right;

  return ev;
}


// LOOP Call event
EVENT_NODE* new_event_LOOPCall()
{

  EVENT_NODE* ev = new_event();
  ev->type = LOOPCall;

  //log_message_from_sim("------- new_event_LOOPCall %d -----", ev->robot_id);

  return ev;
}


/*----------------------- EVENT_LOT DATA STRUCTURE --------------------------*/


// creates a new event log
EVENT_LOG* make_new_event_log(int capacity, double time_start)
{

   EVENT_LOG* el = (EVENT_LOG*)malloc(sizeof(EVENT_LOG));

   el->H = sbbsHeapConstruct(capacity);  
   el->time_start = time_start;
   el->event_count = 0;

   //log_message_from_sim("[event log construct]");

   return el;
}


// creates a new event log
void destroy_event_log(EVENT_LOG* el)
{
  while(el->H->indexOfLast >= 0)
  {
    EVENT_NODE* event_node = popHeap(el->H);
    free(event_node);
  }
  free(el);
}


// adds the event to the event log
void log_event(EVENT_LOG* el, EVENT_NODE* ev)
{

  if(el == NULL)
  {
    return;
  }

  double time_now =  bot_global->time_since_last_time_update + bot_global->time_since_start;

  if(el->H->indexOfLast < el->H->capacity-1)
  {
    // the event heap still has room

    ev->time = time_now;
    ev->inHeap = false;
    ev->heapIndex = -1;

    ev->event_id = el->event_count;
    el->event_count++;

    addToHeap(el->H, ev, time_now);

/*
    if(ev->type == LEDChange)
    {
      log_message_from_sim("[event log %d add %d]: (%f + %f = %f,  LEDChange)", ev->robot_id, el->event_count, bot_global->time_since_last_time_update, bot_global->time_since_start, time_now);
    }
    else if(ev->type == MOTORChange)
    {
      log_message_from_sim("[event log %d add %d]: (%f + %f = %f,  MOTORChange)", ev->robot_id, el->event_count, bot_global->time_since_last_time_update, bot_global->time_since_start, time_now);
    }
    else if(ev->type == LOOPCall)
    {
      log_message_from_sim("[event log %d add %d]: (%f + %f = %f,  LOOPCall)", ev->robot_id, el->event_count, bot_global->time_since_last_time_update, bot_global->time_since_start, time_now);
    }
*/
  }
}




// processes events from the log up until those of final time
void process_events_from_log(EVENT_LOG* EL, SWARM* swarm, double final_time)
{

  while(EL->H->indexOfLast >= 0)
  {
    EVENT_NODE* ev = topHeap(EL->H);
    if(ev->time > final_time)
    {
      // all events up until the desired time have been processed
      break;
    }

    // remove this event and process it
    ev = popHeap(EL->H);

    now_computing_with(swarm->robots[ev->robot_id]);

    if(ev->type == LEDChange)
    {
      //log_message_from_sim("[event log process %d]: (%f < %f,  LEDChange)", ev->event_id, ev->time, final_time);
      set_color_event(ev);
    }
    else if(ev->type == MOTORChange)
    {
      //log_message_from_sim("[event log process %d]: (%f < %f,  MOTORChange)", ev->event_id, ev->time, final_time);
      set_motors_event(ev);
    }
    else if(ev->type == LOOPCall)
    {
      //log_message_from_sim("[event log process %d]: (%f < %f,  LOOPCall)", ev->event_id, ev->time, final_time);
      enable_loop_event(ev);
    }
    free(ev);


    done_computing_with(swarm->robots[ev->robot_id]);
  }

}

#endif //__SIM_DATA_STRUCTURES_CPP__




