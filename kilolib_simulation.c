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

#ifndef __KILOLIB_SIMULATION_C__
#define __KILOLIB_SIMULATION_C__

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>





// ---------------------- globals --------------------------------

// these globals are expected to exist by our simulation code, 
// and used to help switch between the computation of different robots
ROBOT* bot_global; 
//pthread_mutex_t timing_mutext;
//pthread_barrier_t barrier;
long check_in_time_dt = 100;          // each thread sleeps for this many miliseconds to let other code process, so 100 = 1/10 of a second
ROBOT* robot_local_copy_global_pass;  // used once (per robot, while robot has control mutext) to help setup main loops
void* kilo_user_globals[NUM_ROBOTS];  // user variables, one per each robot


int num_light_patterns = 3;
LIGHTPROJECTOR** light_patterns;         // used to store different light projection patterns

DEBUG_LOG* debug_log = NULL;  // data for logging debugging info

EVENT_LOG* event_log = NULL;         // event log

SWARM* swarm_ = NULL;
int iii = 0;   // counter for number of loops on main thread


int seeded_the_rand = 0;

char light_data_filename_0[] = "light_patterns/A.bmp";
char light_data_filename_1[] = "light_patterns/B.bmp";
char light_data_filename_2[] = "light_patterns/C.bmp";




// start of stuff replacing kilolib.h functionality -------------------------------------



// these are in the global namespace (and wrap the thins in the individual namespaces)
void kilo_message_rx_wrapper(ROBOT* bot, message_t *m, distance_measurement_t *d) 
{ 
  if(bot->kilo_message_rx != NULL)
  {
    bot->kilo_message_rx(m, d);
  }
}

message_t* kilo_message_tx_wrapper(ROBOT* bot) 
{ 
  if(bot->kilo_message_tx != NULL)
  {
    return bot->kilo_message_tx();
  }
  else
  {
    return NULL;
  }
}

void kilo_message_tx_success_wrapper(ROBOT* bot) 
{  
  if(bot->kilo_message_tx_success != NULL)
  {
    bot->kilo_message_tx_success();
  }
}



// (selected) function headers
uint8_t estimate_distance(const distance_measurement_t *dist);

// end (for now) of stuff copied from kilolib.h (there is more lower down) -------------------------------------




/* ----------------------------  stuff for debugging ------------------------------------*/
// checks the state of the swarm and prints an error if it is not good
// on_sim_main_thread = 1 prints from simulator
// on_sim_main_thread = 0 prints from the id of the robot
void check_state(SWARM* swarm, DEBUG_LOG* dl, int on_sim_main_thread, const char *format, ...) 
{    
  if(swarm == NULL || dl == NULL)
  {
    return;
  }

  // extract user stuff to print
  char buffer[DEBUG_CHARS_PER_LINE];
  va_list args;
  va_start(args,format);
  vsprintf(buffer,format,args);
  va_end(args);


  int found_error = 0;
  for(int i = 0; i < swarm->num_robots; i++)
  {
    if(swarm->robots[i]->id != (int)swarm->robots[i]->kilo_uid)
    {
      if(on_sim_main_thread == 1)
      {
        error_message_from_sim("[%s] IDs do not match for robot %d (%d)", buffer, swarm->robots[i]->id, (int)swarm->robots[i]->kilo_uid); 
      }
      else
      {
        error_message("[%s] IDs do not match for robot %d (%d)", buffer, swarm->robots[i]->id, (int)swarm->robots[i]->kilo_uid); 
      }
    }
  }
}


/*------------------- STUFF FOR CALLING INDIVIDUAL ROBOT CODE ---------------------------*/
//                             (not from kilolib.h)



// -returns- the visual light data as seen by the robot
// assumption is that light projector covers min to max allowed bot locations
float get_visual_light_data(ROBOT* bot, LIGHTPROJECTOR* lp)
{
  int x = (int)((bot->position[0] - bot->pose_min[0])/(bot->pose_max[0] - bot->pose_min[0])*(float)lp->pixel_cols);

  int y = (int)((bot->position[1] - bot->pose_min[1])/(bot->pose_max[1] - bot->pose_min[1])*(float)lp->pixel_rows);

  return lp->light_data[x][y];
}




// this sets bot to be the active robot (i.e., as far as the kilolib code
// global variables are concerned)
void now_computing_with(ROBOT* bot)
{

  //check_state(swarm_, debug_log, 1, "simulation loop [%d], starting with %d", iii, bot->id);

  // set the globals used by our simulation code
  bot_global = bot;

 
  // set the globals exppected by kilobot code (from kilolib.h)
  kilo_uid = bot->kilo_uid;
  bot->kilo_ticks = (uint32_t)((bot->time_since_start + bot->time_since_last_time_update)*30.0);
  kilo_ticks = bot->kilo_ticks;


  //log_message_from_sim("robot %d time: (%f, %u)", bot->id, bot->time_since_start, bot->kilo_ticks);


  // set the user defined globals expected by the kilobot code
  *kilo_globals = ((void*)kilo_user_globals[bot->id]);
  
  #ifdef codeSpaceA_exists
    if(bot->code_space == A)
    {
      codeSpaceA::g = (codeSpaceA::GLOBALS*)*kilo_globals;
    }
  #endif 

  #ifdef codeSpaceB_exists
    if(bot->code_space == B)
    {
      codeSpaceB::g = (codeSpaceB::GLOBALS*)*kilo_globals;
    }
  #endif


  #ifdef codeSpaceC_exists
    if(bot->code_space == C)
    {
      codeSpaceC::g = (codeSpaceC::GLOBALS*)*kilo_globals;
    }
  #endif

  #ifdef codeSpaceD_exists
    if(bot->code_space == D)
    {
      codeSpaceD::g = (codeSpaceD::GLOBALS*)*kilo_globals;
    }
  #endif

  #ifdef codeSpaceE_exists
    if(bot->code_space == E)
    {
      codeSpaceE::g = (codeSpaceE::GLOBALS*)*kilo_globals;
    }
  #endif


  // set the ambiant light value for the current robot
  if(curr_light_pattern >= 0)
  {
    float l_in = get_visual_light_data(bot, light_patterns[curr_light_pattern]);
    bot->visual_light_input = l_in; // update the one used for display
    bot->ambient_light = (int16_t)((int)(l_in*1024.0f));

  }
  else
  {
    bot->ambient_light = 0;
  }
}

// this takes the nn data from globals and stores into the bot
// for instance, any led values that have changed
void done_computing_with(ROBOT* bot)
{
  // remember state of globals from kilobot code
  bot->kilo_uid = kilo_uid;
  bot->kilo_ticks = (uint32_t)((bot->time_since_start + bot->time_since_last_time_update)*30.0);

 #ifdef codeSpaceA_exists
    if(bot->code_space == A)
    {
      codeSpaceA::g = NULL;
    }
  #endif 

  #ifdef codeSpaceB_exists
    if(bot->code_space == B)
    {
      codeSpaceB::g = NULL;
    }
  #endif


  #ifdef codeSpaceC_exists
    if(bot->code_space == C)
    {
      codeSpaceC::g = NULL;
    }
  #endif

  #ifdef codeSpaceD_exists
    if(bot->code_space == D)
    {
      codeSpaceD::g = NULL;
    }
  #endif

  #ifdef codeSpaceE_exists
    if(bot->code_space == E)
    {
      codeSpaceE::g = NULL;
    }
  #endif







  //check_state(swarm_, debug_log, 1, "simulation loop [%d], finished with %d", iii, bot->id);
}










//----------------------- COMMUNICATION STUFF -----------------------//





// copies the message data from msgA to msgB
void copy_message_from_to(message_t* msgA, message_t* msgB)
{
  //memcpy(msgB, msgA, sizeof(message_t));

  msgB->crc = msgA->crc;
  msgB->type = msgA->type;
  msgB->data[0] = msgA->data[0];
  msgB->data[1] = msgA->data[1];
  msgB->data[2] = msgA->data[2];
  msgB->data[3] = msgA->data[3];
  msgB->data[4] = msgA->data[4];
  msgB->data[5] = msgA->data[5];
  msgB->data[6] = msgA->data[6];
  msgB->data[7] = msgA->data[7];
  msgB->data[8] = msgA->data[8];
  //msgB->tag = msgA->tag;


}



// sends a message from the robot, note the simulation has each robot
// store a copy of its most recent outgoing message internally
void send_message_from(ROBOT* robot)
{
  //pthread_mutex_lock(&timing_mutext);
  now_computing_with(robot);

  //log_message_from_sim("robot %d sending", robot->id);

  message_t* msg = kilo_message_tx_wrapper(robot);
  if(msg != NULL)
  {
    copy_message_from_to(msg, robot->last_message_sent);
    //log_message_from_sim("||  data[0]: %u", msg->data[0]);
  }

  done_computing_with(robot);
  //pthread_mutex_unlock(&timing_mutext);
}

// init the distance measurment 
void init_distance_measurement(distance_measurement_t* dm)
{
  dm->distance_in_workspace_ratio = 10000.0f; // a large number
}

// set up things in the distance measurement, given the sending and recieveing robot)
void populate_distance_measurement(distance_measurement_t* dm, ROBOT* robot_send, ROBOT* robot_receive)
{
  dm->distance_in_workspace_ratio = dist_robots(robot_send, robot_receive);
}

// message A recieves the most recent message from robot B
void get_message_from(ROBOT* robotA, ROBOT* robotB)
{

  //log_message_from_sim("robot %d recieved message from robot %d", robotA->id, robotB->id);

  now_computing_with(robotA);

  populate_distance_measurement(&(robotA->incomming_distance_measurement), robotB, robotA);

  //log_message_from_sim("|  workspace ratio: %f", robotA->incomming_distance_measurement.distance_in_workspace_ratio);


  // copy message in case user decides to change data in it on the recieving end
  copy_message_from_to(robotB->last_message_sent, &(robotA->incomming_message_storage));

  //log_message_from_sim("|  data[0]: %u", robotB->last_message_sent->data[0]);


  kilo_message_rx_wrapper(robotA, &(robotA->incomming_message_storage), &(robotA->incomming_distance_measurement));
 
  done_computing_with(robotA);
}







// sets up the message times etc. for the entire swarm
void setup_message_times(SWARM* swarm, float message_time, float ave_inter_message_time, float max_inter_message_deviation)
{
  int i;
  float soonest_next_time = 100000000000.0f;
  for(i = 0; i < swarm->num_robots; i++)
  {
    swarm->robots[i]->inter_message_time = rand_f_between(ave_inter_message_time-max_inter_message_deviation, ave_inter_message_time+max_inter_message_deviation);

    swarm->robots[i]->message_time = message_time;
    swarm->robots[i]->next_message_send_at = rand_f_between(0.0f, ave_inter_message_time);
  
     soonest_next_time = fmin(soonest_next_time, swarm->robots[i]->next_message_send_at);
  }
  printf("first message sent at %f\n", soonest_next_time);
  swarm->next_event = soonest_next_time;
}





// this version is used to debug!!!! and sends all messages, even if they
// have colided with other messages
void do_communication_debug(SWARM* swarm, float now)
{
  // while there are events left
  while(swarm->next_event <= now)
  {
    //printf("pulse  %f  %f\n", swarm->next_event, now);

    int i, j;
    int num_com = 0;

    // see what robots have *just* started to send messages, send those messages
    for(i = 0; i < swarm->num_robots; i++)
    {
      swarm->robots[i]->sending = 1;

      if(swarm->robots[i]->next_message_send_at <= swarm->next_event)
      {

        // places the messages from i into robots[i]->last_message_sent
        send_message_from(swarm->robots[i]);




        // send to neighbors
        for(j = 0; j < swarm->robots[i]->num_neighbors; j++)
        {
          // sends j the message from robots[i]->last_message_sent
          get_message_from(swarm->robots[i]->neighbors[j], swarm->robots[i]);
        }

        // calculate next send time
        swarm->robots[i]->next_message_send_at += swarm->robots[i]->message_time + swarm->robots[i]->inter_message_time;

      }
      swarm->robots[i]->sending = 0;
    }
  

    // final pass, find the time of the next event
    float soonest_next_event = 100000000000.0f;
    for(i = 0; i < swarm->num_robots; i++)
    {
      // message sent at:
      soonest_next_event = fmin(soonest_next_event, swarm->robots[i]->next_message_send_at);
    }
    swarm->next_event = soonest_next_event;
  }
}




// this sends and recievs messages between robots, assuming that the current
// time is now
void do_communication(SWARM* swarm, float now)
{
  // while there are events left
  while(swarm->next_event <= now)
  {
    // in this pass through the outer , we are concerned with things that
    // have happened before and up to time swarm->next_event


    // first pass, mark any robot with communication that has just finished
    // and set its next communication time
    int i, j;
    int num_com = 0;
    for(i = 0; i < swarm->num_robots; i++)
    {
      if(swarm->robots[i]->sending == 1 && swarm->robots[i]->next_message_send_at + swarm->robots[i]->message_time <= swarm->next_event)
      {
        // this robot just finished, so find the next time that it sends
        swarm->robots[i]->sending = 2;
        swarm->robots[i]->next_message_send_at += swarm->robots[i]->message_time + swarm->robots[i]->inter_message_time;
      }
    }

    // second pass, for all nodes, see if any -neighbors- have just finished
    // sending, reset collisions to not happening if no neighbors are sending
    for(i = 0; i < swarm->num_robots; i++)
    {
      int num_neighbors_sending = 0;
      int num_neighbors_finished = 0;
      int index_of_finished_neighbor = -1;
      for(j = 0; j < swarm->robots[i]->num_neighbors; j++)
      {
        if(swarm->robots[i]->neighbors[j]->sending == 1)
        {
          num_neighbors_sending += 1;
        }
        else if(swarm->robots[i]->neighbors[j]->sending == 2)
        {
          num_neighbors_finished += 1;
          index_of_finished_neighbor = j;
        }
      }
 
      // if more than 1 neighbor has been sending or just finished, then mark that
      // collisions are happening
      if(num_neighbors_sending + num_neighbors_finished >= 2)
      {
        swarm->robots[i]->message_collision = 1;

        //log_message_from_sim("robot %d is blocked", i);
      }

      if(index_of_finished_neighbor >= 0)
      {
        if(swarm->robots[i]->message_collision == 0)
        {
          // i recieved a message from neighbor index_of_finished_neighbor
          swarm->total_recieved += 1;

          // -- do stuff for successfull coms here ---
          //log_message_from_sim("robot %d recieved message from robot %d", i, swarm->robots[i]->neighbors[index_of_finished_neighbor]->id);
          get_message_from(swarm->robots[i], swarm->robots[i]->neighbors[index_of_finished_neighbor]);
        }
        else
        {
          swarm->total_dropped += num_neighbors_finished;
        }
      }

      // if no neighbors are sending right now, then reset so that
      // no collisions are happening (moving forward in time)
      if(num_neighbors_sending == 0)
      {
        swarm->robots[i]->message_collision = 0;
      }    
    }


    // third pass, mark finished robots as not sending
    for(i = 0; i < swarm->num_robots; i++)
    {
      if(swarm->robots[i]->sending == 2)
      {
        swarm->robots[i]->sending = 0;
      }
    }

    // fourth pass, see what robots have just started to send messages
    for(i = 0; i < swarm->num_robots; i++)
    {
      if(swarm->robots[i]->sending == 0 && swarm->robots[i]->next_message_send_at <= swarm->next_event)
      {
        // this robot wants to start sending a message
        // check if it can sense any neighbors that are sending, if so, then 
        // it will wait a random ammount of time before sending
        int need_to_back_off = 0;
        for(j = 0; j < swarm->robots[i]->num_neighbors; j++)
        {
          if(swarm->robots[i]->neighbors[j]->sending == 1)
          {
            need_to_back_off = 1;
            break;
          }
        }

        if(need_to_back_off == 1)
        {
          // neighbor is sending, so this robot will back off a random
          // ammount of time
          swarm->total_backoffs++;

          swarm->robots[i]->next_message_send_at += rand_f_between(swarm->robots[i]->message_time, swarm->robots[i]->inter_message_time/2.0f);

          //printf("this %f\n", swarm->robots[i]->next_message_send_at);
        }
        else
        {
          swarm->robots[i]->sending = 1;

          // --- do stuff for i sending here ---

          //log_message_from_sim("robot %d sending message", i);
          send_message_from(swarm->robots[i]);



        }
      }
    }

  
    // final pass, find the time of the next event
    float soonest_next_event = 100000000000.0f;
    for(i = 0; i < swarm->num_robots; i++)
    {
      if(swarm->robots[i]->sending == 0)
      {
        // message sent at:
        soonest_next_event = fmin(soonest_next_event, swarm->robots[i]->next_message_send_at);
      }
      else if(swarm->robots[i]->sending == 1)
      {
        // message done being sent at
        soonest_next_event = fmin(soonest_next_event, swarm->robots[i]->next_message_send_at + swarm->robots[i]->message_time);

      }
      else
      {
        printf("ERROR! this should not happen\n");
      }
    }
    swarm->next_event = soonest_next_event;

  }
}







/*-------------------  stuff replacing kilolib.h functionality ---------------------------*/


// on kilobot hardware we need a message checksum
// we'll just return 0 in the simulation
uint16_t message_crc(const message_t* msg) 	
{
  return (uint16_t)0;
}


// required function that, in hardware, is called in the top of the kilobot's main look
void  kilo_init()
{

}

// required function that, in hardware, sets up the setup and loop functions
void kilo_start(void (*setup)(void), 
                void (*loop_ptr)(void), 
                void (*kilo_message_rx_ptr)(message_t *, distance_measurement_t *d),
                message_t *(*kilo_message_tx_ptr)(void),
                void (*kilo_message_tx_success_ptr)(void))
{

  ROBOT* bot = robot_local_copy_global_pass;            // store this here where it will not change
  kilo_user_globals[bot->id] = (void*)(*kilo_globals);  // kilo_globals was set in the kilobot's main() function

  done_computing_with(bot);


  // unlock the mutext (which as been locked prior to the call to
  // entering the kilobot's main() function (which then called this function)
  //pthread_mutex_unlock(&timing_mutext);


  // but wait for all robots to get here
  //pthread_barrier_wait(&barrier);

  // lock the mutext
  //pthread_mutex_lock(&timing_mutext); 

  now_computing_with(bot);

  // run setup
  if(bot->kilolib_setup == 0)
  {
    // save local pointers to the callbak functions of this robot
    bot->kilo_message_rx = kilo_message_rx_ptr;
    bot->kilo_message_tx = kilo_message_tx_ptr;
    bot->kilo_message_tx_success = kilo_message_tx_success_ptr;

    bot->kilo_loop = loop_ptr;


    (*setup)();
    bot->kilolib_setup = 1;
  }
  bot->need_to_run_loop = 1;

  done_computing_with(bot);


  // lock the mutext
  //pthread_mutex_unlock(&timing_mutext); 

/*
  // now enter infinite control loop
  while(true)
  {

    // lock the mutext
    pthread_mutex_lock(&timing_mutext); 


    // swap this robot into global space
    now_computing_with(bot);


    // do the computation loop for this robot
    (*(bot->kilo_loop))();  // run one loop


    delay(10);  // delay for 10 ms since this is where we are hiding some threading things in this function

    // save any values that have changed 
    done_computing_with(bot);

    // unlock the mutext
    pthread_mutex_unlock(&timing_mutext);


    if(debug_log->displaying_error_message == 1)
    {
      break;
    }
  }
*/
}



// just a dummy wrapper, since random numbers are being handled differently
void rand_seed(uint8_t seed)
{
  // nothing goes here
}

// returns a random number based on the time seed (since
// we have no voltage data in simulation
uint8_t rand_hard()
{
  if(seeded_the_rand == 0)
  {
    srand(time(NULL));
    seeded_the_rand = 1;
  }

  return (uint8_t)(rand() % 256);
}

// returns a random number
uint8_t rand_soft()
{
  return (uint8_t)(rand() % 256);
}

// gets and returns the (10 bit) ambiant light reading
// also sets the corresponding float value for the simulation display
int16_t get_ambientlight()
{
  return bot_global->ambient_light;
}

// sets the led based on the color
void set_color(uint8_t color)
{

  log_event(event_log, new_event_LEDChange(color));


 // bot_global->led_color[0] = ((float)(color & 3))/1.0f;
 // bot_global->led_color[1] = ((float)((color >> 2) & 3))/1.0f; 
 // bot_global->led_color[2] = ((float)((color >> 4) & 3))/1.0f;

}

// sets the led based on the color (process the event)
void set_color_event(EVENT_NODE* ev)
{

  uint8_t color = ev->color;

  bot_global->led_color[0] = ((float)(color & 3))/1.0f;
  bot_global->led_color[1] = ((float)((color >> 2) & 3))/1.0f; 
  bot_global->led_color[2] = ((float)((color >> 4) & 3))/1.0f;

}




// delays the current robot for this many ms
// we also pass control between different robot computations
// on a dely command
void delay(uint16_t ms)
{


/*
   struct timespec timeout_time, next_check_time, now_time;
   const int gettime_rv = clock_gettime(CLOCK_REALTIME, &timeout_time);
   next_check_time = timeout_time;

   // figure out what the time will be ms miliseconds from now
   long additional_sec = ((long)ms)/1000;
   long total_nsec =  timeout_time.tv_nsec + ((long)ms)*1000000;
   additional_sec += total_nsec/1000000000;
   timeout_time.tv_nsec = (total_nsec - additional_sec*1000000000);
   timeout_time.tv_sec += (uint)additional_sec;



   // remember globals from the kilobot workspace
   ROBOT* robot_local_copy = bot_global;
   done_computing_with(bot_global);

   // unlock the mutext
   pthread_mutex_unlock(&timing_mutext);

   // now sleep for desired time in slices of dt
   while(true)
   {
     total_nsec =  next_check_time.tv_nsec + (check_in_time_dt)*1000000;
     additional_sec += total_nsec/1000000000;
     next_check_time.tv_nsec = (total_nsec - additional_sec*1000000000);
     next_check_time.tv_sec += (uint)additional_sec;

     // sleep for dt
#ifdef _WIN32
     Sleep((int)check_in_time_dt);
#else
     usleep((useconds_t)check_in_time_dt);
#endif

     // after sleeping for dt, see if we are finished with the delay call
     const int gettime_rv = clock_gettime(CLOCK_REALTIME, &now_time);
     if(timeout_time.tv_sec < now_time.tv_sec || (timeout_time.tv_sec == now_time.tv_sec && timeout_time.tv_nsec <= now_time.tv_nsec))
     {
       // we are finished with the delay call
       break;
     }
     // otherwise we are not finished with the delay call, 
     // so the loop continues
   }

   pthread_barrier_wait(&barrier);

   // lock the mutext
   pthread_mutex_lock(&timing_mutext); 

   // populate the globals of the kilobot workspace
   now_computing_with(robot_local_copy);
*/

  //log_message_from_sim("------- in delay %d -----", bot_global->id);

   bot_global->time_since_last_time_update += (((double)ms) / 1000.0);
   bot_global->kilo_ticks = (uint32_t)((bot_global->time_since_start + bot_global->time_since_last_time_update)*30.0);
   kilo_ticks = bot_global->kilo_ticks;

}







// delays the current thread for this many ms
// we also pass control between different robot computations here
void delay_thread(uint16_t ms)
{
   struct timespec timeout_time, next_check_time, now_time;
   const int gettime_rv = clock_gettime(CLOCK_REALTIME, &timeout_time);
   next_check_time = timeout_time;

   // figure out what the time will be ms miliseconds from now
   long additional_sec = ((long)ms)/1000;
   long total_nsec =  timeout_time.tv_nsec + ((long)ms)*1000000;
   additional_sec += total_nsec/1000000000;
   timeout_time.tv_nsec = (total_nsec - additional_sec*1000000000);
   timeout_time.tv_sec += (uint)additional_sec;


   // now sleep for desired time in slices of dt
   while(true)
   {
     total_nsec =  next_check_time.tv_nsec + (check_in_time_dt)*1000000;
     additional_sec += total_nsec/1000000000;
     next_check_time.tv_nsec = (total_nsec - additional_sec*1000000000);
     next_check_time.tv_sec += (uint)additional_sec;

     // sleep for dt
#ifdef _WIN32
     Sleep((int)check_in_time_dt);
#else
     usleep((useconds_t)check_in_time_dt);
#endif

     // after sleeping for dt, see if we are finished with the delay call
     const int gettime_rv = clock_gettime(CLOCK_REALTIME, &now_time);
     if(timeout_time.tv_sec < now_time.tv_sec || (timeout_time.tv_sec == now_time.tv_sec && timeout_time.tv_nsec <= now_time.tv_nsec))
     {
       // we are finished with the delay call
       break;
     }
     // otherwise we are not finished with the delay call, 
     // so the loop continues
   }
}







// returns the distance that is stored in the distance measurement
uint8_t estimate_distance(const distance_measurement_t *dist) 
{

  int dist_int_mm = (int)(dist->distance_in_workspace_ratio * DISTANCE_RATIO_MULT);
  if(dist_int_mm >= 128)
  {
    dist_int_mm = 127;
  }

  uint8_t dist_mm = (uint8_t)(dist_int_mm);
  return dist_mm;

    // this is the original code from kilolib.h, in case we ever need to reverse engineer how
    // the gains in the original distance_measurement_t translate to distance
    /*
    uint8_t i;
    uint8_t index_high=13;
    uint8_t index_low=255;
    uint8_t dist_high=255;
    uint8_t dist_low=255;

    if (dist->high_gain < 900) {
        if (dist->high_gain > kilo_irhigh[0]) {
            dist_high=0;
        } else {
            for (i=1; i<14; i++) {
                if (dist->high_gain > kilo_irhigh[i]) {
                    index_high = i;
                    break;
                }
            }

            double slope=(kilo_irhigh[index_high]-kilo_irhigh[index_high-1])/0.5;
            double b=(double)kilo_irhigh[index_high]-(double)slope*((double)index_high*(double)0.5+(double)0.0);
            b=(((((double)dist->high_gain-(double)b)*(double)10)));
            b=((int)((int)b/(int)slope));
            dist_high=b;
        }
    }

    if (dist->high_gain > 700) {
        if (dist->low_gain > kilo_irlow[0]) {
            dist_low=0;
        } else {
            for(i=1; i<14; i++) {
                if(dist->low_gain > kilo_irlow[i]) {
                    index_low = i;
                    break;
                }
            }

            if(index_low == 255) {
                dist_low=90;
            } else {
                double slope=(kilo_irlow[index_low]-kilo_irlow[index_low-1])/0.5;
                double b=(double)kilo_irlow[index_low]-(double)slope*((double)index_low*(double)0.5+(double)0.0);
                b=(((((double)dist->low_gain-(double)b)*(double)10)));
                b=((int)((int)b/(int)slope));
                dist_low=b;
            }
        }
    }

    if (dist_low != 255) {
        if (dist_high != 255) {
            return 33 + ((double)dist_high*(900.0-dist->high_gain)+(double)dist_low*(dist->high_gain-700.0))/200.0;
        } else {
            return 33 + dist_low;
        }
    } else {
        return 33 + dist_high;
    }

    */
}



// sets the motor speed. Note that according to kilobot design this should only
// be called in 1 of 4 different ways:
// go straight : set_motors(kilo_straight_left, kilo_straight_right)
// turn right  : set_motors(kilo_turn_left, 0)
// turn left   : set_motors(0, kilo_turn_right)
// stop        : set_motors(0, 0)
void set_motors(uint8_t left, uint8_t right)
{
  log_event(event_log, new_event_MOTORChange(left, right));

/*
  if((left < kilo_straight_left && left < kilo_turn_left) && (right < kilo_straight_right && right < kilo_turn_right))
  {
    // STOP
    // robot cannot maintain speed, so it stops
    bot_global->speed[0] = 0.0f;   // set left wheel speed to 0
    bot_global->speed[1] = 0.0f;   // set right wheel speed to 0
  }
  else if((left < kilo_straight_left && left < kilo_turn_left) && (right > kilo_straight_right || right > kilo_turn_right))
  {
    // BANK LEFT
    // left motor is ~stopped and right motor is on
    bot_global->speed[0] = 0.0f;                 // set left wheel speed to 0
    bot_global->speed[1] = ((float)right)/128.f; // set right wheel speed
  }
  else if((left > kilo_straight_left || left > kilo_turn_left) && (right < kilo_straight_right && right < kilo_turn_right))
  {
    // BANK RIGHT
    // right motor is ~stopped and right motor is on
    bot_global->speed[0] = ((float)left)/128.f;  // set right wheel speed
    bot_global->speed[1] = 0.0f;                 // set right wheel speed to 0
  }
  else
  {
    // GO STRAIGHT
    // both motors are on
    bot_global->speed[0] = ((float)left)/128.f;  // set right wheel speed
    bot_global->speed[1] = ((float)right)/128.f;  // set right wheel speed
  }

  bot_global->motors_spun_up = 0;

  */
}



// processes the event stored above
void set_motors_event(EVENT_NODE* ev)
{

 uint8_t left = ev->left;
 uint8_t right = ev->right;

 if((left < kilo_straight_left && left < kilo_turn_left) && (right < kilo_straight_right && right < kilo_turn_right))
  {
    // STOP
    // robot cannot maintain speed, so it stops
    bot_global->speed[0] = 0.0f;   // set left wheel speed to 0
    bot_global->speed[1] = 0.0f;   // set right wheel speed to 0
  }
  else if((left < kilo_straight_left && left < kilo_turn_left) && (right > kilo_straight_right || right > kilo_turn_right))
  {
    // BANK LEFT
    // left motor is ~stopped and right motor is on
    bot_global->speed[0] = 0.0f;                 // set left wheel speed to 0
    bot_global->speed[1] = ((float)right)/128.f; // set right wheel speed
  }
  else if((left > kilo_straight_left || left > kilo_turn_left) && (right < kilo_straight_right && right < kilo_turn_right))
  {
    // BANK RIGHT
    // right motor is ~stopped and right motor is on
    bot_global->speed[0] = ((float)left)/128.f;  // set right wheel speed
    bot_global->speed[1] = 0.0f;                 // set right wheel speed to 0
  }
  else
  {
    // GO STRAIGHT
    // both motors are on
    bot_global->speed[0] = ((float)left)/128.f;  // set right wheel speed
    bot_global->speed[1] = ((float)right)/128.f;  // set right wheel speed
  }

  bot_global->motors_spun_up = 0;
}




// on the real kilobots it is advisable to call this function 
// if the kilobot is starting from a stopped position, 
// the mechanical effect is to break static friction with 
// the surface on which the kilobot is sitting.
void spinup_motors()
{
  bot_global->motors_spun_up = 1;
}




// remembers when we need to run the next loop for this robot
void enable_loop()
{
  log_event(event_log, new_event_LOOPCall());
}



// sets a flag so that the robot is allowed to run a single loop event in the future
void enable_loop_event(EVENT_NODE* ev)
{
  bot_global->need_to_run_loop = 1;

}


#endif //__KILOLIB_SIMULATION_C__

