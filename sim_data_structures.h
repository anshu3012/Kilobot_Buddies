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

#ifndef __SIM_DATA_STRUCTURES_H__
#define __SIM_DATA_STRUCTURES_H__

#include <string>
#include <stdarg.h>
#include <stdio.h>

#include "kilolib/message.h"
#include "kilolib/message_crc.h"
#include <vector>

/*------------------------------- COLORS --------------------------------*/
float WHITE[] = {1,1,1,1};
float BLACK[] = {0,0,0,1};
float DARK[] = {.1,.1,.1,1};
float DARKGRAY[] = {.3,.3,.3,1};
float GRAY[] = {.5,.5,.5,1};
float SOMEGRAY[] = {.6,.6,.6,1};
float LIGHTGRAY[] = {.8,.8,.8,1};
float LIGHT[] = {.9,.9,.9,1};
float RED[] = {1,0,0,1};
float BLUE[] = {0,0,1,1};
float GREEN[] = {0,1,0,1};
float MEDIUMGREEN[] = {0,.7,0,1};
float DARKGREEN[] = {0,.5,0,1};

/* ---------------------------- globals for drawing ---------------------*/

int display_com_rads = 0;
int display_graph = 0;
int display_debugging_log = 1;

int display_messages = 1;
int selected_robot = -1;     // used to select a robot with the mouse
int curr_light_pattern = -1; // used to toggle between the light
                             // pattern that is currently on



/*-------------------- DATA STRUCTURE TYPEDEFS --------------------------*/

struct ROBOT;
typedef struct ROBOT ROBOT;

struct SWARM;
typedef struct SWARM SWARM;

struct LIGHTPROJECTOR;
typedef struct LIGHTPROJECTOR LIGHTPROJECTOR;

struct PARAMS;
typedef struct PARAMS PARAMS;

struct DEBUG_LOG;
typedef struct DEBUG_LOG DEBUG_LOG;

enum EventType {LEDChange, MOTORChange, LOOPCall, NULLEvent};
struct nodeT   // this holds the events
{
  double time;
  EventType type;
  int robot_id;
  int event_id;

  uint8_t color;
  uint8_t left;
  uint8_t right;

  bool inHeap;
  int heapIndex;
};
typedef struct nodeT nodeT;
typedef nodeT EVENT_NODE;

struct EVENT_LOG;
typedef struct EVENT_LOG EVENT_LOG;


#include "sbbsHeap.h"


/*----------------------- ROBOT DATA STRUCTURE --------------------------*/
struct ROBOT
{
  float radius;
  float com_radius;
  int buddy_flag;

  float position[3];          // currently just x and y, the z is depricited from an older version of openGL, in which case it was
                              // the buffer height, but left here incase we every want to use it for height in a 3D environment
  float orientation[1];       // theta, but stored like this in case we ever want to move to 3D

  float speed[3];             // currenly just left and right "wheel" speeds
  float d_orientation[1];     // derivitive of orientation

  int motors_spun_up;         // flag to indicate if motors have been spun up or not

  float color[3];
  int id;
  CodeSpace code_space;       // sotres an enumerated value to indicate which code to use

  int num_neighbors;          // number of robots that this robot can hear
  ROBOT** neighbors;          // pointers to robots that this robot can hear

  float inter_message_time;   // this is how long the robot waits after finishing
                              // sending a message before it sends a new message
  float message_time;         // the ammount of time it takes to send a message
  float next_message_send_at; // the time that the next message will be sent at

  int sending;                // 1 if sending now, 0 if not
  int message_collision;      // set to 1 when there is a collision
  message_t* last_message_sent;// the last message sent by this robot

  float pose_min[2];          // max pose allowed
  float pose_max[2];          // min pose allowed

  double start_time;
  double time_since_start;
  double time_since_last_time_update;
  int need_to_run_loop;       // set to 0 after loop is run

  float visual_light_input;   // sensor input from visual light (this is the internal floating point value)

  uint16_t kilo_uid;          // the kilolib id of this robot  
  uint32_t kilo_ticks;        // how long the robot has been running, incremented approximately 32 times per second, or once every 30ms
 
  uint8_t send_message_type;  // toggles throguh message this robot sends

  int kilolib_setup;          // set to one once setup() equilivant has been run

  float led_color[4];         // rgb for this robots color     

  int16_t ambient_light;      // a storage place for the ambiant light this robot will measure from its sensor


  // function pointers to things the kilobot expects to exist
  void (*kilo_message_rx)(message_t *, distance_measurement_t *d);
  message_t *(*kilo_message_tx)(void);
  void (*kilo_message_tx_success)(void);
  void (*kilo_loop)(void);   

  message_t incomming_message_storage;  // memory that will store a recieved message
  distance_measurement_t incomming_distance_measurement;

  GLuint circle_mem_offset;   // in the global draw buffer   (in verticies)
  //GLfloat* draw_circle_verticies;    // points to this robot's circle's position in the vertex array used
                                     // to draw the robot 
                                     // NOTE: this must be dealocated by the outer loop and not by the robot

  GLuint disc_mem_offset;     // in the global draw buffer   (in verticies)
  //GLfloat* draw_disc_verticies;      // points to this robot's triangle's position in the vertex array used 
                                     // to draw the robot
                                     // NOTE: this must be dealocated by the outer loop and not by the robot

};




/*----------------------- SWARM DATA STRUCTURE --------------------------*/
struct SWARM
{
  float pose_min[2];
  float pose_max[2];

  float robot_radius; // all have the same radius
  float com_radius;   // all have the same (for now)

  int num_robots;     // number of robots in swarm
  ROBOT** robots;     // vector robot pointers

  float next_event;   // the time that the next event happens

  int total_dropped;  // total coms that have been dropped
  int total_recieved; // total coms that have been recieved
  int total_backoffs; // total coms that have waited

  float ave_error;    // last calculated average error over all robots
  float max_error;    // last calculated max error over all robots
  float class_error;  // last calculated classification error over all robots



  // the following use the convention [from][to]
  message_t last_message_sent[NUM_ROBOTS][NUM_ROBOTS];
  distance_measurement_t last_message_sent_dist[NUM_ROBOTS][NUM_ROBOTS];
  int last_message_sent_exists[NUM_ROBOTS][NUM_ROBOTS];
  std::vector< std::pair <int,int> > vect;

};






/*---------------- LIGHTPROJECTOR DATA STRUCTURE --------------------*/



// this holds a "visual light projector"
// that is used to provide visual light sensor input data to the swarm
// note that "pixels" are pixels of light that are given a particular
// value by the projector, note that pixels are given coordinates
// similar to the viewer (x, y) increasing to the right/up and NOT (row col) 
// in order to make math between screen coords and matrix pixels nice
struct LIGHTPROJECTOR
{
  int pixel_rows;     // this many "pixels" up 
  int pixel_cols;     // this many "pixels" down

  float** light_data; // pixel_cols by pixel_rows matric holding visual light data

};


/*---------------- PARAMS DATA STRUCTURE --------------------*/

struct PARAMS
{
  SWARM* swarm;
  int id;
};

/*---------------- DEBUG_LOG DATA STRUCTURE --------------------*/


#define DEBUG_LINES 20
#define DEBUG_CHARS_PER_LINE 1000
#define EXTRA_CHARS_PER_LINE 100   // used for the logging tag the simulator adds to each message
struct DEBUG_LOG
{
  char** ring_buffer; // each message is stored in its own string

  int total_lines;
  int current_line;

  char* log_formatted; // this is where we put everything  so it can be passed to javascript


  int displaying_error_message;   // starts as 1, set to 0 if an error message is displayed so that log messages do not wipe it out

};


// adds the message to the log data
void log_message_raw(int mode, DEBUG_LOG* DL, const char* message);


// adds the message to the log data, but works like sprintf
// (which requires using arg lists, see info about <stdarg.h>
// for more details
void log_message(const char *format, ...);

// same as above, bur from the simulator
void log_message_from_sim(const char *format, ...);



// adds the error message to the log data and posts it ASAP
// then disables new messages
void error_message_raw(int mode, DEBUG_LOG* DL, const char* message);


// displays an error message 
void error_message(const char *format, ...);


// same as above, bur from the simulator
void error_message_from_sim(const char *format, ...);

/*---------------- EVENT_NODE (nodeT) DATA STRUCTURE --------------------*/

EVENT_NODE* new_event_LEDChange(uint8_t color);

EVENT_NODE* new_event_MOTORChange(uint8_t left, uint8_t right);

EVENT_NODE* new_event_LOOPCall();

/*---------------- EVENT_LOG DATA STRUCTURE --------------------*/

struct EVENT_LOG
{
  sbbsHeap* H;

  double time_start;
  int event_count;
};



void log_event(EVENT_LOG* el, EVENT_NODE* ev);





#endif //__SIM_DATA_STRUCTURES_H__
