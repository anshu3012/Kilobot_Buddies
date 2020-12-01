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

#ifndef __KILOLIB_SIMULATION_H__
#define __KILOLIB_SIMULATION_H__


#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)


enum CodeSpace {A, B, C, D, E, None};

CodeSpace codeSpaceMap[NUM_ROBOTS];


typedef unsigned char uint8_t;


#include "kilolib/message.h"

namespace codeSpaceA
{
  struct GLOBALS;
  typedef struct GLOBALS GLOBALS;
} // namespace codeSpaceA

namespace codeSpaceB
{
  struct GLOBALS;
  typedef struct GLOBALS GLOBALS;
} // namespace codeSpaceB

namespace codeSpaceC
{
  struct GLOBALS;
  typedef struct GLOBALS GLOBALS;
} // namespace codeSpaceC

namespace codeSpaceD
{
  struct GLOBALS;
  typedef struct GLOBALS GLOBALS;
} // namespace codeSpaceD

namespace codeSpaceE
{
  struct GLOBALS;
  typedef struct GLOBALS GLOBALS;
} // namespace codeSpaceE



typedef struct {
    float distance_in_workspace_ratio;
    
  // the following is what was originally in here from kilolib.h
  // (in case we ever want to make this the same be reverse engineering how the gains work)
  /*
  int16_t low_gain;  ///< Low gain 10-bit signal-strength measurement.
  int16_t high_gain; ///< High gain 10-bit signal-strength measurement.
  */
} distance_measurement_t;





void** kilo_globals = NULL; // used to register user globals for sim interaction, only used once per each robot



typedef void (*message_rx_t)(message_t *, distance_measurement_t *d);
typedef message_t *(*message_tx_t)(void);
typedef void (*message_tx_success_t)(void);


void log_message_from_sim(const char *format, ...);


// default functions in case user does not set them up (copied from kilolib.h)
void message_rx_dummy(message_t *m, distance_measurement_t *d) 
{ 
  log_message_from_sim("| D data[0]: %u", m->data[0]);
}

message_t *message_tx_dummy() 
{ 
  log_message_from_sim("|| D ");

  return NULL; 
}
void message_tx_success_dummy() 
{


}

// function pointers assumed to exist for message callback functions, set to default dummy values
message_rx_t kilo_message_rx = message_rx_dummy;
message_tx_t kilo_message_tx = message_tx_dummy;
message_tx_success_t kilo_message_tx_success = message_tx_success_dummy;



// these globals are expected to exist in the global namespace by the kilobot code
uint16_t kilo_uid = -1;
uint32_t kilo_ticks = 0;             // how long the robot has been running, incremented approximately 32 times per second, or once every 30ms
uint8_t  kilo_turn_left      = 120;
uint8_t  kilo_turn_right     = 120;
uint8_t  kilo_straight_left  = 100; 
uint8_t  kilo_straight_right = 100;


// -------------------- function headers ---------------------------------





uint16_t message_crc(const message_t *msg);



// required function that, in hardware, is called in the top of the kilobot's main look
void  kilo_init();


// required function that, in hardware, sets up the setup and loop functions
void kilo_start(void (*setup)(void), 
                void (*loop_ptr)(void), 
                void (*kilo_message_rx_ptr)(message_t *, distance_measurement_t *d),
                message_t *(*kilo_message_tx_ptr)(void),
                void (*kilo_message_tx_success_ptr)(void));






// dummy wrapper that always returns 0
uint16_t message_crc(const message_t* msg);

// just a dummy wrapper, since random numbers are being handled differently
void rand_seed(uint8_t seed);


// returns a random number based on the time seed (since
// we have no voltage data in simulation
uint8_t rand_hard();

// returns a random number
uint8_t rand_soft();

// gets and returns the (10 bit) ambiant light reading
// also sets the corresponding float value for the simulation display
int16_t get_ambientlight();

// sets the led based on the color
void set_color(uint8_t color);

// delays the current robot for this many ms
// we also pass control between different robot computations
// on a dely command
void delay(uint16_t ms);

// returns the distance (in mm) that is stored in the distance measurement
uint8_t estimate_distance(const distance_measurement_t *dist);


// sets the motor speed. Note that according to kilobot design this should only
// be called in 1 of 4 different ways:
// go straight : set_motors(kilo_straight_left, kilo_straight_right)
// turn right  : set_motors(kilo_turn_left, 0)
// turn left   : set_motors(0, kilo_turn_right)
// stop        : set_motors(0, 0)
void set_motors(uint8_t left, uint8_t right);

// on the real kilobots it is advisable to call this function 
// if the kilobot is starting from a stopped position, 
// the mechanical effect is to break static friction with 
// the surface on which the kilobot is sitting.
void spinup_motors();







#endif //__KILOLIB_SIMULATION_C__
