//#include "kilolib.h"

struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  uint16_t wait_time;
  message_t message;
  int messages_sent;

  int distance;
  int messages_received;

}* g; // there should only be one GLOBAL, this is it, remember to register it in main()


// put your setup code here, will be run once at the beginning
// this is a good place, for example, to define the intial values
// of global variables
void setup()
{
  g->wait_time = 500;
  g->messages_sent = 0;
  g->distance = 1000;
  g->messages_received = 0;
}



// send message function
message_t *message_tx()
{
  // Initialize message:
  // The type is always NORMAL.
  g->message.type = NORMAL;

  g->message.data[0] = kilo_uid; // send this robots ID to the other robot*/

  // It's important that the CRC is computed after the data has been set;
  // otherwise it would be wrong.
  g->message.crc = message_crc(&(g->message));


  return &(g->message);
}

// message success callback (called *after* a message is sent)
void message_tx_success()
{
  g->messages_sent += 1;
}

// message recieve callback
void message_rx(message_t *m, distance_measurement_t *d)
{
  g->messages_received += 1;

  // reset the number of loops since we heard from the sending robot
  int sending_robot_id = m->data[0];  
  
  if( g->distance > estimate_distance(d))
  {
    g->distance = estimate_distance(d);
  }
}


// put your main code here, will be run repeatedly
void loop() 
{

  if(g->distance < 0)
  {
    set_color(RGB(1,0,1));
  }
  else if(g->distance < 30)
  {
    set_color(RGB(0,0,1));
  }
  else if(g->distance < 35)
  {
    set_color(RGB(0,1,1));
  }
  else if(g->distance < 40)
  {
    set_color(RGB(0,1,0));
  }
  else if(g->distance < 45)
  {
    set_color(RGB(1,1,0));
  }
  else if(g->distance < 50)
  {
    set_color(RGB(1,0,0));
  }
  else
  {
    set_color(RGB(1,1,1));
  }

  log_message("distance to neighbor: %d mm", (int)(g->distance ));
  log_message("ID of neighbor: %d mm", (int)(g->message.data[0] ));
  


  g->distance = 1000; // reset distance



  delay(g->wait_time);
}



int main()
{
    GLOBALS* g_safe =  (GLOBALS*)malloc(sizeof(GLOBALS));

    #ifdef USING_SIMULATION
      // register the global variables (only necessary on simulation)
      kilo_globals = (void**)&g;
    #endif

    kilo_init();


    // Register the message_rx callback function.
    kilo_message_rx = message_rx;

    // Register the message_tx callback function.
    kilo_message_tx = message_tx;

    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;

    g = g_safe;

    #ifdef USING_SIMULATION
      kilo_start(setup, loop, message_rx, message_tx, message_tx_success);
    #else
      kilo_start(setup, loop);
      // free user defined globals
      free(g_safe);
    #endif
  
  
    return 0;
}

