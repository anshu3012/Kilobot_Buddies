//#include "kilolib.h"

// Constants for orbit control.
#define TOO_CLOSE_DISTANCE 35
#define TOO_FAR_DISTANCE 50

// Constants for motion handling function.
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3



struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  int current_motion;
  int distance;
  int new_message;

  message_t outgoing_message;

}* g; // there should only be one GLOBAL, this is it, remember to register it in main()


// put your setup code here, will be run once at the beginning
// this is a good place, for example, to define the intial values
// of global variables
void setup()
{
  g->current_motion = STOP;
  g->distance = 10000;
  g->new_message = 0;


  // (note that because we are never changing the data in the message in this example
  // then we only need to do this once, otherwise we'd need to do it every time we 
  // wanted to send a new message)

  // The type is always NORMAL.
  g->outgoing_message.type = NORMAL;

  // indicate we are a planet
  g->outgoing_message.data[0] = 1;

  // It's important that the CRC is computed after the data has been set;
  // otherwise it would be wrong.
  g->outgoing_message.crc = message_crc(&g->outgoing_message);

}





// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (g->current_motion != new_motion)
    {
        g->current_motion = new_motion;
        
        if (g->current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (g->current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (g->current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (g->current_motion == LEFT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}


void loop()
{
    // Update the motion whenever a message is received.
    if (g->new_message == 1)
    {
        g->new_message = 0;
        
        if(g->distance < TOO_CLOSE_DISTANCE)
        {
          // If too close, turn left to face away

          set_color(RGB(1, 0, 0));
          set_motion(LEFT);

          delay(100);
          set_motion(FORWARD);
          delay(200);

        }
        else if(g->distance > TOO_FAR_DISTANCE)
        {
          // If not too far turn right to face toward

          set_color(RGB(0, 0, 1));
          set_motion(RIGHT);

          delay(200);
          set_motion(FORWARD);
          delay(200);
        }
        //star planet situation where you want the planets to stop if within a certain distance to star
        else if(g->distance==25 || g->distance==26||g->distance==27||g->distance==27||g->distance==29||g->distance==30||g->distance==31||g->distance==32||g->distance==33)
        {
          set_motion(STOP);
        }
        else
        {
           // in ok region, so move straight
           set_color(RGB(0, 1, 0));
           set_motion(FORWARD);

           delay(400);
        }
    }

    log_message("Im a planet: %d", (int) g->distance);

    set_color(RGB(0, 0, 0));
    set_motion(STOP);

}


// recieve message callback function
void message_rx(message_t *m, distance_measurement_t *d)
{

    //log_message_from_sim("| P  data[0]: %u", m->data[0]);

    if(m->data[0] == 2)
    {
      // then the message was from the star

      g->new_message = 1;
      g->distance = estimate_distance(d);
    }
}

// send message function
message_t *message_tx()
{
    //log_message_from_sim("|| P  data[0]: %u", g->outgoing_message.data[0]);


    return &(g->outgoing_message);
}


// sucessfull send message callback
void message_tx_success()
{
  // do nothing
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

