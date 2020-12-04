//#include "kilolib.h"

struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  uint16_t wait_time;
  int state;

}* g; // there should only be one GLOBAL, this is it, remember to register it in main()


// put your setup code here, will be run once at the beginning
// this is a good place, for example, to define the intial values
// of global variables
void setup()
{
  g->wait_time = 2000;
  g->state = 0;
}




// put your main code here, will be run repeatedly
void loop() 
{

  if(g->state == 0)
  {
    // in this case we are starting the motors from being stopped
    // so it is advisable to spin up the motors to overcome static friction
    spinup_motors();

    // RIGHT TURN (requires left motor on)
    set_motors(kilo_turn_left, 0);

    set_color(RGB(1,0,1));
  }
  else if(g->state == 1)
  {
    // GO STRAIGHT (requires both motors on)
    set_motors(kilo_straight_left, kilo_straight_right);

    set_color(RGB(0,0,1));
  }
  else if(g->state == 2)
  {
    // LEFT TURN (requires right motor on)
    set_motors(0, kilo_turn_right);

    set_color(RGB(0,1,1));
  }
  else if(g->state == 3)
  {
    set_color(RGB(1,1,1));

    // STOP (requires both motors off)
    set_motors(0, 0);
  }

  // incriment state
  g->state += 1;
  if(g->state >= 4)
  {
    g->state = 0;
  }


  float rand_val = ((float)rand_soft())/(256.0f);

  g->wait_time = ((int16_t) (2000.0f * rand_val));  // wait a random ammount of time between 0 and 2 seconds

  // delay for a bit
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


    g = g_safe;

    #ifdef USING_SIMULATION
      kilo_start(setup, loop, NULL, NULL, NULL);
    #else
      kilo_start(setup, loop);
      // free user defined globals
      free(g_safe);
    #endif
  
  
    return 0;
}

