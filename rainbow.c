//#include "kilolib.h"

struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  uint16_t wait_time;
}* g; // there should only be one GLOBAL, this is it, remember to register it in main()



// put your setup code here, will be run once at the beginning
// this is a good place, for example, to define the intial values
// of global variables
void setup()
{
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot
  g->wait_time = 300;
}

// put your main code here, will be run repeatedly
void loop() 
{

  if(kilo_uid == 0)
    set_color(RGB(0,0,1));
  else if(kilo_uid == 1)
    set_color(RGB(0,1,0));
  else if(kilo_uid == 2)
    set_color(RGB(1,0,0));
  else
    set_color(RGB(1,1,1)); 

  delay(g->wait_time);


  set_color(RGB(0,0,1));
  delay(g->wait_time);
  set_color(RGB(0,1,1));
  delay(g->wait_time);
  set_color(RGB(0,1,0));
  delay(g->wait_time);
  set_color(RGB(1,1,0));
  delay(g->wait_time);
  set_color(RGB(1,0,0));
  delay(g->wait_time);
  set_color(RGB(1,0,1));
  delay(g->wait_time);



  set_color(RGB(1,1,1)); 

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


