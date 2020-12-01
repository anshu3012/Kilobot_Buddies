//#include "kilolib.h"
struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.


  int16_t light_value;
}* g; // there should only be one GLOBAL, this is it, remember to register it in main()


// put your setup code here, will be run once at the beginning
void setup()
{
  g->light_value = 0;
}


// returns the average light over num_samples sucessfull samples
int16_t sample_light(int16_t num_samples) 
{
  int16_t num_valid_samples = 0;
  long average = 0;

  while(num_valid_samples < num_samples) 
  {
    int16_t sample = get_ambientlight();
    if(sample != -1) 
    {
      average += sample;
      num_valid_samples++;
    }
  }
  return average / num_samples;
}


// sets the hue based on color value (0 to 1023)
void set_color_from_value(int16_t value)
{
  if(value < 300)
  {
    set_color(RGB(0,0,1));
  }
  else if(300 <= value && value < 450)
  {
    set_color(RGB(0,1,1));
  }
  else if(450 <= value && value < 600)
  {
    set_color(RGB(0,1,0));
  }
  else if(600 <= value && value < 750)
  {
    set_color(RGB(1,1,0));
  }
  else if(750 <= value && value < 900)
  {
    set_color(RGB(1,0,0));
  }
  else //900 <= value
  {
    set_color(RGB(1,0,1));
  }
}


// put your main code here, will be run repeatedly
void loop() 
{

  // set light based on that value
  set_color_from_value(g->light_value);

  // delay for a bit (0.1 sec)
  delay(100);

  // take new sample (average over 30 individual samples)
  g->light_value = sample_light(30);


  // delay for a bit (0.1 sec)
  delay(100);

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
