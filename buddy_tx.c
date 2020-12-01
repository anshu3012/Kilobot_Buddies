//#include "kilolib.h"

struct GLOBALS
{
  uint16_t wait_time;
  message_t message;
  int messages_sent;
  int buddy_flag;
  int recieved_buddy_flag;
  int distance;
  int messages_received;
  int no_buddy;
  int id;
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
  g->buddy_flag=0;
  g->recieved_buddy_flag=0;
  g->id=0;
}



// send message function
message_t *message_tx()
{

    g->message.type = NORMAL;
    g->message.data[0] = 0; // transmits I dont have a buddy

    if (g->distance<25)
    {
      g->message.data[0] = 1; // transmits I have a buddy
    }

    g->message.crc = message_crc(&(g->message));
    return &(g->message);
 
}


// message success callback (called *after* a message is sent)
void message_tx_success()
{
  g->messages_sent += 1;
}


void message_rx(message_t *m, distance_measurement_t *d)
{
  g->distance = estimate_distance(d);
}



// put your main code here, will be run repeatedly
void loop() 
{
  set_color(RGB(1,0,0));
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

