//#include "kilolib.h"



struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  message_t message;

  // Flag to keep track of message transmission.
  int message_sent;
}* g; // there should only be one GLOBAL, this is it, remember to register it in main()



void setup()
{
    // Initialize message 
    // (note that because we are never changing the data in the message in this example
    // then we only need to do this once, otherwise we'd need to do it every time we 
    // wanted to send a new message)

    // The type is always NORMAL.
    g->message.type = NORMAL;

    // indicate we are a star
    g->message.data[0] = 2; 

    // It's important that the CRC is computed after the data has been set;
    // otherwise it would be wrong.
    g->message.crc = message_crc(&g->message);
}

void loop()
{

    // Blink LED magenta whenever a message is sent.
    if (g->message_sent == 1)
    {
        // Reset flag so LED is only blinked once per message.
        g->message_sent = 0;
        
        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }


    log_message("Im a star");
    delay(100);

}


message_t *message_tx()
{
    //log_message_from_sim("|| S  data[0]: %u", g->message.data[0]);

    return &(g->message);
}


void message_tx_success()
{
    // Set flag on message transmission.
    g->message_sent = 1;
}


int main()
{
    GLOBALS* g_safe = (GLOBALS*)malloc(sizeof(GLOBALS));

    #ifdef USING_SIMULATION
      // register the global variables (only necessary on simulation)
      kilo_globals = (void**)&g_safe;
    #endif

    kilo_init();

    // Register the message_tx callback function.
    kilo_message_tx = message_tx;

    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;

    g = g_safe;

    #ifdef USING_SIMULATION
      kilo_start(setup, loop, NULL, message_tx, message_tx_success);
    #else
      kilo_start(setup, loop);
      // free user defined globals
      free(g_safe);
    #endif
    

    return 0;
}

