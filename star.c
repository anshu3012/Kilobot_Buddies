//#include <kilolib.h>

// declare variables

struct GLOBALS
{
  // global variables 
  // NOTE: the use of a GLOBALS struct will also work on a normal kilobot, 
  //       but is not required on a normal kilobot.
  //       It is, however, required by the simulator.

  uint8_t message_sent = 0;

  // Flag to keep track of message transmission.
  message_t msg;
}*g;

void setup() {
    // initialize message
    g->msg.type = NORMAL;
    g->msg.crc = message_crc(&(g->msg));
}

void loop() {
    // blink red when message is sent
    if (g->message_sent) {
        g->message_sent = 0;
        set_color(RGB(1,0,0));
        delay(20);
        set_color(RGB(0,0,0));
    }
}

message_t *message_tx() {
    return &(g->msg);
} 

void message_tx_success() {
    g->message_sent = 1;
}

int main() {
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