// each namespace contain the file(s) that would normally be compile to a single hex file for upload to a kilobot
// the mapping of which robots goes to each namespace appears below. Note, currently we only support 5 (A-E)

// NOTE: due to a bug, due to a bug, make sure each has at least some valid include file in it right now

//nice!

namespace codeSpaceA
{
  //#include "rainbow.c"
  //#include "lightSensor.c"
  //#include "dist_to_neighbor.c"
  //#include "num_neighbors.c"
  //#include "simple_movement.c"

  #include "buddy_tx.c"
  int x;

}
namespace codeSpaceB
{
  //#include "rainbow.c"
  //#include "num_neighbors.c"
  //#include "simple_movement.c"

  #include "buddy_rx.c"
}
namespace codeSpaceC
{
  #include "num_neighbors.c"
}
namespace codeSpaceD
{
  #include "dist_to_neighbor.c"
}
namespace codeSpaceE
{
  //#include "rainbow.c"
  #include "simple_movement.c"
}

// define which codespaces we are actually using
// (comment out the ones not being used)
// NOTE: due to a bug, leave all commented in for now
#define codeSpaceA_exists
#define codeSpaceB_exists
#define codeSpaceC_exists
#define codeSpaceD_exists
#define codeSpaceE_exists

void doCodeSpaceMap()
{
  for(int i = 0; i < NUM_ROBOTS; i++)
  {
    codeSpaceMap[i] = E;

  }

  /*
  codeSpaceMap[0] = B;
  codeSpaceMap[1] = B;
  codeSpaceMap[2] = B;
  codeSpaceMap[3] = B;
  codeSpaceMap[4] = B;
  */

}
