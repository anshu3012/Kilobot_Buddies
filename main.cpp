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

#define USING_SIMULATION

#include <functional>
#include <stdio.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


#include <pthread.h> 

#include <emscripten.h>
#include <SDL.h>
#include <emscripten/html5.h>

#define GL_GLEXT_PROTOTYPES 1
#include <SDL_opengles2.h>


#include "robot_code_map.h"


// things that need to be defined in global space
#define ARROW_VERTICES 6  // per arrow

#define NUM_CIRCLE_VERTICES_PER_ROBOT 25        
#define NUM_DISC_VERTICES_PER_ROBOT 12    
#define NUM_COMS_RANGE_VERTICES_PER_ROBOT 25
#define NUM_STATUS_VERTICES_PER_ROBOT 25
#define NUM_ARROW_VERTICES_PER_ROBOT (ARROW_VERTICES * 2)

#define NUM_VERTICES_PER_ROBOT 150      // needs to be > sum of the "per robot" globals

#define VERTEX_OFFSET_FOR_COMMS_GRAPH (NUM_ROBOTS * NUM_VERTICES_PER_ROBOT + 10)

#define VERTEX_OFFSET_FOR_LIGHTPROJECTOR (VERTEX_OFFSET_FOR_COMMS_GRAPH + NUM_ROBOTS*NUM_ROBOTS + 10)

#define LIGHTPROJECTOR_IMAGE_MAX_WIDTH 100
#define LIGHTPROJECTOR_IMAGE_MAX_HEIGHT 100

#define NUM_VERTICES_FOR_LIGHTPROJECTOR (LIGHTPROJECTOR_IMAGE_MAX_WIDTH * LIGHTPROJECTOR_IMAGE_MAX_HEIGHT * 3 * 2)  // height_max, width_max, vertices in a triangle, triangles required to make a square

#define TOTAL_VERTEX_BUFFER_REQUIRED (VERTEX_OFFSET_FOR_LIGHTPROJECTOR + NUM_VERTICES_FOR_LIGHTPROJECTOR)

#define MEM_SLOTS_PER_VERTEX 5

// globals for user interaction
#define CANVAS_HEIGHT 500
#define CANVAS_WIDTH 500

#define DISTANCE_RATIO_MULT 100.0   // distance in mm for each unit of draw distance (screen is 2 units by 2 units)
                                   

#include "kilolib_simulation.h"
#include "sim_data_structures.h"
#include "sim_helper_functions.h"

#include "robot_code_map.cpp"

#include "kilolib_simulation.c"

#include "sim_helper_functions.cpp"
#include "sim_data_structures.cpp"
#include "sim_drawing_functions.cpp"






// ---------------------  Shaders C Code  --------------------------------------

// the vertex shader sets the postion based on the buffer information
// it also extracts the color data from the buffer and makes it available to 
// the fragment shader
const GLchar* vertexSource =
    "attribute vec4 position;                     \n" 
    "attribute vec4 color;                        \n" 
    "varying vec3 colorV;                         \n"
 
    "void main()                                  \n"
    "{                                            \n"
    "  gl_Position = vec4(position.xyz, 1.0);     \n"
    "  // color =  vec3(0.5, 0.5, 1.0);           \n"   
    "  colorV =  color.xyz;                       \n"   
    "}                                            \n"; 

// the fragement shader just cares about color
const GLchar* fragmentSource =
    "precision mediump float;                     \n"
    "varying vec3 colorV;                         \n"
    "void main()                                  \n"
    "{                                            \n"
    "  gl_FragColor[0] = colorV[0];               \n"
    "  gl_FragColor[1] = colorV[1];               \n"
    "  gl_FragColor[2] = colorV[2];               \n"
    "}                                            \n";




// params
int num_robots = NUM_ROBOTS;

float x_pose_min = -1.0f;
float x_pose_max = 1.0f;
float y_pose_min = -1.0f;
float y_pose_max = 1.0f;

float robot_radius = 0.1f;

float com_radius = 0.8f;


float message_time = 0.001f;              // time it takes to send a message packet
float ave_inter_message_time = 0.5f;     // average time a robot waits after sending
                                         // before sending again
float max_inter_message_deviation = 0.1; // this is how much a robot might deviate
                                         // ave_inter_message_time

//float display_speed_mult = 10.0f;        // display things at this X speed


// ----------- stuff from hello worlds
float xDirection = 0.0f; 
float yDirection = 0.0f; 
int ctr = 0;


// --------- functions called by the javascript half of the code ----------
// note that in javascript these functions need to have a "_" prefix



/*  depriciated in favour of callback in this code directly
extern "C" void EMSCRIPTEN_KEEPALIVE on_click_or_touch() 
{ 
 if(ctr == 0)
  {
    yDirection = 1.0f;
    xDirection = 0.0f;
  }
  else if(ctr == 1)
  {
    yDirection = 0.0f;
    xDirection = 1.0f;
  }
  else if(ctr == 2)
  {
    yDirection = -1.0f;
    xDirection = 0.0f;
  }
  else if(ctr == 3)
  {
    yDirection = 0.0f;
    xDirection = -1.0f;
  }
  else
  {
    yDirection = 0.0f;
    xDirection = 0.0f;
    ctr = -1;
  }
  ctr++;
}
*/

// simple function to help debug
void drive_robot()
{
  if(ctr == 0)
  {
    yDirection = 1.0f;
    xDirection = 0.0f;
  }
  else if(ctr == 1)
  {
    yDirection = 0.0f;
    xDirection = 1.0f;
  }
  else if(ctr == 2)
  {
    yDirection = -1.0f;
    xDirection = 0.0f;
  }
  else if(ctr == 3)
  {
    yDirection = 0.0f;
    xDirection = -1.0f;
  }
  else
  {
    yDirection = 0.0f;
    xDirection = 0.0f;
    ctr = -1;
  }
  ctr++;
}


// --------- callback functions for user interactions ----------

// see if user is selecting a new robot
void user_robot_select(SWARM* swarm, float mouse_relative_x, float mouse_relative_y, float dist_thresh)
{
  int ind = get_index_of_closest_robot(swarm, mouse_relative_x, mouse_relative_y);
  float dist_to_ind_robot = dist_robot_to_point(swarm->robots[ind], mouse_relative_x, mouse_relative_y);
  if(dist_to_ind_robot < 0.2f)
  {
    selected_robot = ind;
  }
  else
  {
    selected_robot = -1;
  }

}

// moves user chosen robot around
void user_drag_robot(SWARM* swarm, float mouse_relative_x, float mouse_relative_y, float dist_thresh)
{
  if(selected_robot > -1 && dist_robot_to_point(swarm->robots[selected_robot], mouse_relative_x, mouse_relative_y) < dist_thresh)
  {
    // do nothing
  }
  else
  {
    // see if user is selecting a robot
    user_robot_select(swarm, mouse_relative_x, mouse_relative_y, dist_thresh);
  }

  if(selected_robot > -1)
  {
    swarm->robots[selected_robot]->position[0] = mouse_relative_x;
    swarm->robots[selected_robot]->position[1] = mouse_relative_y;
  }
}



// spins user chosen robot around to different orientations
void user_spin_robot(SWARM* swarm, float mouse_relative_x, float mouse_relative_y, float dist_thresh)
{
  if(selected_robot > -1 && dist_robot_to_point(swarm->robots[selected_robot], mouse_relative_x, mouse_relative_y) < dist_thresh)
  {
    // do nothing
  }
  else
  {
    // see if user is selecting a robot
    user_robot_select(swarm, mouse_relative_x, mouse_relative_y, dist_thresh);
  }

  if(selected_robot > -1)
  {

    // make robot face toward mouse
    float dx = mouse_relative_x - swarm->robots[selected_robot]->position[0];
    float dy = mouse_relative_y - swarm->robots[selected_robot]->position[1];
    
    swarm->robots[selected_robot]->orientation[0] = atan2f(dy, dx);
  }
}



// the mouse callback 
EM_BOOL mouse_callback(int eventType, const EmscriptenMouseEvent *e, void *swarm_data)
{


 
  if (e->screenX != 0 && e->screenY != 0 &&              // relative to screen
      0 < e->clientX && e->clientX <= CANVAS_WIDTH &&    // absolute to canvas X
      0 < e->clientY && e->clientY <= CANVAS_HEIGHT &&   // absolute to canvas Y
      e->targetX != 0 && e->targetY != 0)                // relative to canvas (I think)
  {

    float mouse_relative_x = 2.0f*((e->clientX)/(float)(CANVAS_WIDTH)) - 1.0f ;   // in -1 to 1
    float mouse_relative_y = 2.0f*(((float)(CANVAS_HEIGHT) - (e->clientY))/(float)CANVAS_HEIGHT) - 1.0f ;  // in -1 to 1


    // note that e->buttons is a unsigned short char that uses a bitmask to return data about 
    // multiple buttons so we need to extract which buttons have been pressed
    unsigned short buttons = e->buttons;
    int left_pressed   = 0;
    int center_pressed = 0;
    int right_pressed  = 0;
    if(buttons >= 32768){buttons = buttons - 32768;}  // currently unused
    if(buttons >= 16384){buttons = buttons - 16384;}  // currently unused
    if(buttons >= 8192 ){buttons = buttons - 8192 ;}  // currently unused
    if(buttons >= 4096 ){buttons = buttons - 4096 ;}  // currently unused
    if(buttons >= 2048 ){buttons = buttons - 2048 ;}  // currently unused
    if(buttons >= 1024 ){buttons = buttons - 1024 ;}  // currently unused
    if(buttons >= 512  ){buttons = buttons - 512  ;}  // currently unused
    if(buttons >= 256  ){buttons = buttons - 258  ;}  // currently unused
    if(buttons >= 128  ){buttons = buttons - 128  ;}  // currently unused
    if(buttons >= 64   ){buttons = buttons - 64   ;}  // currently unused
    if(buttons >= 32   ){buttons = buttons - 32   ;}  // currently unused
    if(buttons >= 16   ){buttons = buttons - 16   ;}  // currently unused
    if(buttons >= 8    ){buttons = buttons - 8    ;}  // currently unused
    if(buttons >= 4    )
    {
      // middle button / mouse wheel pressed
      center_pressed = 1;
      buttons = buttons - 4    ;
    }
    if(buttons >= 2    )
    {
      // secondary button pressed
      right_pressed = 1;
      buttons = buttons - 2    ;
    }
    if(buttons >= 1    )
    {
      // primary button pressed
      left_pressed = 1;
    }

    if (eventType == EMSCRIPTEN_EVENT_CLICK)
    {
      // single click

      // see if user is selecting a robot
      if(left_pressed == 1 || right_pressed == 1)
      {
        // either left or right
        user_robot_select((SWARM*)swarm_data, mouse_relative_x, mouse_relative_y, 0.2f);
      }
    }
    else if (eventType == EMSCRIPTEN_EVENT_MOUSEDOWN && e->buttons != 0)
    {
      // mouse down

      if(right_pressed == 1)
      {
        // right button down
        user_spin_robot((SWARM*)swarm_data, mouse_relative_x, mouse_relative_y, 1.0f);
      }
      else if(left_pressed == 1)
      {
        // left button down
        user_drag_robot((SWARM*)swarm_data, mouse_relative_x, mouse_relative_y, 0.2f);
      }
    }
    else if (eventType == EMSCRIPTEN_EVENT_MOUSEMOVE && (e->movementX != 0 || e->movementY != 0) && e->buttons != 0)
    {
      // moving with mouse down

      if(right_pressed == 1)
      {
        // right button down
        user_spin_robot((SWARM*)swarm_data, mouse_relative_x, mouse_relative_y, 1.0f);
      }
      else if(left_pressed == 1)
      {
        // left button down
        user_drag_robot((SWARM*)swarm_data, mouse_relative_x, mouse_relative_y, 0.2f);
      }
    }
    else if (eventType == EMSCRIPTEN_EVENT_MOUSEMOVE && (e->movementX != 0 || e->movementY != 0)  && e->buttons == 0)
    {
      // moving with mouse up
      selected_robot = -1;

    }
    else if (eventType == EMSCRIPTEN_EVENT_MOUSEUP && e->buttons == 0)
    {
      // mouse up
      selected_robot = -1;

    }
    else if (eventType == EMSCRIPTEN_EVENT_DBLCLICK)
    {
      // double click

      if(curr_light_pattern == 2)
        curr_light_pattern = 0;
      else
        curr_light_pattern++;

    }

  }

  return 0;
}



// --------------------- Individual Kilobot Robot Threads Loop --------------------------------


/*
// A normal C function that is executed as a thread  
// when its name is specified in pthread_create() 
//void* singleKilobotThread(void* bot) 
void* singleKilobotThread(void* params) 
{ 
  int id_local_copy = ((PARAMS*)params)->id;

  // lock mutext
  pthread_mutex_lock(&timing_mutext); 

  // temporarily story this robot's ID so that we can pass it to within 
  // kilo_start() via kilobot's main() function
  robot_local_copy_global_pass = ((PARAMS*)params)->swarm->robots[id_local_copy]; // used to help send bot ID to main()
  ROBOT* robot_local_copy = robot_local_copy_global_pass;


  robot_local_copy->code_space = codeSpaceMap[robot_local_copy->id];

  kilo_globals = NULL; // set to null so that old values do not get used (this is reset by main if it is used)
  kilo_uid = id_local_copy;  

  #ifdef codeSpaceA_exists
    if(robot_local_copy->code_space == A)
    {
      codeSpaceA::main();   // Note: by convention this sets init and setup function pointers 
                            // and then starts an infinite loop via the kilobot function kilo_start
                            // note that the mutext will be unlcocked and locked as necessary from
                            // within kilo_start()
    }
  #endif

  #ifdef codeSpaceB_exists
    if(robot_local_copy->code_space ==  B)
    {
      codeSpaceB::main();   // ditto
    }
  #endif

  #ifdef codeSpaceC_exists
    if(robot_local_copy->code_space ==  C)
    {
      codeSpaceC::main();   // ditto
    }
  #endif

  #ifdef codeSpaceD_exists
    if(robot_local_copy->code_space ==  D)
    {
      codeSpaceD::main();   // ditto
    }
  #endif

  #ifdef codeSpaceE_exists
    if(robot_local_copy->code_space ==  E)
    {
      codeSpaceE::main();   // ditto
    }
  #endif


  done_computing_with(robot_local_copy);

  // unlock the mutext
  pthread_mutex_unlock(&timing_mutext);

  pthread_exit(NULL);

} 
*/




// A normal C function that is executed as a thread  
// when its name is specified in pthread_create() 
//void* singleKilobotThread(void* bot) 
void singleKilobotThreadReplacement(void* params) 
{ 
  int id_local_copy = ((PARAMS*)params)->id;

  // lock mutext
  //pthread_mutex_lock(&timing_mutext); 

  // temporarily story this robot's ID so that we can pass it to within 
  // kilo_start() via kilobot's main() function
  robot_local_copy_global_pass = ((PARAMS*)params)->swarm->robots[id_local_copy]; // used to help send bot ID to main()
  ROBOT* robot_local_copy = robot_local_copy_global_pass;


  robot_local_copy->code_space = codeSpaceMap[robot_local_copy->id];

  kilo_globals = NULL; // set to null so that old values do not get used (this is reset by main if it is used)
  kilo_uid = id_local_copy;  

  #ifdef codeSpaceA_exists
    if(robot_local_copy->code_space == A)
    {
      codeSpaceA::main();   // Note: by convention this sets init and setup function pointers 
                            // and then starts an infinite loop via the kilobot function kilo_start
                            // note that the mutext will be unlcocked and locked as necessary from
                            // within kilo_start()
    }
  #endif

  #ifdef codeSpaceB_exists
    if(robot_local_copy->code_space ==  B)
    {
      codeSpaceB::main();   // ditto
    }
  #endif

  #ifdef codeSpaceC_exists
    if(robot_local_copy->code_space ==  C)
    {
      codeSpaceC::main();   // ditto
    }
  #endif

  #ifdef codeSpaceD_exists
    if(robot_local_copy->code_space ==  D)
    {
      codeSpaceD::main();   // ditto
    }
  #endif

  #ifdef codeSpaceE_exists
    if(robot_local_copy->code_space ==  E)
    {
      codeSpaceE::main();   // ditto
    }
  #endif


  done_computing_with(robot_local_copy);

  // unlock the mutext
 // pthread_mutex_unlock(&timing_mutext);

  //pthread_exit(NULL);

} 


// --------------------- Start of Main Loop --------------------------------
std::function<void()> loop;
void main_loop() 
{
  loop(); 
}

int main()
{ 
    // load in user code space preferences
    doCodeSpaceMap();


    // store current time
    double time_start = get_curr_time();


    // create the swarm  ----------------------
    // (needs to be done prior to registering mouse callback)
    GLuint num_vertices_used = 0;                              // added to help keep track of how many verticies are used

    SWARM* swarm = make_rand_swarm(num_robots, x_pose_min, x_pose_max, y_pose_min, y_pose_max, robot_radius, com_radius);

    swarm_ = swarm;

    // attempt to move overlapping robots
    make_swarm_non_overlapping(swarm, 1000);
    swarm_particle_distribute(swarm, 1000);

    // windowing stuff ---------------------
    SDL_Window *window;
    SDL_CreateWindowAndRenderer(CANVAS_WIDTH, CANVAS_HEIGHT, 0, &window, nullptr);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);


    // mouse callback stuff -----------------------
    EMSCRIPTEN_RESULT ret = emscripten_set_click_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, swarm, 1, mouse_callback);
    ret = emscripten_set_mousedown_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, swarm, 1, mouse_callback);
    ret = emscripten_set_mouseup_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, swarm, 1, mouse_callback);
    ret = emscripten_set_dblclick_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, swarm, 1, mouse_callback);
    ret = emscripten_set_mousemove_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, swarm, 1, mouse_callback);


    
    // create a buffer object to store the robot vertex data
    GLuint robot_vertex_buffer_object[1];                         // buffer object (I think an array of handles or a "names" )
    glGenBuffers(1, robot_vertex_buffer_object);                  // in theory this returns an array of 1 thing, so
                                                                  // robot_vertex_buffer_object[0] holds a buffer object "name"

    glBindBuffer(GL_ARRAY_BUFFER, robot_vertex_buffer_object[0]); // this binds the buffer object with the specified name a 
                                                                  // buffer of type GL_ARRAY_BUFFER (which holds "vertex attributes")
 
    // memory we will actually use to store the data for the buffer
    GLfloat drawing_vertices[TOTAL_VERTEX_BUFFER_REQUIRED*MEM_SLOTS_PER_VERTEX];       // times 2 because each vertex needs two memory slots (x, y)

    for(int b = 0; b < TOTAL_VERTEX_BUFFER_REQUIRED*MEM_SLOTS_PER_VERTEX; b++)
    {
      drawing_vertices[b] = 0.5f;
    }

    glBufferData(GL_ARRAY_BUFFER, sizeof(drawing_vertices), drawing_vertices, GL_DYNAMIC_DRAW);  // links the beffer object to the actual memory
 

    // [TUTORIAL] Create and compile the vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);                     // create a shader
    glShaderSource(vertexShader, 1, &vertexSource, nullptr);                    // shaders are loaded from source code, find it here
    glCompileShader(vertexShader);                                              // "compile" the shader from the source code
    // from docs: "A shader of type GL_VERTEX_SHADER is a shader that is intended to run on the programmable vertex processor."


    // [TUTORIAL] Create and compile the fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);                 // [TUTORIAL] 
    glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);                // [TUTORIAL] 
    glCompileShader(fragmentShader);                                            // [TUTORIAL] 
    // from docs: "A shader of type GL_FRAGMENT_SHADER is a shader that is intended to run on the programmable fragment processor."


    // [TUTORIAL] Link the vertex and fragment shader into a shader program
    GLuint shaderProgram = glCreateProgram();                               // creates a "program" to which shader objects can be attached
    glAttachShader(shaderProgram, vertexShader);                            // attach the vertex shader to the program
    glAttachShader(shaderProgram, fragmentShader);                          // attach the fragement shader to the program
    glLinkProgram(shaderProgram);                                           // link the shader (see below)
    // "from docs: "links the program object specified by program. If any shader objects of type GL_VERTEX_SHADER are attached to program, they will be used to create an executable that will run on the programmable vertex processor. "

    glUseProgram(shaderProgram);                                            // start using the program, see below 
    // from docs: "installs the program object specified by program as part of current rendering state. One or more executables are created in a program object by successfully attaching shader objects to it with glAttachShader, successfully compiling the shader objects with glCompileShader, and successfully linking the program object with glLinkProgram. "


    // [TUTORIAL] Specify the layout of the vertex data
    GLint posAttrib = glGetAttribLocation(shaderProgram, "position");        // create a way to interact with position data in the shader program
    GLint colorAttrib = glGetAttribLocation(shaderProgram, "color");         // create a way to interact with color data in the shader program

    glEnableVertexAttribArray(posAttrib);                                    // enables the position attribute
    glEnableVertexAttribArray(colorAttrib);                                  // enables the color attribute


    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, MEM_SLOTS_PER_VERTEX*(sizeof(GLfloat)), (void*)0);  
//    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, MEM_SLOTS_PER_VERTEX * sizeof(float), (void*)0);           
                                                                             // sets up how the things in the pose buffer will be stored, 
                                                                             // i.e., each vertex contains two floats (not normalized)
                                                                             // stride = 0 "the generic vertex attributes are understood to 
                                                                             // be tightly packed in the array.")
                                                                             // the offset of the first thing is 0.

    glVertexAttribPointer(colorAttrib, 3, GL_FLOAT, GL_FALSE, MEM_SLOTS_PER_VERTEX*(sizeof(GLfloat)), (void*)(2*sizeof(GLfloat)));
//    glVertexAttribPointer(colorAttrib, 3, GL_FLOAT, GL_FALSE, MEM_SLOTS_PER_VERTEX * sizeof(float), (void*)2);
                                                                            // sets up how the things in the color buffer will be stored, 
                                                                             // i.e., each vertex contains two floats (not normalized)
                                                                             // stride = 0 "the generic vertex attributes are understood to 
                                                                             // be tightly packed in the array.")
                                                                             // the offset of the first thing is 2. (right after the position stuff)




    // setup message stuff
    setup_message_times(swarm, message_time, ave_inter_message_time, max_inter_message_deviation);


    // create an event log
    event_log = make_new_event_log(1000, time_start);




    // p-thread stuff
    //pthread_mutex_init(&timing_mutext, NULL);



    PARAMS params[num_robots];
    pthread_t thread_id[num_robots]; 
    //pthread_barrier_init(  &barrier, NULL, 1);
    for(int i = 0; i < NUM_ROBOTS; i++)
    {
      params[i].swarm = swarm;
      params[i].id = i;

      //pthread_create(&thread_id[i], NULL, singleKilobotThread, &params[i]); 
      //pthread_detach(thread_id[i]);


      singleKilobotThreadReplacement(&params[i]);


    }

    // load light projector data
    load_all_light_data();
    curr_light_pattern = 0;

    // create a debug log 
    debug_log = make_new_debug_log();

    char cst[DEBUG_LINES*(DEBUG_CHARS_PER_LINE+EXTRA_CHARS_PER_LINE) + 100];
    double time_then = get_curr_time() - time_start;  // this is time since we started the simulation
                                                // doing it this way prevents numerical error

    int displayed_error_message = 0;
    loop = [&]
    {
        //emscripten_run_script("print_debugging_log('here 0<br>')");

        iii++;
        // log_message_from_sim("------- start of main loop %d -----", iii);



    //    for(int i = 0; i < NUM_ROBOTS; i++)
    //    {
    //      run_full_loop(swarm->robots[i]);
    //    }





        // lock the mutext
        //pthread_mutex_lock(&timing_mutext); 

        display_com_rads = EM_ASM_INT(return get_display_com_rads());
        display_graph = EM_ASM_INT(return get_display_graph());  
        display_debugging_log = EM_ASM_INT(return get_display_debugging_log());  

        // get current time
        double time_now = get_curr_time() - time_start;  // this is time since we started the simulation
                                                         // doing it this way prevents numerical error





        for(int i = 0; i < num_robots; i++)
        {
          ROBOT* robot_local_copy = swarm->robots[i];
  
          if(robot_local_copy->need_to_run_loop)
          {

            // swap this robot into global space
            now_computing_with(robot_local_copy);

            robot_local_copy->need_to_run_loop = 0;

            // start timer for this run
            robot_local_copy->time_since_last_time_update = 0.0;

            // do the computation loop for this robot
            (*(robot_local_copy->kilo_loop))();  // run one loop

            delay(50);  // delay for 50 ms since this is where we are hiding some threading things in this function


            // store when the next loop should start
            enable_loop();

            // save elapsed time
            robot_local_copy->time_since_start += robot_local_copy->time_since_last_time_update;
            robot_local_copy->time_since_last_time_update = 0.0;

            // save any values that have changed 
            done_computing_with(robot_local_copy);
          }
        }



        // process robot events that have occoured since the last time
        process_events_from_log(event_log, swarm, time_now);



        // drive robots
        //drive_swarm_for_duration(swarm, time_now - time_then);
        drive_buddy_swarm_for_duration(time_now - time_then, swarm);
        //make_buddies(swarm);
        //drive_swarm_for_duration(swarm, time_now - time_then);
        //drive_buddies_near(make_buddies(swarm),swarm,time_now - time_then);
        //make_buddies(swarm, time_now - time_then);
        time_then = time_now;

        // collision check vs workspace bouandary
        collision_check_bouandary(swarm);
        collision_check_robots(swarm);

        // calculate current communication graph
        calculate_connectivity_graph(swarm);

        //emscripten_run_script("print_debugging_log('here 5<br>')");
        // do the communication (handle events that have happened since last time)
        //do_communication_debug(swarm, time_now);
        do_communication(swarm, time_now);


        // set background color
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT); 

        // draw everything
        glBufferData(GL_ARRAY_BUFFER, sizeof(drawing_vertices), drawing_vertices, GL_DYNAMIC_DRAW); 
        draw_lightprojector(light_patterns[curr_light_pattern], 0, drawing_vertices);
        draw_swarm(swarm, drawing_vertices);

        // OPEN_GL swap window
        SDL_GL_SwapWindow(window);

  
        // display user debugging messages
        if(display_debugging_log == 1 && displayed_error_message == 0)
        {
          format_debug_log(debug_log);  // get string of debug information for sending to javascript
          sprintf(cst, "print_debugging_log('loop: %d<br> <br>%s')", iii, (const char *)(debug_log->log_formatted));
          emscripten_run_script((const char*)cst);

          if(debug_log->displaying_error_message == 1)
          {
            displayed_error_message = 1;
          }
        }

        // unlock the mutext
        //pthread_mutex_unlock(&timing_mutext);









    };

    emscripten_set_main_loop(main_loop, 0, true);                               // [TUTORIAL] 


    //destroy_robot(robot);
    //destroy_robot(robotB);
    //destroy_robot(robotC);


    //pthread_join(thread_id_A, NULL); 
    //pthread_join(thread_id_B, NULL); 
    //pthread_join(thread_id_C, NULL); 

    destroy_event_log(event_log);
    destroy_debug_log(debug_log);
    destroy_swarm(swarm);
    destroy_all_light_data();  


    return EXIT_SUCCESS;                                                        // [TUTORIAL] 
}


