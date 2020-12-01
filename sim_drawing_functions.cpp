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

#ifndef __SIM_DRAWING_FUNCTIONS_CPP__
#define __SIM_DRAWING_FUNCTIONS_CPP__

/*------------------- SIMPLE SHAPE DRAW FUNCTIONS -----------------------*/



// draws a circle (outline)
void draw_circle(float* pos, float rad, float* color, float line_width, GLfloat* vertices,  GLuint buffer_mem_start, GLuint num_vertices, int robotNum)
{
  for(GLuint i = 0; i < num_vertices; i++)
  {    
    float a = 2.0f*PI*((float)i)/((float)num_vertices); 
    int ind = buffer_mem_start + i*MEM_SLOTS_PER_VERTEX;

    vertices[ind] = rad*cos(a) + pos[0];    // x
    vertices[ind+1] = rad*sin(a) + pos[1];  // y

    vertices[ind+2] = color[0];             // r
    vertices[ind+3] = color[1];             // g
    vertices[ind+4] = color[2];             // b

  } 


  glLineWidth(line_width);

  GLuint buffer_vertex_start = buffer_mem_start/MEM_SLOTS_PER_VERTEX;
  glDrawArrays(GL_LINE_LOOP, buffer_vertex_start, num_vertices);

  glLineWidth(2.0f);
}



// draws a disc (filled circle)
void draw_disc(float* pos, float rad, float* color, GLfloat* vertices, GLuint buffer_mem_start, GLuint num_vertices, GLuint robotNum)
{
  // first vertex is at the center of the robot
  vertices[buffer_mem_start]     = pos[0]; // x
  vertices[buffer_mem_start + 1] = pos[1]; // y

  vertices[buffer_mem_start+2] = color[0]; // r
  vertices[buffer_mem_start+3] = color[1]; // g
  vertices[buffer_mem_start+4] = color[2]; // b

  // the next go around the circle
  for(GLuint i = 0; i < num_vertices-1; i++)
  {    
    float a = 2.0f*PI*((float)i)/((float)(num_vertices-2)); 
    int ind = buffer_mem_start + MEM_SLOTS_PER_VERTEX + i*MEM_SLOTS_PER_VERTEX;
    vertices[ind]   = rad*cos(a) + pos[0];  // x
    vertices[ind+1] = rad*sin(a) + pos[1];  // y

    vertices[ind+2] = color[0];             // r
    vertices[ind+3] = color[1];             // g
    vertices[ind+4] = color[2];             // b
  } 

  GLuint buffer_vertex_start = buffer_mem_start/MEM_SLOTS_PER_VERTEX;
  glDrawArrays(GL_TRIANGLE_FAN, buffer_vertex_start, num_vertices);
}



// draws an arrow
void draw_arrow(float* pos, float* orientation, float length, float pointer_start_rad, float pointer_d_orientation, float* color, GLfloat* vertices, GLuint buffer_mem_start)
{

  for(int i = 0; i < 6; i++)
  {
    int ind = buffer_mem_start + i*MEM_SLOTS_PER_VERTEX;

    if(i == 0)
    {
      // first vertex is at the center of the robot
      vertices[ind]     = pos[0]; // x
      vertices[ind + 1] = pos[1]; // y
    }
    else if(i == 1 || i == 3 || i == 5)
    {
      // second, fourth, and sixth vertices are at the front of the robot
      vertices[ind]     = pos[0] + length*cos(orientation[0]); // x
      vertices[ind + 1] = pos[1] + length*sin(orientation[0]); // y
    }
    else if(i == 2)
    {
      // third vertex is front left
      vertices[ind]     = pos[0] + pointer_start_rad*cos(orientation[0] + pointer_d_orientation); // x
      vertices[ind + 1] = pos[1] + pointer_start_rad*sin(orientation[0] + pointer_d_orientation); // y
    }
    else if(i == 4)
    {
      // fifth vertex is front right
      vertices[ind]     = pos[0] + pointer_start_rad*cos(orientation[0] - pointer_d_orientation); // x
      vertices[ind + 1] = pos[1] + pointer_start_rad*sin(orientation[0] - pointer_d_orientation); // y
    }

    vertices[ind+2] = color[0]; // r
    vertices[ind+3] = color[1]; // g
    vertices[ind+4] = color[2]; // g
  }

  glLineWidth(2.0f);
  GLuint buffer_vertex_start = buffer_mem_start/MEM_SLOTS_PER_VERTEX;
  glDrawArrays(GL_LINES, buffer_vertex_start, ARROW_VERTICES);
  glLineWidth(1.0f);
}



//// outputs the text
//void draw_string(float* pos, float pad_left, float pad_bottom, const char* string, float buffer_depth) 
//{
//  glColor3f(0.5f,0.5f,0.5f);
//  glRasterPos3f(pos[0]+pad_left, pos[1]+pad_bottom, buffer_depth);
//  for(int i = 0; string[i] != '\0'; i++)
//    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
//}

void draw_robot(ROBOT* bot, GLfloat* vertices)
{

  
  float* status_color = WHITE;
  if(bot->message_collision == 1)
  {
    status_color = RED;
  }
  else if(bot->sending >= 1)
  {
    status_color = GREEN;
  }

/*
  float sensor_input_color[] = {bot->visual_light_input, bot->visual_light_input, bot->visual_light_input};

  // draw a bold white outline if this robot is recieving light data
  // but do not draw if user has requested that the light pattern display be off 
  if(toggle_input_output != 2)
  {
    draw_disk(bot->position, bot->radius, sensor_input_color, buffer_depth - .002);
  }
*/

  // figure out where we put this in the draw buffer
  GLuint buffer_mem_start = MEM_SLOTS_PER_VERTEX*(NUM_VERTICES_PER_ROBOT*bot->id + NUM_CIRCLE_VERTICES_PER_ROBOT);  
  draw_disc(bot->position, 0.8f*bot->radius, bot->led_color, vertices, buffer_mem_start, NUM_DISC_VERTICES_PER_ROBOT, bot->id);   

  // float cc = (bot->n_net->test_signals[0][0]+1.0f)/2.0f;
  // float clr[] = {cc,cc,cc};
  // draw_disk(bot->position, 0.8f*bot->radius, clr, buffer_depth - .001);


  if(display_messages == 1)
  {

    buffer_mem_start = MEM_SLOTS_PER_VERTEX*(NUM_VERTICES_PER_ROBOT*bot->id + NUM_CIRCLE_VERTICES_PER_ROBOT + NUM_DISC_VERTICES_PER_ROBOT + NUM_COMS_RANGE_VERTICES_PER_ROBOT);  

    draw_disc(bot->position, bot->radius/4.0f, status_color, vertices, buffer_mem_start, NUM_STATUS_VERTICES_PER_ROBOT, bot->id);   
  }

  if(display_com_rads == 1)
  { 

    // figure out where we put this in the draw buffer
    buffer_mem_start = MEM_SLOTS_PER_VERTEX*NUM_VERTICES_PER_ROBOT*bot->id;
    draw_circle(bot->position, bot->radius, bot->color, 3.0f, vertices, buffer_mem_start, NUM_CIRCLE_VERTICES_PER_ROBOT, bot->id); 
  }
  else
  {

    // figure out where we put this in the draw buffer
    buffer_mem_start = MEM_SLOTS_PER_VERTEX*NUM_VERTICES_PER_ROBOT*bot->id;
    draw_circle(bot->position, bot->radius, WHITE, 1.0f, vertices, buffer_mem_start, NUM_CIRCLE_VERTICES_PER_ROBOT, bot->id); 
  }


  // draw heading arrow
  buffer_mem_start = MEM_SLOTS_PER_VERTEX*(NUM_VERTICES_PER_ROBOT*bot->id + NUM_CIRCLE_VERTICES_PER_ROBOT + NUM_DISC_VERTICES_PER_ROBOT + NUM_COMS_RANGE_VERTICES_PER_ROBOT + NUM_STATUS_VERTICES_PER_ROBOT);  
  draw_arrow(bot->position, bot->orientation, 0.8f*bot->radius, 0.4f*bot->radius, PI/6.0f, SOMEGRAY, vertices, buffer_mem_start);  


  // draw angular velocity arrow
  if(bot->d_orientation[0] != 0.0f)
  {
    buffer_mem_start = MEM_SLOTS_PER_VERTEX*(NUM_VERTICES_PER_ROBOT*bot->id + NUM_CIRCLE_VERTICES_PER_ROBOT + NUM_DISC_VERTICES_PER_ROBOT + NUM_COMS_RANGE_VERTICES_PER_ROBOT + NUM_STATUS_VERTICES_PER_ROBOT + ARROW_VERTICES);  
 
    float dir_arrow_pose[] = {bot->position[0] + bot->radius*cos(bot->orientation[0]),
                            bot->position[1] + bot->radius*sin(bot->orientation[0])};
    float d_dir_arrow_pose[] = {bot->orientation[0] + bot->d_orientation[0]};  

    draw_arrow(dir_arrow_pose, d_dir_arrow_pose, 0.8f*bot->radius, 0.35f*bot->radius, PI/6.0f, MEDIUMGREEN, vertices, buffer_mem_start);  
  }

//  if(display_ids == 1)
//  {
//    char c[10];
//    sprintf(c, "%d", bot->id);
//    draw_string(bot->position, 0.0f, 0.0f, c, data_depth_buffer);
//  }

//  // if this is the selected robot then outline it
//  if(selected_robot == bot->id)
//  {
//    // figure out where we put this in the draw buffer
//    buffer_mem_start =
//    draw_circle(bot->position, bot->radius*1.1, GREEN, 5.0f, buffer_depth + .0005);
//  }


}


// draws a single robot's comms range
void draw_com_range(ROBOT* bot, GLfloat* vertices)
{
  // figure out where we put this in the draw buffer
  GLuint buffer_mem_start = MEM_SLOTS_PER_VERTEX*(NUM_VERTICES_PER_ROBOT*bot->id + NUM_CIRCLE_VERTICES_PER_ROBOT + NUM_DISC_VERTICES_PER_ROBOT);

  draw_circle(bot->position, bot->com_radius, bot->color, 3.0f, vertices, buffer_mem_start, NUM_COMS_RANGE_VERTICES_PER_ROBOT, bot->id); 
}


// draws the swarms comm range
void draw_comms_graph(SWARM* swarm, GLfloat* vertices)
{
  GLuint buffer_mem_start = VERTEX_OFFSET_FOR_COMMS_GRAPH*MEM_SLOTS_PER_VERTEX;

  int i,j;
  int num_vertices = 0;
  int ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;
  for(i = 0; i < swarm->num_robots; i++)
  {
    for(j = 0; j < swarm->robots[i]->num_neighbors; j++)
    {
      vertices[ind] = swarm->robots[i]->position[0];   // x
      vertices[ind+1] = swarm->robots[i]->position[1]; // y

      vertices[ind+2] = 1.0f; // r
      vertices[ind+3] = 1.0f; // g
      vertices[ind+4] = 1.0f; // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;

      vertices[ind] = swarm->robots[i]->neighbors[j]->position[0];
      vertices[ind+1] = swarm->robots[i]->neighbors[j]->position[1];

      vertices[ind+2] = 1.0f; // r
      vertices[ind+3] = 1.0f; // g
      vertices[ind+4] = 1.0f; // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;
    }
  }


  glLineWidth(2.0f);
 
  glDrawArrays(GL_LINES, VERTEX_OFFSET_FOR_COMMS_GRAPH, num_vertices);

  glLineWidth(1.0f);

}

void draw_swarm(SWARM* swarm, GLfloat* vertices)
{
  int i,j;

  // draw communication ranges
  if(display_com_rads == 1)
  {
    for(i = 0; i < swarm->num_robots; i++)
    {
      draw_com_range(swarm->robots[i], vertices);
    }
  }


  // draw the connectivity graph
  if(display_graph == 1)
  {
    draw_comms_graph(swarm, vertices);
  }

  // draw robots
  for(i = 0; i < swarm->num_robots; i++)
  {
     draw_robot(swarm->robots[i], vertices);
  }
}




// draws the light pattern across the area
// make_red == 1 only draws the red channel
void  draw_lightprojector(LIGHTPROJECTOR* lp, int make_red, GLfloat* vertices)
{

  GLuint buffer_mem_start = VERTEX_OFFSET_FOR_LIGHTPROJECTOR*MEM_SLOTS_PER_VERTEX;

  float d_x = 2.0f/(float)lp->pixel_cols;   // mult by 2 because the area is -1 to 1
  float d_y = 2.0f/(float)lp->pixel_rows;   // mult by 2 because the area is -1 to 1

  int i,j;
  int num_vertices = 0;
  int ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;
  float r, b, g;

  // make sure that we stay withing the memory we've allocated for drawing
  int max_width = lp->pixel_cols;
  int max_height = lp->pixel_rows;
  if(max_width > LIGHTPROJECTOR_IMAGE_MAX_WIDTH)
  {
    max_width = LIGHTPROJECTOR_IMAGE_MAX_WIDTH;
  }
  if(max_height > LIGHTPROJECTOR_IMAGE_MAX_HEIGHT)
  {
    max_height = LIGHTPROJECTOR_IMAGE_MAX_HEIGHT;
  }
 
  // now do the drawing
  for(i = 0; i < max_width; i++)
  {
    for(j = 0; j < max_height; j++)
    {    
      if(make_red == 0)
      {
        r = lp->light_data[i][j];
        g = lp->light_data[i][j];
        b = lp->light_data[i][j];
      }
      else
      {
        r = lp->light_data[i][j];
        g = 0.0f;
        b = 0.0f;
      }
      float left = d_x * (float)i -1.0f;    // -1 because the area is -1 to 1
      float right = left + d_x;
      float bottom = d_y * (float)j -1.0f;  // -1 because the area is -1 to 1
      float top = bottom + d_y;

      // note this used to be implimented using GL_QUADS whish has been depriciated
      // so we modified it to use two triangles where there used to be one quad

      // first triangle -----------------
      vertices[ind] = left;     // x
      vertices[ind+1] = bottom; // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;

      vertices[ind] = left;     // x
      vertices[ind+1] = top;    // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;

      vertices[ind] = right;    // x
      vertices[ind+1] = top;    // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;

      // second triangle -----------------

      vertices[ind] = right;    // x
      vertices[ind+1] = top;    // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;


      vertices[ind] = right;    // x
      vertices[ind+1] = bottom; // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;

      vertices[ind] = left;     // x
      vertices[ind+1] = bottom; // y
      vertices[ind+2] = r;      // r
      vertices[ind+3] = g;      // g
      vertices[ind+4] = b;      // b

      num_vertices++;
      ind = buffer_mem_start + num_vertices*MEM_SLOTS_PER_VERTEX;
    }
  }
   

  glDrawArrays(GL_TRIANGLES, VERTEX_OFFSET_FOR_LIGHTPROJECTOR, num_vertices);
} 






/*------------------- Functions for user interaction  ---------------------------*/
// returns the index of the robot that is coloset to x_pos y_pos
int get_index_of_closest_robot(SWARM* swarm, float x_pos, float y_pos)
{
  int closest_ind = -1;
  float closest_dist = 100000000.0f;

  int j;
  for(j = 0; j < swarm->num_robots; j++)
  {
    float this_dist = dist_robot_to_point(swarm->robots[j], x_pos, y_pos);
    if(this_dist < closest_dist)
    {
      closest_dist = this_dist;
      closest_ind = j;
    }
  }
  return closest_ind;
}











#endif //__SIM_DRAWING_FUNCTIONS_CPP__


