

//Contains the function used to move buddies togther 
void drive_buddies_for_duration(ROBOT* bot, SWARM* swarm,  float dt)
{
  int i;
  for (i=0; i<(swarm->num_robots/2); i++)
  {
    if ((swarm->vect[i].first==bot->id)||(swarm->vect[i].second==bot->id)) // checking at which index of the vector is our jth robot 
    {
      i=i;
      break;
    }
  }

  if (swarm->vect[i].first==bot->id)
  {
    int q;
    for (int j=0; j<swarm->num_robots;j++)
    {  
      if (swarm->robots[j]->id==swarm->vect[i].second)
      {
        q=j;
        break;
      }   
    }

    float time_resolution = 0.01;   // time resolution
    int last_loop = 0;
    float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
    float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
    // note that bot->speed is the current ratio of max alowed speed

    // if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
    // {
    //   // robot is stopped

    //   bot->d_orientation[0] = 0.0f;
    //   return;
    // }

    if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) && (swarm->robots[q]->speed[0]<= 0.0f && swarm->robots[q]->speed[1]<= 0.0f) || dt <= 0.0f)
    //if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
    {
      // robot is stopped

      bot->d_orientation[0] = 0.0f;
      swarm->robots[q]->d_orientation[0]=0.0f;
      return;
    }

    // assuming tank steering is a good enough approximation
    float x = bot->position[0];
    float y = bot->position[1];
    float theta = bot->orientation[0];
    bot->speed[0]=bot->speed[1];

    float x1 = swarm->robots[q]->position[0];
    float y1 =swarm->robots[q]->position[1];
    float theta1 = swarm->robots[q]->orientation[0];
    swarm->robots[q]->speed[0]=swarm->robots[q]->speed[1];


    // if robot is moving straight
    //if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f  && fabs(swarm->robots[q]->speed[1] - swarm->robots[q]->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
    if(fabs(swarm->robots[q]->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
    {
      float speed = ((swarm->robots[q]->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
      float dx = dt*cos(theta)*speed;
      float dy = dt*sin(theta)*speed;
      bot->position[0] += dx;
      bot->position[1] += dy;
      bot->d_orientation[0] = 0.0f;

      // float speed1 = ((swarm->robots[q]->speed[1] + swarm->robots[q]->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
      // float dx1 = dt*cos(theta)*speed1;
      // float dy1 = dt*sin(theta)*speed1;
      swarm->robots[q]->position[0] += dx;
      swarm->robots[q]->position[1] += dy;
      swarm->robots[q]->d_orientation[0] = 0.0f;
      return; 
    }
      
    // otherwise robot is turning

    float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
    float v_r = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

    // float v_l1 = swarm->robots[q]->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius; 
    // float v_r1 = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*(bot->radius+2*bot->radius);

    float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
    float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

    // float R1 = 2*bot->radius * (v_r1 -2*v_l1)/(v_l1 - v_r1);  // radius of rotation
    // float omega1 = (v_r - v_l)/bot->radius;

    float c_x = x - R*sin(theta); // center of rotation x
    float c_y = y + R*cos(theta); // center of rotation y

    // float c_x1 = x1 - R1*sin(theta1); // center of rotation x
    // float c_y1 = y1 + R1*cos(theta1);

    x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
    y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
    theta = theta + omega*dt;

    x1 = cos(omega*dt)*(x1 - c_x) - sin(omega*dt)*(y1 - c_y) + c_x;
    y1 = sin(omega*dt)*(x1 - c_x) + cos(omega*dt)*(y1 - c_y) + c_y;
    // theta1 = theta1 + omega1*dt;

    while(theta > 2.0f*PI)
    {
      theta -= 2.0f*PI;
    }
    while(theta < -2.0f*PI)
    {
      theta += 2.0f*PI;
    }

    // while(theta1 > 2.0f*PI)
    // {
    //   theta1 -= 2.0f*PI;
    // }
    // while(theta1 < -2.0f*PI)
    // {
    //   theta1 += 2.0f*PI;
    // }


    bot->position[0] = x;
    swarm->robots[q]->position[0]=x1;

    bot->position[1] = y;
    swarm->robots[q]->position[1]=y1;
  
    bot->orientation[0] = theta;
    swarm->robots[q]->orientation[0]=theta;

    bot->d_orientation[0] = omega;
    swarm->robots[q]->d_orientation[0]=omega;
  }
  
  // if (swarm->vect[i].second==bot->id)
  // {
  //   int q;
  //   for (int j=0; j<swarm->num_robots;j++)
  //   {
  //     if (swarm->robots[j]->id==swarm->vect[i].first)
  //     {
  //       q=j;
  //       break;
  //     }
  //   }

  //   float time_resolution = 0.01;   // time resolution
  //   int last_loop = 0;
  //   float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
  //   float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
  //   // note that bot->speed is the current ratio of max alowed speed

  //   //if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) && (swarm->robots[q]->speed[0]<= 0.0f && swarm->robots[q]->speed[1]<= 0.0f) || dt <= 0.0f)
  //   if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
  //   {
  //     // robot is stopped

  //     bot->d_orientation[0] = 0.0f;
  //     //swarm->robots[q]->d_orientation[0]=0.0f;
  //     return;
  //   }

  //   // assuming tank steering is a good enough approximation
  //   float x = bot->position[0];
  //   float y = bot->position[1];
  //   float theta = bot->orientation[0];

  //   // float x1 = swarm->robots[q]->position[0];
  //   // float y1 =swarm->robots[q]->position[1];
  //   // float theta1 = swarm->robots[q]->orientation[0];

    


  //   // if robot is moving straight
  //   //if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f  && fabs(swarm->robots[q]->speed[1] - swarm->robots[q]->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  //   if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  //   {
  //     float speed = ((bot->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
  //     float dx = dt*cos(theta)*speed;
  //     float dy = dt*sin(theta)*speed;
  //     bot->position[0] += dx;
  //     bot->position[1] += dy;
  //     bot->d_orientation[0] = 0.0f;

  //     // float speed1 = ((swarm->robots[q]->speed[1] + swarm->robots[q]->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;
  //     // float dx1 = dt*cos(theta)*speed1;
  //     // float dy1 = dt*sin(theta)*speed1;
  //     // swarm->robots[q]->position[0] += dx1;
  //     // swarm->robots[q]->position[1] += dy1;
  //     // swarm->robots[q]->d_orientation[0] = 0.0f;
  //     return; 
  //   }
      
  //   // otherwise robot is turning

  //   float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
  //   float v_r = bot->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

  //   // float v_l1 = swarm->robots[q]->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius; 
  //   // float v_r1 = swarm->robots[q]->speed[1]*max_speed_while_turning_in_rads_per_sec*(bot->radius+2*bot->radius);

  //   float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
  //   float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

  //   // float R1 = 2*bot->radius * (v_r1 -2*v_l1)/(v_l1 - v_r1);  // radius of rotation
  //   // float omega1 = (v_r - v_l)/bot->radius;

  //   float c_x = x - R*sin(theta); // center of rotation x
  //   float c_y = y + R*cos(theta); // center of rotation y

  //   // float c_x1 = x1 - R1*sin(theta1); // center of rotation x
  //   // float c_y1 = y1 + R1*cos(theta1);

  //   x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
  //   y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
  //   theta = theta + omega*dt;

  //   // x1 = cos(omega1*dt)*(x1 - c_x1) - sin(omega1*dt)*(y1 - c_y1) + c_x1;
  //   // y1 = sin(omega1*dt)*(x1 - c_x1) + cos(omega1*dt)*(y1 - c_y1) + c_y1;
  //   //theta1 = theta1 + omega1*dt;

  //   while(theta > 2.0f*PI)
  //   {
  //     theta -= 2.0f*PI;
  //   }
  //   while(theta < -2.0f*PI)
  //   {
  //     theta += 2.0f*PI;
  //   }

  //   // while(theta1 > 2.0f*PI)
  //   // {
  //   //   theta1 -= 2.0f*PI;
  //   // }
  //   // while(theta1 < -2.0f*PI)
  //   // {
  //   //   theta1 += 2.0f*PI;
  //   // }


  //   bot->position[0] = x;
  //   //swarm->robots[q]->position[0]=x;

  //   bot->position[1] = y;
  //   swarm->robots[q]->position[1]=y;

  //   bot->orientation[0] = theta;
  //   swarm->robots[q]->orientation[0]=theta;

  //   bot->d_orientation[0] = omega;
  //   //swarm->robots[q]->d_orientation[0]=omega;
    
    
  // }
}