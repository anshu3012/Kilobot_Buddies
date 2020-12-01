#include <vector>
#include <iostream>

// int case1(int a,int b,int c, int d, int e)
// {
//  for (int i; i<10;i++)
//  {
//      break:
//      for (int j; j<10;j++)
//      {
//         if (a>b)
//         {
//             break;
//         }

//      }

//     char my_name[] = "Broke out of 1 loop";
//     printf("My name is %s \n", my_name);
//  }
// char my_name[] = "Broke out of 2nd loop";
// printf("My name is %s \n", my_name);
// } 

// #include <iostream>
// int num_robots=10;
// int main()
// {
//     // for loop with two variable i & j
//     // i will start with 0 and keep on incrementing till 10
//     // j will start with 10 and keep on decrementing till 0
//     for (int i = 0, j = num_robots/2; i < (num_robots/2) && j < num_robots; i++, j++)
//     {
//         std::cout << "i = " << i << " :: " << "j = " << j << std::endl;
//     }
//     return 0;
// }



// void makebuddies (SWARM* swarm, float x_pos, float y_pos)
// {
//     loop all robots 
//         for ith robot check distance 
//         if distance is greater than 2*radius 
//             do nothing 
//         if distance is less tahn 2*radius
//             set buddy flag of that robot and current robot to 1 

        


// }

 
 
// #include <vector>
// #include <iostream>
// int main() 
// { 
// 	//declaring vector of pairs 
// 	std::vector< std::pair <int,int> > vect; 

// 	// initialising 1st and 2nd element of 
// 	// pairs with array values 
// 	int arr[] = {10, 20, 5, 40 }; 
// 	int arr1[] = {30, 60, 20, 50}; 
// 	int n = sizeof(arr)/sizeof(arr[0]); 
    
// 	// Entering values in vector of pairs 
// 	for (int i=0; i<n; i++) 
// 		vect.push_back( std::make_pair(arr[i],arr1[i]) ); 

// 	// Printing the vector 
// 	for (int i=0; i<n; i++) 
// 	{ 
// 		// "first" and "second" are used to access 
// 		// 1st and 2nd element of pair respectively 
// 		std::cout << vect[i].first << " "
// 			<< vect[i].second << std::endl; 
// 	} 
// 	return 0; 
// } 


// #include<iostream>
// using namespace std;
// int main() 
// {
// 	int x, n;
// 	cout << "Enter the number of items:" << "\n";
// 	cin >>n;
// 	int *arr = new int(n);
// 	cout << "Enter " << n << " items" << endl;
// 	for (x = 0; x < n; x++) {
// 		cin >> arr[x];
// 	}
// 	cout << "You entered: ";
// 	for (x = 0; x < n; x++) {
// 		cout << arr[x] << " ";
// 	}
// 	return 0;
// }

//#include<iostream>


// std::vector< std::pair <int,int> > foo() 
// {
//     std::vector< std::pair <int,int> > vect; 
//     for (int i = 0, j = 5; i < 5 && j < 10; i++, j++)
//     {
//         vect.push_back( std::make_pair(i,j) );
//         std::cout << "i is " << i << std::endl;
//         std::cout << "j is " << j << std::endl;
//     }
//     for (int k=0; k<5; k++)
//     {
//         std::cout << "k is " << k << std::endl;
//         std::cout << "The number is " << vect[k].first << std::endl;
//     }
//     return vect;
// }

// int main()
// {
//     //std::vector<std::pair<int, int>> vect = foo();
//     auto vect=foo();
//     std::cout << "The xxxxx is " << vect[0].second << std::endl;
// }


// void drive()
// {
//     for (int i=0;i<5;i++)
//     {
//         if ((vect[i].first==swarm->robot[i]->id)||(vect[i].second==swarm->robot[i]->id))
//         {

//             move robot[i]
//         }
        
//     }
// }


void drive_buddy_swarm_for_duration(ROBOT* bot, float dt, SWARM* swarm)
{
  for (int i = 0, j = 0; i < swarm->num_robots && j < (swarm->num_robots/2); i++, j++)
    {
        now_computing_with(swarm->robots[i]);
        drive_buddies_for_duration(swarm->robots[i], swarm, dt);
        done_computing_with(swarm->robots[i]);
    }
}


void drive_buddies_for_duration(ROBOT* bot, swarm,  float dt);
{
    for (int i=0; i<(swarm->num_robots/2); i++)
    {
        if ((swarm->vect[i].first==bot->id)||(swarm->vect[i].second==bot->id)) // checking at which index of the vector is our jth robot 
        {
            // if (swarm->vect[i].first==bot->id) // if the bot id corresponds to the first unit in buddy
            //{
            drive_robot_for_duration(bot, float dt);

            for (int j=0; j<NUMROBTS;j++)
            {
                if (swarm->robots[j]->id==swam->vect[i].second)
                {
                    break;
                }   
            }
            swarm->robots[j]->position[0]= position[0]+2*swarm->robot_radius;
            swarm->robots[j]->position[1]=bot->position[1]+2*swarm->robot_radius;
            swarm->robots[j]->orientation[0]= bot->orientation[0];
        }
    }
}







    if ((swarm->vect[i].first==bot->id)||(swarm->vect[i].second==bot->id))
        {
             if (swarm->vect[i].first==bot->id)
             {
                 //move the bot with id a1 
                 
                 for (int j=0; j<NUMROBTS;j++)
                 {
                    if (swarm->robots[i]->id==swam->vect[i].second)
                    break;
                 }

                 change position accordingly 
             }

             else 
             {
                
             }


        }
    }

  float time_resolution = 0.01;   // time resolution
  int last_loop = 0;
  float max_speed_in_rads_per_sec = 1.0f;  // (max speed in robot radii per second)
  float max_speed_while_turning_in_rads_per_sec = 0.4;  // (max speed in robot radii per second)
  // note that bot->speed is the current ratio of max alowed speed

  if((bot->speed[0] <= 0.0f && bot->speed[1] <= 0.0f) || dt <= 0.0f)
  {
    // robot is stopped

    bot->d_orientation[0] = 0.0f;
    return;
  }

  // assuming tank steering is a good enough approximation
  float x = bot->position[0];
  float y = bot->position[1];
  float theta = bot->orientation[0];


  // if robot is moving straight
  if(fabs(bot->speed[1] - bot->speed[0]) < 0.0001f)  // i.e., equal to rough numerical precision
  {
    float speed = ((bot->speed[1] + bot->speed[0])/2.0f)*max_speed_in_rads_per_sec*bot->radius;

    float dx = dt*cos(theta)*speed;
    float dy = dt*sin(theta)*speed;

    bot->position[0] += dx;
    bot->position[1] += dy;

    bot->d_orientation[0] = 0.0f;
    return; 
  }
    
  // otherwise robot is turning

  float v_l = bot->speed[0]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of left wheel in robot radii per second
  float v_r = bot->speed[1]*max_speed_while_turning_in_rads_per_sec*bot->radius;  // speed of center of right wheel in robot radii per second

  float R = bot->radius * (v_l + v_r)/(v_r - v_l);  // radius of rotation
  float omega = (v_r - v_l)/bot->radius;            // speed of rotation (around center of rotation)

  float c_x = x - R*sin(theta); // center of rotation x
  float c_y = y + R*cos(theta); // center of rotation y

  x = cos(omega*dt)*(x - c_x) - sin(omega*dt)*(y - c_y) + c_x;
  y = sin(omega*dt)*(x - c_x) + cos(omega*dt)*(y - c_y) + c_y;
  theta = theta + omega*dt;

  while(theta > 2.0f*PI)
  {
    theta -= 2.0f*PI;
  }
  while(theta < -2.0f*PI)
  {
    theta += 2.0f*PI;
  }


  bot->position[0] = x;
  bot->position[1] = y;
  bot->orientation[0] = theta;

  bot->d_orientation[0] = omega;
}