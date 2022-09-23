/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/* Action Selection
 * Jeff Krichmar
 * Webots controller for "Linking Global Top-Down Views to First-Person Views in the Brain"
 * University of California, Irvine
 *
 * Description:   Robot will randomly explore the environment. Data is saved in the controllers directory every 100 timesteps.
 *    The top view image from the overhead camera is saved as a jpg
 *    The robot camera view is saved as a txt file with the red, green, and blue pixel values.
 *    The position information is saved in a text file.
 * 
 * 
 */



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#define SIM_SEED 0
#define SIM_TICKS 100000

// States
#define FIND_WALL 0
#define FOLLOW_WALL 1
#define AVOID 2
#define EXPLORE 3
#define GET_OBJECT 4
#define DROP_OBJECT 5

// Transitions
#define EXPLORE_TO_AVOID 1
#define EXPLORE_TO_GET_OBJECT 2
#define SEE_OBJECT 10

// Sub-States for Explore
#define STRAIGHT 0
#define TURN_LEFT 1
#define TURN_RIGHT 2

// Sub-states for Follow Wall
#define FOLLOW_LEFT 0
#define FOLLOW_RIGHT 1
#define FOLLOW_CLOSE 500

#define BASE_SPEED 5.0
#define TURN_RATE 1.5
#define CLOSE_DISTANCE 700
#define OBSTACLE_NEAR 100

#define OPEN_GRIP 0.029
#define CLOSED_GRIP 0.005

#define NUM_SENSORS 8
#define FRONT_LEFT 2
#define FRONT_RIGHT 3

#define FILE_WRITE_TIME 100


static WbDeviceTag sensors[NUM_SENSORS], camera, left_motor, right_motor;
static double time_step = 50;
static int image_width, image_height;
static double horizontal_width;

int grip_cnt = 0;
int explore_cnt = 0;
int simTime = 0;


/*
*   set_grip_position
*   
*   Description: Moves the gripper fingers of the 
*                Khepera gripper. Can be used to open
*                or close the gripper.
*
*   Input: pos - position to set left and right finger.
*/
// static void set_grip_position(double pos) {
  // wb_motor_set_position(left_grip, pos);
  // wb_motor_set_position(right_grip, pos);
// }

/*
*   initialize
*   
*   Description: Initializes Webots parameters
*
*/
static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  const char *robot_name = wb_robot_get_name();

  const char khepera_name[] = "ds0";

  char sensors_name[5];

  sprintf(sensors_name, "%s", khepera_name);

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);
    sensors_name[2]++;
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // motor = wb_robot_get_device("motor");
  // left_grip = wb_robot_get_device("left grip");
  // right_grip = wb_robot_get_device("right grip");

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  image_width = wb_camera_get_width(camera);
  image_height = wb_camera_get_height(camera);
  horizontal_width = (double) image_width;

  printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, NUM_SENSORS);
}

/*
*   track_object
*   
*   Description: Gets the size and horizontal position of a red object.
*   Output: obj - object array. element 0 has the horizontal position, element 1 has the number of pixels
*/
void track_object (double *obj) {
   const unsigned char *image = wb_camera_get_image(camera);
   double r, g, b;
   r = g = b = 0.0;
   unsigned char red, grn, blu;
   FILE *fp;
   char fn[256];
   obj[0] = 0.0; // object position 
   obj[1] = 0.0; // object size 
   
    if ((simTime % FILE_WRITE_TIME) == 0) {
       sprintf(fn, "camera_%i.txt", SIM_SEED*SIM_TICKS+simTime);
       fp = fopen(fn, "w");
    }   
   // go through the camera frame and find the red pixels
   for (int x = 0; x < image_width; x++) {
       for (int y = 0; y < image_height; y++) {
          red = wb_camera_image_get_red(image, image_width, x, y);
          grn = wb_camera_image_get_green(image, image_width, x, y);
          blu = wb_camera_image_get_blue(image, image_width, x, y);
          if ((simTime % FILE_WRITE_TIME) == 0) {
              fprintf(fp,"%i %i %i ", red, grn, blu);
          }
          r = (double)red/256.0;
          g = (double)grn/256.0;
          b = (double)blu/256.0;
          if (r > 2*g && r > 2*b)  {
             obj[0] += ((double)x - horizontal_width/2.0)/horizontal_width;
             obj[1]++;
          }
          else if (g > 2*r && g > 2*b)  {
             obj[0] += ((double)x - horizontal_width/2.0)/horizontal_width;
             obj[1]++;
          }
          else if (b > 2*r && b > 2*g)  {
             obj[0] += ((double)x - horizontal_width/2.0)/horizontal_width;
             obj[1]++;
          }
       }
   }
   if ((simTime % FILE_WRITE_TIME) == 0) {
      fprintf(fp,"\n");
      fclose(fp);
   }
}

/*
*   rand01
*   
*   Description: returns a random number between 0 and 1.
*/
double rand01() {
   return (double)rand()/(double)RAND_MAX;
}


/*
*   STATE: find_wall
*   
*   Description: Drives straight until the robot finds a wall with its distance sensors.
*                Orients the robot perpindicular to the wall.              
*   Input: ds - distance sensors for Khepera
*   Outputs: velo - speed settings for the robot
*            complete - returns true when state is completed.
*/
bool find_wall (double *ds, double *velo) {
   bool complete = false;
   bool close = false;
   double facingLeft = 0.0;
   double facingRight = 0.0;
   int numSensors = NUM_SENSORS-2;
   
   // only use the front facing sensors
   for (int i = 0; i < numSensors; i++) {
      if (ds[i] > CLOSE_DISTANCE) {
        close = true;
      }
      if (i < numSensors/2) {
         facingLeft += ds[i];
      }
      else {
         facingRight += ds[i];
      } 
   }
   
   if (close) {
      if (fabs(ds[FRONT_LEFT] - ds[FRONT_RIGHT]) < 50) {
         complete = true;
      }
      else if (facingLeft > facingRight) {
         velo[0] = -1.0;
         velo[1] = 1.0;
      }
      else { // (facingRight > facingLeft) 
         velo[0] = 1.0;
         velo[1] = -1.0;
      }
   }
   else {
      velo[0] = BASE_SPEED;
      velo[1] = BASE_SPEED;
   }
   return complete;
}

/*
*   STATE: follow_wall
*   
*   Description: Uses the distance sensors to follow close to a wall.              
*   Inputs: side - side of the robot closest to the wall
*           wallTime - timer for how long to follow the wall
*           ds - distance sensors for Khepera
*           
*   Outputs: velo - speed settings for the robot
*            complete - returns true when followed wall for specified amount of time.
*/
bool follow_wall (int side, int wallTime, double *ds, double *velo) {
   bool complete = false;
   bool close = false;
   int numSensors = NUM_SENSORS-2;
 
   if (side == FOLLOW_LEFT) {     
      // check if any of the left sensors are too close
      for (int i = 0; i < numSensors/2; i++) {
         if (ds[i] > FOLLOW_CLOSE) {
            close = true;
         }
      }
      if (close) {
         velo[0] = 2*BASE_SPEED;
         velo[1] = 0.0;
      }
      else {
         velo[0] = 0.0;
         velo[1] = 2*BASE_SPEED;
      }
   }
   else {
      // check if any of the right sensors are too close
      for (int i = numSensors/2; i < numSensors; i++) {
         if (ds[i] > FOLLOW_CLOSE) {
            close = true;
         }
      }
      if (close) {
         velo[0] = 0.0;
         velo[1] = 2*BASE_SPEED;
      }
      else {
         velo[0] = 2*BASE_SPEED;
         velo[1] = 0.0;
      }   
   }

   if (wallTime < 0) {
      complete = true;
   }
   return complete;
}

/*
*   STATE: explore
*   
*   Description: Peforms a random walk by sometimes going straight and other times turning.              
*   Inputs: turn - signal to turn left, right, or straight.
*           object - number of pixels in the field of view that contain the object, and the horizontal position of the object.
*           ds - distance sensors for Khepera
*           
*   Outputs: velo - speed settings for the robot
*            complete - returns 1 (transition to avoid state) or 2 (transition to get object state) when completed.
*/
int explore (int turn, double *object, double *ds, double *velo) {
   bool complete = false;

   int numSensors = NUM_SENSORS-2;
   bool obstacle = false;
   ++explore_cnt;
   
   // check for obstacles
   for (int i = 0; i < numSensors; i++) {
      if (ds[i] > OBSTACLE_NEAR) {
         obstacle = true;
      }
   }
   // found obstacle, transition to avoid state
   if (obstacle) {
      velo[0] = 0.0;
      velo[1] = 0.0;
      complete = EXPLORE_TO_AVOID;
   }
   // found object, transition to get object state
   // JLK - No object tracking - 22 Sept 2021
   // else if ((object[1] > SEE_OBJECT) ) {
      // velo[0] = 0.0;
      // velo[1] = 0.0;
      // complete = EXPLORE_TO_GET_OBJECT;
   // }
   // explore by driving left, right, or straight
   else {
      if (turn == TURN_LEFT) {
         velo[0] = BASE_SPEED;
         velo[1] = BASE_SPEED*1.15;
      }
      else if (turn == TURN_RIGHT) {
         velo[0] = BASE_SPEED*1.15;
         velo[1] = BASE_SPEED;
      }
      else  {
         velo[0] = BASE_SPEED;
         velo[1] = BASE_SPEED;
      }
   }
   return complete;
}

/*
*   STATE: avoid
*   
*   Description: Uses distance sensors to rotate away from an obstacle.              
*   Inputs: turn - signal to turn left, right, or straight.
*           ds - distance sensors for Khepera
*           
*   Outputs: velo - speed settings for the robot
*            complete - returns true when clear of obstacle.
*/
bool avoid (double *ds, int turn, double *velo) {
   bool complete = false;
   double facingLeft = 0.0;
   double facingRight = 0.0;
   int numSensors = NUM_SENSORS-2;
   
   // only use the front facing sensors
   for (int i = 0; i < numSensors; i++) {
      if (i < numSensors/2) {
         facingLeft += ds[i];
      }
      else {
         facingRight += ds[i];
      }    
   }
   // printf("fL=%f fR=%f\n", facingLeft, facingRight);
   
   // sensors are clear avoid state is complete
   if ((facingLeft == 0) && (facingRight == 0)) {
      complete = true;
   }
   // obstacle on the left, rotate right
   // else if (facingLeft > facingRight) {
   else if (turn == TURN_RIGHT) {
      velo[0] = 1.0;
      velo[1] = -1.0;
   }
   // obstacle on the right, rotate left
   // else if (facingRight > facingLeft) {
   else {
      velo[0] = -1.0;
      velo[1] = 1.0;
   }
   // rare chance left and right are equal, back up and turn
   // else {
      // velo[0] = -1.0;
      // velo[1] = -1.0;
   // }
   return complete;
}

/*
*   STATE: get_object
*   
*   Description: Uses camera and distance sensors to get close to an object. Once lose, picks up the obejct              
*   Inputs: object - number of pixels in the field of view that contain the object, and the horizontal position of the object.
*           ds - distance sensors for Khepera
*           
*   Outputs: velo - speed settings for the robot
*            complete - returns true when object is picked up.
*/
bool get_object (double *object, double *ds, double *velo) {
   bool complete = false;
   double turn;
   
   // close to object, stop going to object
   if ((ds[FRONT_LEFT] >  CLOSE_DISTANCE) || (ds[FRONT_RIGHT] >  CLOSE_DISTANCE)) {
      complete = true;
   }
   // track towards the object based on its size and postion.
   else if ((object[1] > SEE_OBJECT)) {
      turn = object[0]/object[1]*TURN_RATE;       
      velo[0] = BASE_SPEED+turn;
      velo[1] = BASE_SPEED-turn;
   }
   // rotate in place until find object again
   else {
      velo[0] = BASE_SPEED/2;
      velo[1] = -BASE_SPEED/2;
   }
   return complete;
}

// get the distance between two vectors. assuming these are 2D vectors
double distance(double x1, double x2, double y1, double y2) {
    return sqrt(pow(x1-y1,2.0) + pow(x2-y2,2.0));
}


/*
*   STATE: drop_object
*   
*   Description: Drops off object in gripper           
*           
*   Outputs: velo - speed settings for the robot
*            complete - returns true when gripper releases object
*/
// bool drop_object (double *velo) {

   // bool complete = false;

   // if (grip_cnt == 20) {
      // wb_motor_set_position(motor, 0.0); /* arm down */
   // }
   // else if (grip_cnt == 40) {
      // set_grip_position(OPEN_GRIP); /* open the gripper */
   // }
   // else if (grip_cnt == 60) {
       // wb_motor_set_position(motor, -1.4); /* arm up */
   // }
   // else if (grip_cnt == 80) {
      // complete = true;
   // }
   // ++grip_cnt;
   // velo[0] = 0.0;
   // velo[1] = 0.0;
   // return complete;
// }

/*
*   MAIN ROUTINE
*   
*   Description: Controls the action selection of a Khepera robot. Robot explores, finds objects to pick up and drop
*                off. Robot may follow walls between looking for objects.  Uses a state transition control similar
*                to the subsumption architecture           
*           
*/
int main() {
  int state = EXPLORE; 
  int turn = STRAIGHT;
  double speed[2];
  double object[2];
  double sensors_value[NUM_SENSORS];
  double r;
  int explore_state;
  char snapFn [256];
  char posFn [256];
  FILE *posFp;

  srand(SIM_SEED);
   
  initialize();
  
  // use supervisor to get and set position of the stick.
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef robot_trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef robot_rot_field = wb_supervisor_node_get_field(robot_node, "rotation");

  WbNodeRef red_stick_node = wb_supervisor_node_get_from_def("RED_STICK");
  WbFieldRef red_trans_field = wb_supervisor_node_get_field(red_stick_node, "translation");
  const double *red_trans_value = wb_supervisor_field_get_sf_vec3f(red_trans_field);
 
  WbNodeRef green_stick_node = wb_supervisor_node_get_from_def("GREEN_STICK");
  WbFieldRef green_trans_field = wb_supervisor_node_get_field(green_stick_node, "translation");
  const double *green_trans_value = wb_supervisor_field_get_sf_vec3f(green_trans_field);

  WbNodeRef blue_stick_node = wb_supervisor_node_get_from_def("BLUE_STICK");
  WbFieldRef blue_trans_field = wb_supervisor_node_get_field(blue_stick_node, "translation");
  const double *blue_trans_value = wb_supervisor_field_get_sf_vec3f(blue_trans_field);

  // printf("Explore\n");
  
  r = rand01();
  if (r < 0.25) {
     turn = TURN_LEFT;
  }
  else if (r < 0.50) {
     turn = TURN_RIGHT;
  }
  else {
     turn = STRAIGHT;
  }
  
  while (simTime < FILE_WRITE_TIME*1000) {
    wb_robot_step(time_step);

    for (int i = 0; i < NUM_SENSORS; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }
    
    // look for objects (red stick in this simulation).
    track_object(object);
    
    if ((simTime % FILE_WRITE_TIME) == 0) {
       sprintf(snapFn, "simView_%i.jpg", SIM_SEED*SIM_TICKS+simTime);
       wb_supervisor_export_image(snapFn, 100);
       const double *robot_trans_value = wb_supervisor_field_get_sf_vec3f(robot_trans_field);
       const double *robot_rot_value = wb_supervisor_field_get_sf_rotation(robot_rot_field);
       sprintf(posFn, "position_%i.txt", SIM_SEED*SIM_TICKS+simTime);
       posFp = fopen(posFn, "w");
       fprintf(posFp, "%f %f %f %f %f %f %f %f %f %f\n", 
          robot_trans_value[0], 
          robot_trans_value[1], 
          robot_trans_value[2], 
          robot_rot_value[0], 
          robot_rot_value[1], 
          robot_rot_value[2], 
          robot_rot_value[3],
          distance(robot_trans_value[0], robot_trans_value[2], red_trans_value[0], red_trans_value[2]),
          distance(robot_trans_value[0], robot_trans_value[2], green_trans_value[0], green_trans_value[2]),
          distance(robot_trans_value[0], robot_trans_value[2], blue_trans_value[0], blue_trans_value[2]));
       fclose(posFp); 
       if ((simTime % (100*FILE_WRITE_TIME)) == 0) {
          printf("simTime = %i\n", simTime/FILE_WRITE_TIME); 
       }
    }
    ++simTime;
    
    // Switches between states.  Once a state is complete, there is a transition to a new
    // state.
    switch (state) {
       case EXPLORE:
          // explore can transition to either the avoid or the get object state.
          explore_state = explore(turn, object, sensors_value, speed);
          if (explore_state == EXPLORE_TO_AVOID) {
             state = AVOID;
             r = rand01();
             if (r < 0.5) {
               turn = TURN_LEFT;
             }
             else {
               turn = TURN_RIGHT;
             }             
             // printf("Avoid\n");
          }
          else if (explore_state == EXPLORE_TO_GET_OBJECT) {     
             state = GET_OBJECT;
             // printf("Get Object\n");
          }
          break;

       case AVOID:
          if (avoid(sensors_value, turn, speed)) {
             // before transitioning to explore, set the new explore direction
             // JLK - Don't turn as much 22 SEPT 2021
             r = rand01();
             if (r < 0.25) {
               turn = TURN_LEFT;
             }
             else if (r < 0.50) {
               turn = TURN_RIGHT;
             }
             else {
               turn = STRAIGHT;
             }
             state = EXPLORE;
             // printf("Explore\n");
          }
          break;
       case GET_OBJECT:
          if (get_object(object, sensors_value, speed)) {
             state = AVOID;
             // printf ("Avoid\n");
          }
          break;
       // case DROP_OBJECT:
          // if (drop_object(speed)) {
             // once the stick has been dropped off, put it back.
             // wb_supervisor_field_set_sf_vec3f(trans_field, trans_value);
             // wb_supervisor_field_set_sf_rotation(rot_field, rot_value);
             
             // after dropping off the stick, half the time follow the wall to the left or right.
             // r = rand01();
             // if (rand01() < 0.5) {
                // wallSide = FOLLOW_LEFT;
             // }
             // else {
                // wallSide = FOLLOW_RIGHT;
             // }
             // wallClock = 1000 + (int)(rand01()*1000.0);
             // state = FOLLOW_WALL;
             // printf("Follow Wall\n");
          // }
          // break;
       default:
          printf("ERROR: bad state %i\n", state);
    }
    
    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  return 0;
}
