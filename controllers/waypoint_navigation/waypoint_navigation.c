#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/pen.h>
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <prm.h>

#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
#define TRUE 1
#define MAX_COMPASS_VALUE 180

//compass values are converted to degrees in range of -180 to +180

typedef int boolean;
int workspace[33][33];



double left_wheel_speed, right_wheel_speed;

double target_xl=     -0.2;
double target_zl=      0.5;

double get_target_bearing(WbDeviceTag gps,double target_xl,double target_zl)
{
  const double *currentposition =(wb_gps_get_values(gps));
 
  double slope=atan((currentposition[2]-target_zl)/(currentposition[0]-target_xl));
  
  if(target_xl>currentposition[0])
  {
    slope += M_PI;
  }

  
  double slopetonextangle=(slope-1.5708)/M_PI*180;
  if (slopetonextangle<0.0)
  {
    slopetonextangle=360+slopetonextangle;

  }  
  return (slopetonextangle-180)*-1.0;
  

}


double get_current_bearing_in_degrees(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0],north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  
	if (bearing < 0.0)
    bearing=360+bearing;

  
	return (bearing-180) * -1.0;
}




double difference(double currentBearings,double destinationBearings){
  double diff= destinationBearings - currentBearings;
  if (diff>180.0)
  {
    diff=diff-360.0;
    
  }
  if(diff<-180.0)
  {
    diff=diff+360.0;
  }
  return diff;
}




int main(int argc, char *argv[]) 
{

  wb_robot_init();
  WbDeviceTag pen;
  pen = wb_robot_get_device("pen");
  wb_pen_write(pen, 1);
  
  struct vertex goal;
  goal.Vx=target_xl;
  goal.Vz=target_zl;
  goal.visited=0;
  goal.previous=0;
  
  
  printf("1goal.vx=%.2f",goal.Vx);
  printf("1goal.vz=%.2f",goal.Vz);
  
  

  double current_bearing, error_bearing, last_error_bearing, cumulative_error_bearing, change_error_bearing;
  double dt = 1; // duration of one time step for pid output calculation
  double Kp_compass = 1.0, Ki_compass = 0.0, Kd_compass =0; // pid constants for compass
  double Kp_gps=200.0, Ki_gps = 0.0, Kd_gps =0;
  double output = 0; // pid output
  double epsilon_compass = 0.5; // max error for convergence
  double epsilon_gps= 0.005;
  int time_step = 0;
  
  double compass_noise;
  
  
  error_bearing = last_error_bearing = cumulative_error_bearing = 0;
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass,TIME_STEP);

  /* main loop */
   WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);

  struct vertex start;
  start.Vx = 0;
  while(start.Vx == 0)
  {
  const double *currentposition =(wb_gps_get_values(gps));
  start.Vx= currentposition[0];
  start.Vz= currentposition[2];
  start.previous=0;
  start.visited=0;
  printf("Invalid gps\n");
  wb_robot_step(TIME_STEP);
  }
  printf("start point = (%.2f, %.2f)\n", start.Vx, start.Vz);


  FILE *fp;
  char *mode = "r";
  char str[66];
  char* read;

  fp = fopen("input.txt", mode);

  if(fp == NULL) 
   {
      perror("Error opening file");
      return(-1);
   }
  int x=31;

  while((read=fgets(str,66,fp))!=NULL)
  {
    if(strcmp(str, " \n")==0 || strcmp(str, "\n")==0)
      continue;
   
    printf("%s", str);
    int z= 0;
    char * pch;
    printf ("Splitting string \"%s\" into tokens:\n",str);
    pch = strtok (str," ,.-");
    printf ("%s\n",pch);
    while (pch != NULL)
    {
      pch = strtok (NULL, " ,.-");
      
      if(pch != NULL && strcmp(pch, "\r") != 0 && strcmp(pch, "\n") != 0)
      {
        printf ("%s\n",pch);
        int c=atoi(pch);
        
        workspace[x][z]=c;
        z++;
      }
    }
    x--;
  }

  fclose(fp);

  
  printf("read input file\n");
  
  
  
  prm_build_roadmap(workspace);
  
  printf("built roadmap\n");
  
  int pathLength;
  struct vertex* path = prm_query(start,goal,workspace, &pathLength);
  
  
  printf("Queried for path\n");
  

    printf("2goal.vx=%.2f",goal.Vx);
  printf("2goal.vz=%.2f\n",goal.Vz);
  printf("path size = %d\n", pathLength);


int i;
for(i=0;i<pathLength;i++)
{
 printf("(%f,%f)\n",path[i].Vx,path[i].Vz);
}


 for(i=0; i<pathLength; i++)
 {
     for(;;)
       {
           time_step++;      
  //get current bearing and add random uniform noise in -0.5 to 0.5
           compass_noise = ((double) rand()/RAND_MAX)-0.5;
           current_bearing = get_current_bearing_in_degrees(compass) + compass_noise; 
  

  // calculate the terms for the pid equation
          last_error_bearing = error_bearing;   // differential term

 
          double current_angle= get_target_bearing(gps, path[i].Vx,path[i].Vz);
          error_bearing = difference (current_bearing,current_angle );
          change_error_bearing= error_bearing - last_error_bearing;
          if (abs(cumulative_error_bearing) < MAX_COMPASS_VALUE)
            cumulative_error_bearing += error_bearing;  //integral term, only add if error > epsilon_error
  
  // the pid equation
          output = Kp_compass * error_bearing + (Ki_compass * cumulative_error_bearing * dt) + (Kd_compass * change_error_bearing /dt);
  
          printf ("At step: %d: current bearing = %.2f, error_bearing = %.2f, cumulative_error_bearing = %.2f, control output = %.2f, current_angle=  %.2f\n", time_step, current_bearing, error_bearing, cumulative_error_bearing, output,current_angle);
  
          if (output > epsilon_compass || output < -epsilon_compass){
            left_wheel_speed = 50* -output;
           right_wheel_speed =50* output;
          }
                   else {
            printf("Robot stopped turning at step %d\n",time_step);
            left_wheel_speed = right_wheel_speed = 0;
            break;
          }
            
          wb_differential_wheels_set_speed(left_wheel_speed,right_wheel_speed);
          wb_robot_step(TIME_STEP);
             
       }
          
          const double *initGps = wb_gps_get_values(gps);
          
          double initialPoint[2];
          initialPoint[0] = initGps[0];
          initialPoint[1] = initGps[2];
          
        for(;;)
        {
          time_step++;      
          
          const double *currentposition = wb_gps_get_values(gps);
         
          // calculate the terms for the pid equation
          last_error_bearing = error_bearing;   // differential term
          
  
          error_bearing=hypot((currentposition[0] - path[i].Vx), (currentposition[2] - path[i].Vz));//error term

          if( hypot((currentposition[0] - initialPoint[0]), (currentposition[2] - initialPoint[1])) > hypot((initialPoint[0] - path[i].Vx), (initialPoint[1] - path[i].Vz)))
          {
            error_bearing=(error_bearing*(-1));
          }

          change_error_bearing= error_bearing - last_error_bearing;
          
         if(change_error_bearing > 0.25 || change_error_bearing < -0.25)
            change_error_bearing = 0;
          
          
          if ( error_bearing < 10)
            cumulative_error_bearing += error_bearing;  //integral term, only add if error > epsilon_error
          
          // the pid equation
          output = Kp_gps * error_bearing + (Ki_gps * cumulative_error_bearing * dt) + (Kd_gps * change_error_bearing /dt);
          
          printf ("At step: %d: initial = (%.2f, %.2f), current position (%.2f,%.2f), goal=(%.2f, %.2f),prmgoal=(%.2f, %.2f),error_distance = %.2f, change_error_bearing = %.2f, cumulative_error_distance = %.2f, control output = %.2f\n", time_step, initialPoint[0], initialPoint[1], currentposition[0], currentposition[2],path[i].Vx,path[i].Vz,goal.Vx,goal.Vz, error_bearing, change_error_bearing, cumulative_error_bearing, output);
          
          
          if (output > epsilon_gps){
            left_wheel_speed =  200*output;
            right_wheel_speed = 200*output;
          }
          else {
            printf("Robot stopped turning at step %d\n",time_step);
            left_wheel_speed = right_wheel_speed = 0;
            break;
          }
            
          wb_differential_wheels_set_speed(left_wheel_speed,right_wheel_speed);
          wb_robot_step(TIME_STEP);
             
          
          
          
          
        }
  
  }
  

  return 0;
}
