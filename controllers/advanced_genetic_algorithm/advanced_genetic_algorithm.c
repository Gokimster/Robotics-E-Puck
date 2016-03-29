// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define NUM_SENSORS 8
#define NUM_WHEELS 2
#define NUM_HIDDEN 4
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_HIDDEN + NUM_HIDDEN * NUM_WHEELS + NUM_HIDDEN * NUM_HIDDEN)

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS + NUM_HIDDEN + NUM_HIDDEN][NUM_HIDDEN]; 
double hidden[NUM_HIDDEN];
double rec[NUM_HIDDEN];
WbDeviceTag sensors[NUM_SENSORS];  // proximity sensors
WbDeviceTag receiver;              // for receiving genes from Supervisor

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    const double *data = wb_receiver_get_data(receiver);
    //memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));

    //input to hidden layer
    for(int i = 0; i < NUM_SENSORS; i++)
    {
      for(int j = 0; j < NUM_HIDDEN; j++)
      {
        matrix[i][j] = *data;
        data++;
      }
    }

    //hidden layer to output
    for(int i = NUM_SENSORS; i < NUM_SENSORS + NUM_HIDDEN; i++)
    {
      for(int j = 0; j < NUM_WHEELS; j++)
      {
        matrix[i][j] = *data;
        data++;
      }
    }

    //recurssive context layer to hidden layer
    for(int i = NUM_SENSORS + NUM_HIDDEN; i < NUM_SENSORS + NUM_HIDDEN + NUM_HIDDEN; i++)
    {
      for (int j = 0; j < NUM_HIDDEN; j++)
      {
        matrix[i][j] = *data;
        data++;
      }
    }

    
    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  // compute actuation using Braitenberg's algorithm:
  // The speed of each wheel is computed by summing the value
  // of each sensor multiplied by the corresponding weight of the matrix.
  // By chance, in this case, this works without any scaling of the sensor values nor of the
  // wheels speed but this type of scaling may be necessary with a different problem
  double sensor_values[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++)
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
  
  double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };
  memset(hidden, 0.0, sizeof(hidden));
  
  //input to hidden
  for(int i = 0; i < NUM_HIDDEN; i++)
  {
    hidden[i] = 0;
    for(int j = 0; j < NUM_SENSORS; j++)
    {
      hidden[i] += sensor_values[j] * matrix[j][i]; 
    }
  }

  //recurssive to hidden
  for(int i = 0; i < NUM_HIDDEN; i++)
  {
    for (int j = 0; j < NUM_HIDDEN; j++)
    {
      hidden[i] += rec[j] * matrix[NUM_SENSORS + NUM_HIDDEN + j][i];
    }
    hidden[i] = tanh(hidden[i]);
  }

  //hidden to recursive
  for(int i = 0; i < NUM_HIDDEN; i++)
  {
    rec[i] = hidden[i];
  }

  //hidden to output
  for(int i =0; i <NUM_WHEELS; i++)
  {
    for (int j = 0; j < NUM_HIDDEN; j++)
    {
      wheel_speed[i] += matrix[j + NUM_SENSORS][i] * hidden[j];
    }
    wheel_speed[i] = tanh(wheel_speed[i]) * 1000;
  }
  
  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(wheel_speed[0], wheel_speed[1]);
}

int main(int argc, const char *argv[]) {
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();
    
  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }
    
  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  // initialize matrix to zero, hence the robot 
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));

  memset(hidden, 0.0, sizeof(hidden));

  memset(rec, 0.0, sizeof(rec));
  
  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}


