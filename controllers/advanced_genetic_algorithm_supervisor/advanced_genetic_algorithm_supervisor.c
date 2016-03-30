//   Description:   Supervisor code for genetic algorithm

#include "genotype.h"
#include "population.h"
#include "random.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/display.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define SIZE 10

static const int POPULATION_SIZE = 50;
static const int NUM_GENERATIONS = 25;
static const char *FILE_NAME = "fittest.txt";

// must match the values in the advanced_genetic_algorithm.c code
static const int NUM_SENSORS = 8;
static const int NUM_WHEELS  = 2;
static const int NUM_HIDDEN = 4;
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_HIDDEN + NUM_HIDDEN * NUM_WHEELS + NUM_HIDDEN * NUM_HIDDEN)

// index access
enum { X, Y, Z };

static int time_step;
static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag display;   // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles

  
// start with a demo until the user presses the 'O' key
// (change this if you want)
static bool demo = false;

int top;
int curr = 0;
int steps_till_remove = 100;
int stepsSince[SIZE];
int arr[SIZE];


void add(int x)
{
 if(top < SIZE)
 {
  arr[top] = x;
  stepsSince[top++] = steps_till_remove;
 }else{
  if(curr >= SIZE)
    curr -= SIZE;
  arr[curr] = x;
  stepsSince[curr++] = steps_till_remove;
 }
}

void checkRemoveAreas()
{
  for(int i = 0; i<top; i++)
  {
    if((top < SIZE && i != top - 1) || i != curr - 1)
    {
      stepsSince[i]--;
      if(stepsSince[i] == 0)
      {
        arr[i] = -1;
        stepsSince[i] = -1;
      }
     }
  }
}

void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 10.0;
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
    (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;
  if (generation > 0) {  
    wb_display_set_color(display, 0xff0000); // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);

    wb_display_set_color(display, 0x00ff00); // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}
 
// run the robot simulation for the specified number of seconds


double measure_fitness(){

  int i;
  double explore = 0.0;
  double fear = 0.0;
  double beta = 5.0;
  double cell_size = 0.1;
  double arm_size = 0.5;
  double centerSize = 0.1;
  int position = 1; // 0 = open arm, 1 = closed ar, 2 = center

  const double *load_trans = wb_supervisor_field_get_sf_vec3f(robot_translation);
  double dx = load_trans[X];
  double dz = load_trans[Z];

  /********** Exploration fitness *******/

  int area = 0;
  if(fabs(dx)< centerSize/2)
  {
    position = 0;
    if(fabs(dz) < centerSize/2)
    {
      position = 2;
    }
  }
  //printf("\n position is %d ", position);
  if(position == 1)
  {
    area = (int)(dx / cell_size) + (int)(arm_size/cell_size) + 1;
  }
  if(position == 0)
  {
    area = (int)(dz / cell_size) + ((int)(arm_size/cell_size) * 3) + 1;
  }

  explore = 10.0;  
  int found = 0;
  for(i=0;i<top;i++){
    if(area == arr[i])
    {
        explore = 0.0;
        found = 1;
        break;
    }
  }	
  //printf("\n FOUND is %d ", found);
  //printf("\n area is %d ", area);
  if(found == 0)
  {
    add(area);
    // Closed Arm test coords of the closed arms of the maze
    if(position == 1)
      fear = 0.05;
    // Middle test coord of the middle of the maze
    if(position == 2)
      fear = 0.1;
    // Open arm test
    if(position == 0)
      fear = 0.5;
      
    if(random_get_uniform() <fear)
    {
      fear = -1;
    }else{
      fear = 0;
    }
    
    fear *= beta;
  }
  checkRemoveAreas();
  /* Return the fitness */
  return explore + fear;
}

double run_seconds(double seconds) {
double total_fitness = 0.0;
  int i, n = 1000.0 * seconds / time_step;
  for (i = 0; i < n; i++) {
    if (demo && wb_robot_keyboard_get_key() == 'O') {
      demo = false;
      return total_fitness; // interrupt demo and start GA optimization
    }

    wb_robot_step(time_step);
    total_fitness += measure_fitness();
  }
  return total_fitness;
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {
  
  // send genotype to robot for evaluation
  wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
  
  // reset robot  position
  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);
  
  // measure fitness
  double fitness = run_seconds(60.0);
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
}

void run_optimization() {
  wb_robot_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  int i, j;
  for  (i = 0; i < NUM_GENERATIONS; i++) {    
    for (j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }
  
    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);
    
    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);
    
    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }
  
  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  }
  else
    printf("unable to write %s\n", FILE_NAME);
  
  population_destroy(population);
}
  
// show demo of the fittest individual
void run_demo() {
  wb_robot_keyboard_enable(time_step);
  
  printf("---\n");
  printf("running demo of best individual ...\n");
  printf("select the 3D window and push the 'O' key\n");
  printf("to start genetic algorithm optimization\n");

  FILE *infile = fopen(FILE_NAME, "r");
  if (! infile) {
    printf("unable to read %s\n", FILE_NAME);
    return;
  }
  
  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);
  
  while (demo)
    evaluate_genotype(genotype);
}

int main(int argc, const char *argv[]) {

  // initialize Webots
  wb_robot_init();
  
  // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");
  
  // to display the fitness evolution
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
  
  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));
  
  if (demo)
    run_demo();

  // run GA optimization
  run_optimization();
  
  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
