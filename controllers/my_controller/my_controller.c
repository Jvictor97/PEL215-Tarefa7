#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64

// Velocidade aplicada nos motores
#define SPEED 6.28

// Velocidade negativa aplicada em movimentos na diagonal
#define DIAGONAL_ZERO -1.13425

typedef struct {
 double x;
 double z;
} Point;

typedef struct {
 int row;
 int column;
} Cell;

typedef void (*Function)();

typedef struct {
 Cell cell;
 Function function;
} Movement;

Point goal = { 7.125, -7.125 };

// Distância de influência dos obstáculos
double Ro0 = 3.0;

// Array para referenciar os motores
WbDeviceTag motors[4];
const double katt = 1.0;
const double krep = 999.9;

/* Funcoes para configurar os motores */

// Funcao para configurar a velocidade nos 4 motores
void setSpeed(double* speeds) {
  for (int i = 0; i < 4; i++)
    wb_motor_set_velocity(motors[i], speeds[i]);
}

// Move o robo na diagonal para frente e para a esquerda
void frontLeft() {
  double speeds[4] = {SPEED, DIAGONAL_ZERO, DIAGONAL_ZERO, SPEED};
  setSpeed(speeds);
}

// Move o robo para frente
void front() {
  double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  setSpeed(speeds);
}

// Move o robo na diagonal para frente e para a direita
void frontRight() {
  double speeds[4] = {DIAGONAL_ZERO, SPEED, SPEED, DIAGONAL_ZERO};
  setSpeed(speeds);
}

// Move o robo para esquerda
void left() {
  double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  setSpeed(speeds);
}

// Move o robo para a direita
void right() {
  double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  setSpeed(speeds);
}

// Move o robo na diagonal para tras e para a esquerda
void backLeft() {
  double speeds[4] = {-DIAGONAL_ZERO, -SPEED, -SPEED, -DIAGONAL_ZERO};
  setSpeed(speeds);  
}

// Move o robo para tras
void back() {
  double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  setSpeed(speeds);
}

// Move o robo na diagonal para tras e para a direita
void backRight() {
  double speeds[4] = {-SPEED, -DIAGONAL_ZERO, -DIAGONAL_ZERO, -SPEED};
  setSpeed(speeds);
}


// Funcao para comparar dois valores double, os valores
// sao considerados iguais caso a diferenca absoluna 
// seja menor que 0.1
bool equal(double firstValue, double secondValue) {
  const double tolerance = 0.1;
  
  return fabs(firstValue - secondValue) < tolerance;
}

double euclidianDistance(Point p1, Point p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.z - p2.z, 2));
}

double potential(double row, double column) {
  Point obstacles[7] = {
   { -4.875, -3.375 },
   { -0.375, -5.625 },
   {  4.875, -3.375 },
   { -0.375, -0.375 },
   {  4.125,  1.125 },
   { -3.375,  3.375 },
   {  1.875,  4.875 }
  };
  
  double x = 0.75 * column - 7.125;
  double z = 0.75 * row - 7.125;
  
  Point cellPosition = { x, z };
  
  double goalDistance = euclidianDistance(cellPosition, goal);
  
  double Uatt = 0.5 * katt * pow(goalDistance, 2);
  
  double Urep = 0.0;
  
  for(int i = 0; i < 7; i++) {
    double Roq = euclidianDistance(cellPosition, obstacles[i]);
    
    if (Roq > Ro0) continue;
  
    Urep += 0.5 * krep * pow((1/Roq - 1/Ro0), 2);
  }
  
  return Uatt + Urep;
}

void calculateMatrixPotentials(double matrix[][20]) {
  for(int row = 0; row < 20; row++) {
    for(int column = 0; column < 20; column++) {
      matrix[row][column] = potential(row, column);
    }
  }
}

void printMatrix(double matrix[][20]){
  for (int row = 0; row < 20; row++) {
    for (int column = 0; column < 20; column++) { 
      printf("%.2f ", matrix[row][column]);  
   }
    printf("\n");
 }
 
  printf("\n\n");
}

bool validCell(int row, int column) {
  return row >= 0 && row <= 19 && column >= 0 && column <= 19;
}

Function defineFunction(Cell currentCell, Cell nextCell) {
  if (nextCell.row < currentCell.row) {
    if (nextCell.column < currentCell.column) return frontLeft;
    if (nextCell.column == currentCell.column) return front;
    
    return frontRight;
  }
  else if (nextCell.row == currentCell.row) {
    if (nextCell.column < currentCell.column) return left;
    
    return right;
  }
  else { // Next row > Current row
    if (nextCell.column < currentCell.column) return backLeft;
    if (nextCell.column == currentCell.column) return back;
    
    return backRight;
  }
}

Movement gradientDescent(Cell currentCell, double matrix[20][20]) {
  double minimumPotential = matrix[currentCell.row][currentCell.column];
  Cell nextCell = currentCell;
  Movement movement;

  for (int row = currentCell.row - 1; row <= currentCell.row + 1; row++) {
    for (int column = currentCell.column - 1; column <= currentCell.column + 1; column++) {
      if (!validCell(row, column)) continue;
      double cellPotential = matrix[row][column];
      
      if (cellPotential < minimumPotential) {
        minimumPotential = cellPotential;
        nextCell = (Cell) { row, column };
      }
    }
  }
  
  movement.cell = nextCell;
  movement.function = defineFunction(currentCell, nextCell);
  return movement;
}

double toCoordinate(int index) {
  return 0.75 * index - 7.125;
}

// Funcao principal
int main(int argc, char **argv) {
  // Inicializando o webots
  wb_robot_init();
   
  // Setando a referencia dos motores
  motors[0] = wb_robot_get_device("wheel1"); // front right
  motors[1] = wb_robot_get_device("wheel2"); // front left
  motors[2] = wb_robot_get_device("wheel3"); // back right
  motors[3] = wb_robot_get_device("wheel4"); // back left
  
  // Configurando a posicao dos motores
  wb_motor_set_position(motors[0], INFINITY);
  wb_motor_set_position(motors[1], INFINITY);
  wb_motor_set_position(motors[2], INFINITY);
  wb_motor_set_position(motors[3], INFINITY);
  
  // Referencia para o robo, utilizada pelo supervisor  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  
  // Capturando a informacao de translacao do robo
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  double matrix[20][20];
  calculateMatrixPotentials(matrix);
  printMatrix(matrix);
  
  Cell currentCell = { 19, 0 };

  Movement nextMovement = gradientDescent(currentCell, matrix);
  printf("current: [%d][%d], nextRow: %d, nextColumn: %d\n", 
      currentCell.row, currentCell.column,
      nextMovement.cell.row, nextMovement.cell.column);
      
  // Loop principal
  while (wb_robot_step(TIME_STEP) != -1) {      
    double x, z;
    // //Lendo a posicao do robo no ambiente
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    // //Atribuindo valores da posicao em X e Z
    x = values[0];
    z = values[2];
    
    if (equal(x, goal.x) && equal(z, goal.z)) {
      for (int i = 0; i < 4; i++)
        wb_motor_set_velocity(motors[i], 0);
      continue;
    }
    
    if (equal(x, toCoordinate(nextMovement.cell.column)) &&
        equal(z, toCoordinate(nextMovement.cell.row))) {
      currentCell = nextMovement.cell;
      nextMovement = gradientDescent(currentCell, matrix);
      printf("current: [%d][%d], nextRow: %d, nextColumn: %d\n", 
      currentCell.row, currentCell.column,
      nextMovement.cell.row, nextMovement.cell.column);
    }
    else {
      nextMovement.function();
    }
  };

  // Limpeza do ambiente do webots
  wb_robot_cleanup();
  
  // Fim da funcao principal
  return 0;
}
