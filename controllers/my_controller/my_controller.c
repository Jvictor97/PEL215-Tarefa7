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


/* Structs para auxiliar no desenvolvimento */

// Struct Ponto
typedef struct {
 double x;
 double z;
} Point;

// Struct para Celula
typedef struct {
 int row;
 int column;
} Cell;

// Struct para o Movimento do robo
typedef struct {
 Cell cell;
 Function function;
 double speed;
} Movement;

// Definindo um tipo "Function" (ponteiro para funcao)
typedef void (*Function)();

// Ponto de objetivo do robo na celula [0][19]
Point goal = { 7.125, -7.125 };

// Distancia de influencia dos obstáculos
double Ro0 = 3.0;

// Array para referenciar os motores
WbDeviceTag motors[4];

// Constante Katt
const double katt = 1.0;
// Constante Krep
const double krep = 999.9;
// Fator multiplicador da velocidade
double cellSpeed = 0.0;

// Funcao para garantir que um valor esteja no intervalo [min,max]
double onInterval(double value, double min, double max) {
  if (value < min) return min;
  if (value > max) return max;
  
  return value;
}

/* Funcoes para configurar os motores */

// Funcao para configurar a velocidade nos 4 motores
void setSpeed(double* speeds) {
  for (int i = 0; i < 4; i++) {
    double cellSpeedFactor = onInterval(cellSpeed, 1, 6.28);
    wb_motor_set_velocity(motors[i], speeds[i] / 6.28 * cellSpeedFactor);
  }
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

// Calcula a distancia euclidiana entre dois pontos
double euclidianDistance(Point p1, Point p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.z - p2.z, 2));
}

// Calcula o potencial de uma celula
double potential(double row, double column) {
  // Coordenadas dos 7 obstaculos no ambiente
  Point obstacles[7] = {
   { -4.875, -3.375 },
   { -0.375, -5.625 },
   {  4.875, -3.375 },
   { -0.375, -0.375 },
   {  4.125,  1.125 },
   { -3.375,  3.375 },
   {  1.875,  4.875 }
  };
  
  // Convertendo linha e coluna em coordenadas
  double x = 0.75 * column - 7.125;
  double z = 0.75 * row - 7.125;
  
  // Cria a representação da celula
  Point cellPosition = { x, z };
  
  // Calcula a distancia da celula atual para o objetivo
  double goalDistance = euclidianDistance(cellPosition, goal);
  
  // Calcula o potencial atrativo
  double Uatt = 0.5 * katt * pow(goalDistance, 2);
  
  // Variavel para o potencial repulsivo
  double Urep = 0.0;
  
  // Calcula o potencial repulsivo 
  //(somatorio do potencial repulsivo de todos os obstaculos cujo
  // Roq <= Ro0)
  for(int i = 0; i < 7; i++) {
    // Calcula a distancia da celula para o obstaculo atual
    double Roq = euclidianDistance(cellPosition, obstacles[i]);
    
    // Se a distancia atual for superior a distancia de influencia
    // passa para o proximo obstaculo
    if (Roq > Ro0) continue;
  
    // Caso contrario, soma o potencial de repulsao da celula
    Urep += 0.5 * krep * pow((1/Roq - 1/Ro0), 2);
  }
  
  // Retorna o potencial com a soma de Uatt e Urep
  return Uatt + Urep;
}

// Calcula o potencial de todas as celulas da matriz
void calculateMatrixPotentials(double matrix[][20]) {
  for(int row = 0; row < 20; row++) {
    for(int column = 0; column < 20; column++) {
      matrix[row][column] = potential(row, column);
    }
  }
}

// Verifica se uma celula eh valida
bool validCell(int row, int column) {
  return row >= 0 && row <= 19 && column >= 0 && column <= 19;
}

// Define qual funcao de movimentacao deve ser usada
// comparando a celula atual com a proxima
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

// Funcao para o Gradiente Descendente, define o proximo movimento do robo
Movement gradientDescent(Cell currentCell, double matrix[20][20]) {
  // Potencial da celula atual
  double currentCellPotential = matrix[currentCell.row][currentCell.column];
  // Potencial minimo
  double minimumPotential = currentCellPotential;
  // Velocidade do movimento
  double speed = 0.0;
  // Proxima celula
  Cell nextCell = currentCell;
  // Struct movimento
  Movement movement;

  // Examina o melhor movimento dentre todos os disponiveis
  for (int row = currentCell.row - 1; row <= currentCell.row + 1; row++) {
    for (int column = currentCell.column - 1; column <= currentCell.column + 1; column++) {
      if (!validCell(row, column)) continue;
      double cellPotential = matrix[row][column];
      
      // Caso potencial da celula avaliada seja menor que o menor potencial
      if (cellPotential < minimumPotential) {
        // Atualiza a velocidade do movimento
        speed = currentCellPotential - cellPotential;
        // Atualiza o menor potencial
        minimumPotential = cellPotential;
        // Define a proxima celula como a celula avaliada
        nextCell = (Cell) { row, column };
      }
    }
  }
  
  // Associa as informacoes do movimento e o retorna
  movement.speed = speed;
  movement.cell = nextCell;
  movement.function = defineFunction(currentCell, nextCell);
  return movement;
}

// Converte um indice (de linha ou coluna) em coordenadas
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
  
  // Matriz para representar o ambiente  
  double matrix[20][20];
  // Chamada para calcular os potenciais das celulas
  calculateMatrixPotentials(matrix);
    
  // Celula atual
  Cell currentCell = { 19, 0 };

  // Define o primeiro movimento
  Movement nextMovement = gradientDescent(currentCell, matrix);
      
  cellSpeed = nextMovement.speed;
  // Loop principal
  while (wb_robot_step(TIME_STEP) != -1) {      
    double x, z;
    // //Lendo a posicao do robo no ambiente
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    // //Atribuindo valores da posicao em X e Z
    x = values[0];
    z = values[2];
    
    // Condicao de parada do robo (chegou ao objetivo)
    if (equal(x, goal.x) && equal(z, goal.z)) {
      for (int i = 0; i < 4; i++)
        wb_motor_set_velocity(motors[i], 0);
      continue;
    }
    
    // Vefica se o robo esta na posicao correta
    if (equal(x, toCoordinate(nextMovement.cell.column)) &&
        equal(z, toCoordinate(nextMovement.cell.row))) {
      // Se estiver, atualiza as informacoes da celula atual
      currentCell = nextMovement.cell;
      // E define o proximo movimento
      nextMovement = gradientDescent(currentCell, matrix);
      cellSpeed = nextMovement.speed;
    }
    else {
      // Caso nao esteja na posicao esperada, se movimenta ate ela
      nextMovement.function();
    }
  };

  // Limpeza do ambiente do webots
  wb_robot_cleanup();
  
  // Fim da funcao principal
  return 0;
}
