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

// Array para referenciar os motores
WbDeviceTag motors[4];

/* Funcoes para configurar os motores */

// Funcao para configurar a velocidade nos 4 motores
void setSpeed(double* speeds) {
  for (int i = 0; i < 4; i++)
    wb_motor_set_velocity(motors[i], speeds[i] / SPEED);
}

// Move o robo para frente
void front() {
  double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  setSpeed(speeds);
}

// Move o robo para tras
void back() {
  double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
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

// Move o robo na diagonal para frente e para a esquerda
void frontLeft() {
  double speeds[4] = {SPEED, DIAGONAL_ZERO, DIAGONAL_ZERO, SPEED};
  setSpeed(speeds);
}

// Move o robo na diagonal para tras e para a esquerda
void backLeft() {
  double speeds[4] = {-DIAGONAL_ZERO, -SPEED, -SPEED, -DIAGONAL_ZERO};
  setSpeed(speeds);  
}

// Move o robo na diagonal para tras e para a direita
void backRight() {
  double speeds[4] = {-SPEED, -DIAGONAL_ZERO, -DIAGONAL_ZERO, -SPEED};
  setSpeed(speeds);
}

// Move o robo na diagonal para frente e para a direita
void frontRight() {
  double speeds[4] = {DIAGONAL_ZERO, SPEED, SPEED, DIAGONAL_ZERO};
  setSpeed(speeds);
}

// Funcao para comparar dois valores double, os valores
// sao considerados iguais caso a diferenca absoluna 
// seja menor que 0.1
bool equal(double firstValue, double secondValue) {
  const double tolerance = 0.1;
  
  return fabs(firstValue - secondValue) < tolerance;
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
    
  // Loop principal
  while (wb_robot_step(TIME_STEP) != -1) {
    // Lendo a posicao do robo no ambiente
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    // Atribuindo valores da posicao em X e Z
    double x = values[0], z = values[2];
    
    /* 
     * Bloco if/else para definir o movimento do robo
     * foram definidas 8 (oito) "coordenadas-chave"
     * que indicam ao robo qual movimento deve ser realizado
     */
    
    // Se o robo estiver na origem
    frontLeft();
  };

  // Limpeza do ambiente do webots
  wb_robot_cleanup();
  
  // Fim da funcao principal
  return 0;
}
