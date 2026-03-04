#ifndef PIDCONTROLLERS_H
#define PIDCONTROLLERS_H

#include "MotorControl.h"
#include "InverseKinematics.h"
#include <Arduino.h>
#include <math.h>
#include "Screen.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

extern double kp, ki, kd; //Standard gains
extern double kp_adj, ki_adj, kd_adj; //Adjustive gains
// Constriants and Thresholds
extern double max_output, max_angle; 
extern double integ_limit;
extern double adj_threshold;

void pid_balance(double setpoint_x, double setpoint_y);
void move_to_point(double setpoint_x, double setpoint_y, unsigned long delay);
void move_line(double rx, double ry, double speed, int repeat = 0);
void move_ellipse(double rx, double ry, double speed, double repeat = 0);
void move_square(double side_length, double speed, int repeat = 0);
void move_figure8(double radius, double speed, int repeat = 0);
void move_spiral(double max_radius, double speed, int repeat = 0);
void move_star(double radius, double speed, int repeat = 0);
void move_heart(double size, double speed, int repeat = 0);
void muteAllSerialOutput();
void enableAllSerialOutput();


#endif