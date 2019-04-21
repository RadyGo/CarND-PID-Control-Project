#include "PID.h"
#include <iostream>
#include <cmath>
#include <limits>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */
using namespace std;
PID::PID() {
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // Previous cte.
  prev_cte = 0.0;

  // Counters.
  counter = 0;
  errorSum = 0.0;
  minError = numeric_limits<double>::max();
  maxError = numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  // Proportional error.
  p_error = cte;

  // Integral error.
  i_error += cte;

  // Diferential error.
  d_error = cte - prev_cte;
  prev_cte = cte;

  errorSum += cte;
  counter++;

  if ( cte > maxError ) {
    maxError = cte;
  }
  if ( cte < minError ) {
    minError = cte;
  }
}

double PID::TotalError() {
  //cout<< "p i d "<< Kp<< " " <<Ki << " " << Kd << " " <<endl;
  return p_error * Kp + i_error * Ki + d_error * Kd;
}




