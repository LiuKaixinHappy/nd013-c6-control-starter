/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
    Kp_ = Kpi;
    Ki_ = Kii;
    Kd_ = Kdi;
    output_lim_max_ = output_lim_maxi;
    output_lim_min_ = output_lim_mini;
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    d_error_ = delta_time_ > 0 ? (cte - p_error_) / delta_time_ : 0;
    p_error_ = cte;
    i_error_ += cte * delta_time_;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
    if (control > output_lim_max_) {
        control = output_lim_max_;
    } else if (control < output_lim_min_) {
        control = output_lim_min_;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    double old_delta_time = delta_time_;
    delta_time_ = new_delta_time;
    return old_delta_time;
}