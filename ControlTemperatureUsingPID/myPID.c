#include"myPID.h"

#define KP (double)40
#define KI (double)0.5
#define KD (double)0.0
#define PWM_PERIOD (uint16_t)5000

static double error_integral = 0;
static double last_error;
static double error;
static uint16_t duty_cycle=0;

void PID_Controller(double setpoint, double actual)
{
    // Calculate error
    error = setpoint - actual;
    // Update last error
    last_error = (double)error;
    // Calculate integral
    error_integral += (error+last_error)/2.0;
		// limit the max value of error_integral
		if (error_integral > 100/KI) error_integral = 100/KI;
		if(error_integral<0) error_integral = 0;
    // Calculate derivative
    double error_derivative = (error - last_error)/2;

    // Calculate PID output
    double output_pid = KP*error + KI*error_integral + KD*error_derivative;
		if (output_pid > 100) output_pid = 100;
		if(output_pid<0) output_pid =0.0;
    // Convert output to PWM duty cycle
    duty_cycle = (uint16_t)((output_pid/100) * PWM_PERIOD);
    // Set PWM duty cycle
    setDutyCircle(duty_cycle);
}
