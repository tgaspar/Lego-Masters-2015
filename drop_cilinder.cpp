#include "ev3dev.h"
#include "particle.h"
#include "ev3pfilter.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cmath>

#ifndef NO_LINUX_HEADERS
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#define KEY_RELEASE 0
#define KEY_PRESS   1
#define KEY_REPEAT  2
#endif

#define TOUCH 0
#define GYRO 1
#define ULTRASOUND 2
#define COLOUR 3

#define DEGTORAD 3.14159265358979323846 / 180.0
#define RADTODEG 180.0 / 3.141592653589793
#define PI 3.14159265358979323846

#define SIZEX 18
#define SIZEY 10

using namespace std;
using namespace ev3dev;
  
  
ofstream sensorlog;


//***************************************************************
// MAPS VARIABLES
//***************************************************************
int bigColorMap[SIZEY*SIZEX] = 
{
//X->	00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17// Y
	4, 1, 3, 4, 3, 2, 4, 3, 1, 5, 1, 3, 5, 2, 1, 4, 1, 3,// 00
	0, 3, 1, 1, 3, 0, 2, 2, 2, 0, 2, 2, 5, 2, 2, 2, 5, 1,// 01
	4, 1, 3, 0, 3, 1, 3, 5, 4, 2, 2, 3, 2, 2, 2, 0, 1, 0,// 02
	1, 2, 2, 3, 5, 3, 1, 3, 2, 0, 3, 5, 1, 0, 5, 4, 0, 2,// 03
	4, 0, 4, 3, 2, 3, 2, 5, 4, 0, 5, 3, 2, 4, 4, 0, 0, 4,// 04
	0, 3, 1, 3, 5, 2, 0, 1, 4, 5, 4, 4, 4, 3, 3, 3, 2, 2,// 05
	1, 3, 1, 5, 5, 4, 2, 3, 5, 1, 0, 3, 1, 5, 1, 3, 2, 0,// 06
	5, 4, 2, 0, 5, 3, 3, 1, 2, 4, 2, 3, 5, 4, 1, 1, 0, 4,// 07
	5, 3, 2, 0, 2, 2, 3, 4, 2, 0, 1, 1, 2, 4, 4, 0, 5, 1,// 08
	1, 2, 5, 1, 5, 0, 5, 0, 4, 4, 1, 4, 4, 0, 2, 2, 0, 2 // 09
};

int numColorsRead = 0;
int readColors[SIZEY*SIZEX] = {0};
int matchesSum = 0;
int matchesArray[4] = {0};
int orientationMultiplier = 0;

//***************************************************************  

void display_map(){
cout << "++++++++++++++++++++++++++++++++++++++++\n";
	for(int y = 0; y < SIZEY; y++)
	{
		for(int x = 0; x < SIZEX; x++)
		{
			cout << bigColorMap[(y*SIZEX) + x] << " ";
		}
	cout << "\n";
	}
cout << "++++++++++++++++++++++++++++++++++++++++\n";
}



void battery_status()
{
	char c = 0;

	cout << endl << "*** Battery status ***" << endl;

	cout << endl << "Voltage is " << power_supply::battery.voltage_volts() << " Volt" << endl << endl;
	cout << endl << "Current is " << power_supply::battery.current_amps() << " A" << endl << endl;

}



void calculate_move_to_point(int x_cor, int y_cor)
{

}

  
class control
{
public:
	struct robot_info {
		int X;
		int Y;
		int angle;
	};	

public:
	control();
	~control();

	robot_info return_robot_info();
	
	int return_sensor_value(int sensor_type);
	int return_color_value();
	int print_rgb_values(int print);
	void compare_read_colors();

	void drive(int speed, int time=0);
	void drive_ultrasonic(int drive_distance);
	void drive_speed(int transSpeed, int rotSpeed);
	void turn_gyro(int turn_angle);
	void open_and_close(int angle);

	void stop();
	void reset();

	bool initialized() const;
	bool initialize();


	void terminate_on_key();
	void panic_if_touched();

	void drive_autonomously();
	
	//void drive_straight();


	void terminate() { _terminate = true; }



	robot_info robot_coordinates = {0};
	

private: 
	void correct_angle();


 
protected:
	large_motor		_motor_left;
	large_motor    		_motor_right;
	medium_motor		_motor_dropper;
	infrared_sensor 	_sensor_ir;
	touch_sensor   	 	_sensor_touch;
	ultrasonic_sensor	_sensor_ultrasonic;
	gyro_sensor	 	_sensor_gyro;
	color_sensor		_sensor_color;
	lcd			_lcd;

enum state
	{
	state_idle,
	state_driving,
	state_turning
	};

	state _state;
	bool  _terminate;
};

control::control() :
  _motor_left(OUTPUT_B),
  _motor_right(OUTPUT_C),
  _motor_dropper(OUTPUT_D),
  _state(state_idle),
  _terminate(false)
{
}

control::~control()
{
  reset();
}

control::robot_info control::return_robot_info()
{
	return robot_coordinates;
}

int control::return_sensor_value(int sensor_type)
{

	switch(sensor_type){
		case 0:
			return _sensor_touch.value();
		case 1:
			return  _sensor_gyro.value();
		case 2:
			return _sensor_ultrasonic.value();
	}
}
int control::return_color_value()
{
	double red = _sensor_color.value(0)/255.0;
	double green = _sensor_color.value(1)/255.0;
	double blue = _sensor_color.value(2)/255.0;
	int color = 0;

	if((red > 0.5) && (green > 0.5) && (blue > 0.5))
		color = 5;
	else if((red < 0.1) && (green < 0.1) && (blue < 0.1))
		color = 0;
	else if((red + green > 2*blue) && (red + green > 1.5) && (blue < 0.1))
		color = 4;
	else if((red < 0.3) && (green < 0.3) && (blue < 0.3))
		color = 3;
	else if((red > green) && (red > blue))
		color = 1;
	else if((green > red) && (green > blue)) 
		color = 2;
	else
		color = -1;

	return color;
}

int control::print_rgb_values(int print)
{
	double red = _sensor_color.value(0)/255.0;
	double green = _sensor_color.value(1)/255.0;
	double blue = _sensor_color.value(2)/255.0;
	int color = 0;


// 	else if((blue > red) && (blue > green))
	if((red > 0.5) && (green > 0.5) && (blue > 0.5))
		color = 5;
	else if((red < 0.1) && (green < 0.1) && (blue < 0.1))
		color = 0;
	else if((red + green > 2*blue) && (red + green > 1.5) && (blue < 0.1))
		color = 4;
	else if((red < 0.3) && (green < 0.3) && (blue < 0.3))
		color = 3;
	else if((red > green) && (red > blue))
		color = 1;
	else if((green > red) && (green > blue)) 
		color = 2;
	else
		color = -1;

	if(print)
	{
		cout << "_sensor_color.value(0) = " << red << "\n";
		cout << "_sensor_color.value(1) = " << green << "\n";
		cout << "_sensor_color.value(2) = " << blue << "\n";
		
		switch(color)
		{
			case 0:
				cout << "Colour is BLACK\n";
				break;
			case 1:
				cout << "Colour is RED\n";
				break;
			case 2:
				cout << "Colour is GREEN\n";
				break;
			case 3:
				cout << "Colour is BLUE\n";
				break;
			case 4:
				cout << "Colour is YELLOW\n";
				break;
			case 5:
				cout << "Colour is WHITE\n";
				break;
			default:
				cout << "Nothing \n";
		}
	}
	
	return color;
	
}
void control::compare_read_colors()
{
	int colorsSum = -1;
	int numberOfMatches0 = 0;
	int numberOfMatches1 = 0;
	int numberOfMatches2 = 0;
	int numberOfMatches3 = 0;
	
	
	for(int y = 0; y < (SIZEY); y++)
	{
		for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
		{
			int u = 0;
			colorsSum = 0;
			for(int i = 0; i< numColorsRead; i++)
			{
				colorsSum += abs(readColors[i] - bigColorMap[(y*SIZEX) + x + u]);
				u++;
			}	
			if(colorsSum == 0)
				numberOfMatches0++;
		}
	}
	
	for(int y = 0; y < (SIZEY); y++)
	{
		for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
		{
			colorsSum = 0;
			int u = 0;
			for(int i = 0; i< numColorsRead; i++)
			{	
				colorsSum += abs(readColors[numColorsRead - (i+1)] - bigColorMap[(y*SIZEX) + x + u]);
				u++;
			}	
			if(colorsSum == 0)
				numberOfMatches1++;
		}
	}
	
	for(int y = 0; y < (SIZEY); y++)
	{
		for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
		{
			colorsSum = 0;
			int u = 0;
			colorsSum = 0;
			for(int i = 0; i< numColorsRead; i++)
			{
				colorsSum += abs(readColors[i] - bigColorMap[(y*SIZEX) + x + u*SIZEX]);
				u++;
			}	
			if(colorsSum == 0)
				numberOfMatches2++;
		}
	}
	
	for(int y = 0; y < (SIZEY); y++)
	{
		for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
		{
			colorsSum = 0;
			int u = 0;
			colorsSum = 0;
			for(int i = 0; i< numColorsRead; i++)
			{
				colorsSum += abs(readColors[numColorsRead - (i+1)] - bigColorMap[(y*SIZEX) + x + u*SIZEX]);
				u++;
			}	
			if(colorsSum == 0)
				numberOfMatches3++;
		}
	}
	matchesArray[0] = numberOfMatches0;
	matchesArray[1] = numberOfMatches1;
	matchesArray[2] = numberOfMatches2;
	matchesArray[3] = numberOfMatches3;

	matchesSum = numberOfMatches0 + numberOfMatches1 + numberOfMatches2 + numberOfMatches3;
	
	if(matchesSum == 1)
	{
		cout << "The requirements for localization are met !! \n";
		for (int i = 0; i < 4; i++)
		{
			cout << "matchesArray[" << i << "]" << matchesArray[i] << "\n\n";
			if(matchesArray[i] == 1)
			{
				switch(i)
				{
					case 0:
						orientationMultiplier = 0;
						for(int y = 0; y < (SIZEY); y++)
						{
							for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
							{
								int u = 0;
								colorsSum = 0;
								for(int i = 0; i< numColorsRead; i++)
								{
									colorsSum += abs(readColors[i] - bigColorMap[(y*SIZEX) + x + u]);
									u++;
								}	
								if(colorsSum == 0)
								{	
										robot_coordinates.X = (x + numColorsRead) - 1;
										robot_coordinates.Y = y;
										robot_coordinates.angle = 0;
								}
							}
						}
						break;
					case 1:
						orientationMultiplier = 2;
						for(int y = 0; y < (SIZEY); y++)
						{
							for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
							{
								colorsSum = 0;
								int u = 0;
								for(int i = 0; i< numColorsRead; i++)
								{	
									colorsSum += abs(readColors[numColorsRead - (i+1)] - bigColorMap[(y*SIZEX) + x + u]);
									u++;
								}	
								if(colorsSum == 0)
								{
									robot_coordinates.X = x;
									robot_coordinates.Y = y;
									robot_coordinates.angle = 180;
								}
							}
						}
						break;
					case 2:
						orientationMultiplier = 1;
						for(int y = 0; y < (SIZEY); y++)
						{
							for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
							{
								colorsSum = 0;
								int u = 0;
								colorsSum = 0;
								for(int i = 0; i< numColorsRead; i++)
								{
									colorsSum += abs(readColors[i] - bigColorMap[(y*SIZEX) + x + u*SIZEX]);
									u++;
								}	
								if(colorsSum == 0)
								{
									robot_coordinates.X = x;
									robot_coordinates.Y = y + numColorsRead;
									robot_coordinates.angle = 90;
								}
							}
						}
						break;
					case 3:
						orientationMultiplier = 3;
						for(int y = 0; y < (SIZEY); y++)
						{
							for(int x = 0; x < (SIZEX - numColorsRead + 1); x++)
							{
								colorsSum = 0;
								int u = 0;
								colorsSum = 0;
								for(int i = 0; i< numColorsRead; i++)
								{
									colorsSum += abs(readColors[numColorsRead - (i+1)] - bigColorMap[(y*SIZEX) + x + u*SIZEX]);
									u++;
								}	
								if(colorsSum == 0)
								{
									robot_coordinates.X = x;
									robot_coordinates.Y = y;
									robot_coordinates.angle = 270;
								}
							}
						}
						break;
					default:
						orientationMultiplier = -1;
						break;
				}
			robot_coordinates.X = robot_coordinates.X*100;
			robot_coordinates.Y = robot_coordinates.Y*100;
			break;
			}
		}
	}
}


void control::drive(int speed, int time)
{
	if (time > 0)
	{
		_motor_left .set_run_mode(motor::run_mode_time);
		_motor_right.set_run_mode(motor::run_mode_time);

		_motor_left .set_time_setpoint(time);
		_motor_right.set_time_setpoint(time);
	}
	else
	{
		_motor_left .set_run_mode(motor::run_mode_forever);
		_motor_right.set_run_mode(motor::run_mode_forever);
	}

	_motor_left.set_duty_cycle_setpoint(-speed);

	_motor_right.set_duty_cycle_setpoint(-speed);

	_state = state_driving;

	_motor_left .run();
	_motor_right.run();

	if (time > 0)
	{
		while (_motor_left.running() || _motor_right.running())
			this_thread::sleep_for(chrono::milliseconds(10));
		_state = state_idle;
	}
}

void control::drive_ultrasonic(int drive_distance)
{
	if (_state != state_idle)
		stop();
	
	if (drive_distance < 10)
		return;
	
	int distance_difference = 0;
	int start_distance = 0;
	int final_difference = 0;
	double wheel_turning_speed_left = 0;
	double wheel_turning_speed_right = 0;
	
	int turn_degrees = 0;
	
// 	double wheel_length = 345.4;
	double wheel_length = 2*M_PI*54.9;
	
	turn_degrees = int(round(2*(drive_distance/wheel_length)*360.0));
	
	_sensor_gyro.set_mode(gyro_sensor::mode_speed);

	_motor_right.reset();
	_motor_left.reset();
	
	_motor_left.set_run_mode(motor::run_mode_position);
 	_motor_right.set_run_mode(motor::run_mode_position);
	
	_motor_left.set_stop_mode(motor::stop_mode_brake);
	_motor_right.set_stop_mode(motor::stop_mode_brake);
		
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.set_position_mode(motor::position_mode_absolute);
	_motor_right.set_position_mode(motor::position_mode_absolute);
	
	_motor_left.set_position_setpoint(_motor_left.position() + turn_degrees);
	
	_motor_right.set_position_setpoint(_motor_right.position() + turn_degrees);
	
 	_motor_left.set_pulses_per_second_setpoint(0);
 	_motor_right.set_pulses_per_second_setpoint(0);
	
// 	_motor_left.set_ramp_up(1000);
// 	_motor_right.set_ramp_up(1000);
// 	
// 	
// 	
// 	_motor_left.set_ramp_down(2000);
// 	_motor_right.set_ramp_down(2000);
// 	
// 	cout << "_motor_left.ramp_up_sp()  = " << _motor_left.position() << "\n";
// 	cout << "_motor_right.ramp_up_sp()  = " << _motor_right.position() << "\n";
// 	
	_motor_left.run();
	_motor_right.run();

	
	_state = state_turning;

	double gyroCorrection = 0;
	double gyroGain = 0;
	
	while (_motor_left.running() || _motor_right.running())
	{

		//wheel_turning_speed_left = (turn_degrees - _motor_left.position());
		//wheel_turning_speed_right = (turn_degrees - _motor_right.position());

		if (wheel_turning_speed_left < (turn_degrees - _motor_left.position()))
		{
			wheel_turning_speed_left += 0.5;
		}
		else
		{
			wheel_turning_speed_left -= 1;
		}
		
		if (wheel_turning_speed_right < (turn_degrees - _motor_right.position()))
		{
			wheel_turning_speed_right += 0.501;
		}
		else
		{
			wheel_turning_speed_right -= 1;
		}


		if(wheel_turning_speed_left < 60)
			wheel_turning_speed_left = 60;
		
		if(wheel_turning_speed_right < 60)
			wheel_turning_speed_right = 60;

		if(wheel_turning_speed_left > 750)
			wheel_turning_speed_left = 750;
		
		if(wheel_turning_speed_right > 750)
			wheel_turning_speed_right = 750;
		
		
		//gyroCorrection = gyroCorrection + (_sensor_gyro.value()*gyroGain)*(wheel_turning_speed_left*1.0)/800;

		gyroCorrection = (_sensor_gyro.value()*gyroGain);
		
// 		cout << "gyroCorrection = " << gyroCorrection << "\n";
		_motor_left.set_pulses_per_second_setpoint(wheel_turning_speed_left - gyroCorrection);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed_right + gyroCorrection);	


/*
		cout << "gyroCorrection = " << gyroCorrection << "\n";
		cout << "_sensor_gyro.value() = " << _sensor_gyro.value()<< "\n";
		cout << "_motor_right.pulses_per_second_setpoint = " << _motor_right.pulses_per_second_setpoint() << "\n";
		cout << "_motor_left.pulses_per_second_setpoint = " << _motor_left.pulses_per_second_setpoint() << "\n";		
*/	
	}

	robot_coordinates.X = robot_coordinates.X + cos(robot_coordinates.angle*DEGTORAD)*drive_distance;
	robot_coordinates.Y = robot_coordinates.Y + sin(robot_coordinates.angle*DEGTORAD)*drive_distance;
	
	_state = state_idle;
}
void control::drive_speed(int transSpeed, int rotSpeed)
{
	int distance_difference = 0;
	int start_distance = 0;
	int final_difference = 0;
	double wheel_turning_speed_left = 0;
	double wheel_turning_speed_right = 0;
	
	int turn_degrees = 0;
	
// 	double wheel_length = 345.4;
	double wheel_length = 2*M_PI*54.9;
	double m_to_degrees = 360/wheel_length;
	
	_sensor_gyro.set_mode(gyro_sensor::mode_speed);

	_motor_left.set_run_mode(motor::run_mode_forever);
	_motor_right.set_run_mode(motor::run_mode_forever);
	
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.set_stop_mode(motor::stop_mode_brake);
	_motor_right.set_stop_mode(motor::stop_mode_brake);
	
	_motor_left.set_pulses_per_second_setpoint((int)transSpeed*m_to_degrees*2);
	_motor_right.set_pulses_per_second_setpoint((int)transSpeed*m_to_degrees*2);
	
	_motor_left.run();
	_motor_right.run();
	
}


void control::turn_gyro(int turn_angle)
{
	if (_state != state_idle)
		stop();

	if (turn_angle	 == 0)
		return;

	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_gyro.set_mode(gyro_sensor::mode_speed);
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);

	int angle_difference = 0;
	int start_angle = _sensor_gyro.value();
	
	int distance_difference = 0;
	int start_distance = 0;
	int final_difference = 0;
	double wheel_turning_speed_left = 0;
	double wheel_turning_speed_right = 0;
	
	int turn_degrees = 0;
	int turn_radius = 55*turn_angle*DEGTORAD;
	
// 	double wheel_length = 345.4;
	double wheel_length = 2*M_PI*54.9;
	
	turn_degrees = int(round(2*(turn_radius/wheel_length)*360.0));

	_motor_right.reset();
	_motor_left.reset();
	
	_motor_left.set_run_mode(motor::run_mode_forever);
	_motor_right.set_run_mode(motor::run_mode_forever);
	
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.run();
	_motor_right.run();
/*	
	_motor_right.reset();
	_motor_left.reset();
	
	_motor_left.set_run_mode(motor::run_mode_position);
 	_motor_right.set_run_mode(motor::run_mode_position);
	
	_motor_left.set_stop_mode(motor::stop_mode_brake);
	_motor_right.set_stop_mode(motor::stop_mode_brake);
		
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.set_position_mode(motor::position_mode_absolute);
	_motor_right.set_position_mode(motor::position_mode_absolute);
	
	_motor_left.set_position_setpoint(_motor_left.position() + turn_degrees);
	
	_motor_right.set_position_setpoint(_motor_right.position() - turn_degrees);
	
 	_motor_left.set_pulses_per_second_setpoint(60);
 	_motor_right.set_pulses_per_second_setpoint(60);
	
	
	cout << "_motor_left.position() = " <<_motor_left.position_setpoint() << "\n";
	cout << "_motor_right.position() = " <<_motor_right.position_setpoint() << "\n";
	
	_motor_left.run();
	_motor_right.run();
*/
	
	_state = state_turning;
	while (_motor_left.running() || _motor_right.running())
	{

		angle_difference  = start_angle - _sensor_gyro.value();
			
		int wheel_turning_speed = int(turn_angle - angle_difference)*5;
		
		if(wheel_turning_speed  > 800)
			wheel_turning_speed = 800;
		if(wheel_turning_speed < 60)
			wheel_turning_speed = 60;
		
		_motor_left.set_pulses_per_second_setpoint(-wheel_turning_speed);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed);

		
		if(angle_difference > turn_angle)
		{
				_motor_left.stop();
				_motor_right.stop();
				break;
		}
/*
		if (wheel_turning_speed_left < (turn_degrees - _motor_left.position()))
		{
			wheel_turning_speed_left += 0.5;
		}
		else
		{
			wheel_turning_speed_left -= 0.5;
		}
		
		if (wheel_turning_speed_right < (turn_degrees - _motor_right.position()))
		{
			wheel_turning_speed_right += 0.5;
		}
		else
		{
			wheel_turning_speed_right -= 0.5;
		}
		
		cout << "_motor_left.position() = " << _motor_left.position() << "\n";
		cout << "_motor_right.position() = " << _motor_right.position() << "\n";

		if(wheel_turning_speed_left < 60)
			wheel_turning_speed_left = 60;
		
		if(wheel_turning_speed_right < 60)
			wheel_turning_speed_right = 60;

		if(wheel_turning_speed_left > 500)
			wheel_turning_speed_left = 500;
		
		if(wheel_turning_speed_right > 500)
			wheel_turning_speed_right = 500;
		
		_motor_left.set_pulses_per_second_setpoint(wheel_turning_speed_left);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed_right);	
		
*/
// 		cout << "_motor_right.pulses_per_second_setpoint = " << _motor_right.pulses_per_second_setpoint() << "\n";
// 		cout << "_motor_left.pulses_per_second_setpoint = " << _motor_left.pulses_per_second_setpoint() << "\n";		

	}

	robot_coordinates.angle = robot_coordinates.angle + turn_angle;
	control::correct_angle();
	_state = state_idle;
}

void control::open_and_close(int angle)
{

int small_back = angle/3;
int big_back = angle*2/3;


_motor_dropper.set_position_setpoint(-24);
_motor_dropper.run();
_state = state_turning;
while(_motor_dropper.running());
this_thread::sleep_for(chrono::milliseconds(1000));

_motor_dropper.set_position_setpoint(20);
_motor_dropper.run();
_state = state_turning;
while(_motor_dropper.running());
this_thread::sleep_for(chrono::milliseconds(1000));

_motor_dropper.set_position_setpoint(10);
_motor_dropper.run();
_state = state_turning;
while(_motor_dropper.running());
this_thread::sleep_for(chrono::milliseconds(1000));
_state = state_idle;
}


void control::stop()
{
  _motor_left .stop();
  _motor_right.stop();
  
  _state = state_idle;
}

void control::reset()
{
  if (_motor_left.connected())
    _motor_left .reset();
  
  if (_motor_right.connected())
    _motor_right.reset();
  
  _state = state_idle;
}

bool control::initialized() const
{

if (!_motor_left.connected())
{
	cout << "Left motor not connected\n";
}
else
{
	cout << "Left motor is connected\n";
}

if (!_motor_right.connected())
{
	cout << "Right motor not connected\n";
}
else
{
	cout << "Right motor is connected\n";
}

if (!_sensor_gyro.connected())
{
	cout << "Gyro sensor not connected\n";
}
else
{
	cout << "Gyro sensor is connected\n";

}

if (!_sensor_ultrasonic.connected())
{
	cout << "Ultrasonic sensor not connected\n";
}
else
{
	cout << "Ultrasonic sensor is connected\n";
}

  return (_motor_left.connected() &&
          _motor_right.connected() &&
	  _sensor_gyro.connected() &&
          _sensor_ultrasonic.connected());
}


bool control::initialize()
{
	//Initialize left motor
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_left.set_run_mode(motor::run_mode_forever);
	
	//Initialize right motor
	_motor_right.set_regulation_mode(motor::mode_on);
	_motor_right.set_run_mode(motor::run_mode_forever);
	
	
	//Initialize small motor
	_motor_dropper.set_position_setpoint(0);
	_motor_dropper.set_pulses_per_second_setpoint(500);
	_motor_dropper.set_regulation_mode(motor::mode_on);
	_motor_dropper.set_run_mode(motor::run_mode_position);
		
	//Initialize sensors
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	_sensor_color.set_mode(color_sensor::mode_rgb);

	cout << "Initialization done! \n";
}


void control::terminate_on_key()
{
 #ifndef NO_LINUX_HEADERS
  thread t([&] () {
    int fd = open("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY);
    if (fd  < 0)
    {
      cout << "Couldn't open platform-gpio-keys device!" << endl;
      return;
    }

    input_event ev;
    while (true)
    {
      size_t rb = read(fd, &ev, sizeof(ev));

      if (rb < sizeof(input_event))
        continue;

      if ((ev.type == EV_KEY) /*&& (ev.value == KEY_PRESS)*/)
      {
        terminate();
        return;
      }
    }
  });
  t.detach();
 #endif
}

void control::panic_if_touched()
{
  if (!_sensor_touch.connected())
  {
    cout << "no touch sensor found!" << endl;
    return;
  }
  
  thread t([&] () {
    while (!_terminate) {
      if (_sensor_touch.value())
      {
        reset();
        break;
      }
      this_thread::sleep_for(chrono::milliseconds(100));
    }
  });
  t.detach();
}


void control::drive_autonomously()
{
  if (!_sensor_ultrasonic.connected())
  {
    cout << "no ultrasonic found!" << endl;
    return;
  }
  
  _sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
  _sensor_gyro.set_mode(gyro_sensor::mode_speed);

  while (!_terminate)
  {
    int distance = _sensor_ultrasonic.value();
	int angle = _sensor_gyro.value();
	printf("Gyro sensor value: %d \n",angle);
	int driveVal = 0;
    
		while(true)
		{
		driveVal = -_sensor_gyro.value();
        drive(driveVal);
		}
	}
}

void control::correct_angle()
{
	int corrected_angle = control::robot_coordinates.angle;

	while(corrected_angle > 360)
		corrected_angle = corrected_angle - 360;

	while(corrected_angle < 0)
		corrected_angle = corrected_angle + 360;

	control::robot_coordinates.angle = corrected_angle;
}

void printRobotStatus(control lego_robot)
{
	cout << "lego_robot.robot_coordinates.angle = " << lego_robot.robot_coordinates.angle << "\n";
	cout << "lego_robot.robot_coordinates.X = " << lego_robot.robot_coordinates.X << "\n";
	cout << "lego_robot.robot_coordinates.Y = " << lego_robot.robot_coordinates.Y << "\n";	

}

int main()
{
	int modeSelect = 0;
	int robotPossitionArray[3] = {0};
	std::clock_t start;
	double timePassed;

    start = std::clock();
	
	battery_status();
	cout << "Please select a mode. \n";
	cout << "Possible modes are: \n";
	cout << "Drop the cilinder -> 1 \n";
	cout << "Drive around -> 2 \n";
	cout << "Color reading -> 3 \n";
	cout << "Drive around and read colors -> 4 \n";
	cout << "Localization test -> 5 \n";
	cout << "Localization with particle filter-> 6 \n";
	cin >> modeSelect;

	printf("Selected mode is %d", modeSelect);

	control lego_robot;

	cout << "Initializing \n";

	lego_robot.initialize();
	lego_robot.initialized();

	cout << "Press the touch sensor to start \n";

	//while (!lego_robot.return_sensor_value(TOUCH));

	cout << "Touch sensor pressed !! \n";

	switch (modeSelect)
	{
	case 1:	
		for(int i = 0; i <3; i++)
		{
			lego_robot.open_and_close(60);
			this_thread::sleep_for(chrono::milliseconds(500));
		}
		break;
	case 2:
		this_thread::sleep_for(chrono::milliseconds(1000));
		lego_robot.drive_speed(100,0);
		break;
	case 3:	
		while (lego_robot.return_sensor_value(TOUCH));			
		cout << "Press button to read the BLACK color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		cout << "Press button to read the RED color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		cout << "Press button to read the GREEN color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		cout << "Press button to read the BLUE color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		cout << "Press button to read the YELLOW color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		cout << "Press button to read the WHITE color sensor values \n";
		while (!lego_robot.return_sensor_value(TOUCH));
		while (lego_robot.return_sensor_value(TOUCH));
		lego_robot.print_rgb_values(1);
		break;
	case 4:
		this_thread::sleep_for(chrono::milliseconds(1000));
		lego_robot.drive_ultrasonic(100);
		lego_robot.print_rgb_values(1);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.turn_gyro(90);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.drive_ultrasonic(100);
		lego_robot.print_rgb_values(1);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.turn_gyro(90);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.drive_ultrasonic(100);
		lego_robot.print_rgb_values(1);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.turn_gyro(90);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.drive_ultrasonic(100);
		lego_robot.print_rgb_values(1);
		this_thread::sleep_for(chrono::milliseconds(500));
		lego_robot.turn_gyro(90);
		printRobotStatus(lego_robot);
		break;
	case 5:
		while (lego_robot.return_sensor_value(TOUCH));
		this_thread::sleep_for(chrono::milliseconds(500));
		while(matchesSum != 1)
		{
			readColors[numColorsRead] = lego_robot.print_rgb_values(0);
			numColorsRead += 1;
			lego_robot.compare_read_colors();
			cout << "numColorsRead = " << numColorsRead << "\n";
			cout << "******************************** \n\n";
			if(matchesSum != 1)
			{
				cout << "Driving ... \n";
				lego_robot.drive_ultrasonic(100);	
			}
		}
		printRobotStatus(lego_robot);
		break;
	case 6:
	{
		robotPossitionArray[0] = lego_robot.robot_coordinates.X;
		robotPossitionArray[1] = lego_robot.robot_coordinates.Y;
		robotPossitionArray[2] = lego_robot.robot_coordinates.angle;
		int robotMoves[3] = {0};
		int robotMoveCommands[2] = {40, 0};
		int traveledDistance = 0;
		
// 		for(int i = 0; i < 30; i++)
		while (!lego_robot.return_sensor_value(TOUCH))
		{
			
			//while (!lego_robot.return_sensor_value(TOUCH));
			//while (lego_robot.return_sensor_value(TOUCH));
			
			robotMoveCommands[0] = 70;
			robotMoveCommands[1] = 0;
			
			lego_robot.drive_speed(robotMoveCommands[0],0);
			/*			
			lego_robot.drive_ultrasonic(robotMoveCommands[0]);
			lego_robot.turn_gyro(robotMoveCommands[1]);
			*/		
			robotMoveCommands[0] = (int)robotMoveCommands[0]*1.0*timePassed;
			robotMoves[2] = robotMoveCommands[1];
			robotMoves[0] = int_round(robotMoveCommands[0]*cos(robotMoveCommands[1]*1.0));
			robotMoves[1] = int_round(robotMoveCommands[0]*sin(robotMoveCommands[1]*1.0));
			
			particleFilterMain(&robotPossitionArray[0], &robotMoves[0], &robotMoveCommands[0], lego_robot.print_rgb_values(1));
					
			cout << "robotPossitionArray[0] = "<< robotPossitionArray[0] << "\n";
			cout << "robotPossitionArray[1] = "<< robotPossitionArray[1] << "\n";
			cout << "robotPossitionArray[2] = "<< robotPossitionArray[2] << "\n";
			
			
			traveledDistance  += 2*robotMoveCommands[0];
			
			timePassed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
			start = std::clock();
// 			robotPossitionArray[0] += 10;
		}
		cout << "traveledDistance[0] = " << traveledDistance << "\n";
		break;
	}
	default:
		cout << "Wrong mode selected !!";
	}
	
	

	return 0;
}

