#include "ev3dev.h"
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

#define DEGTORAD 3.141592653589793 / 180.0
#define RADTODEG 180.0 / 3.141592653589793

using namespace std;
using namespace ev3dev;
  
  
ofstream		 sensorlog;
  
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
		int X = 0;
		int Y = 0;
		int angle = 0;
	};	

public:
	control();
	~control();

	robot_info return_robot_info();

	int return_sensor_value(int sensor_type);
	void drive(int speed, int time=0);
	void drive_ultrasonic(int drive_distance);
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



	robot_info robot_coordinates;

private: 
	void correct_angle();


 
protected:
	large_motor     _motor_left;
	large_motor     _motor_right;
	medium_motor	  _motor_dropper;
	infrared_sensor _sensor_ir;
	touch_sensor    _sensor_touch;
	ultrasonic_sensor	_sensor_ultrasonic;
	gyro_sensor	  _sensor_gyro;
	lcd			  _lcd;

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
	int wheel_turning_speed_left = 0;
	int wheel_turning_speed_right = 0;
	
	int turn_degrees = 0;
	
	double wheel_length = 345.4;
	
	turn_degrees = int(2*(drive_distance/wheel_length)*360);
	
	_sensor_gyro.set_mode(gyro_sensor::mode_speed);

	_motor_right.reset();
	_motor_left.reset();
	
	_motor_left.set_run_mode(motor::run_mode_position);
	_motor_right.set_run_mode(motor::run_mode_position);
	
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.set_position_mode(motor::position_mode_absolute);
	_motor_right.set_position_mode(motor::position_mode_absolute);
	
	_motor_left.set_position_setpoint(_motor_left.position() + turn_degrees);
	_motor_right.set_position_setpoint(_motor_right.position() + turn_degrees);
	
	_motor_left.set_pulses_per_second_setpoint(100);
	_motor_right.set_pulses_per_second_setpoint(100);
	
	_motor_left.run();
	_motor_right.run();

	_state = state_turning;
	/*
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	
	_motor_left.set_run_mode(motor::run_mode_forever);
	_motor_right.set_run_mode(motor::run_mode_forever);

	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);

	_motor_left.set_pulses_per_second_setpoint(0);
	_motor_right.set_pulses_per_second_setpoint(0);

	_state = state_turning;

	_motor_left.run();
	_motor_right.run();
	
	start_distance = _sensor_ultrasonic.value();
	
	final_difference = start_distance - drive_distance;
	cout << "_sensor_ultrasonic.value() = " << _sensor_ultrasonic.value() << "\n";
	cout << "final_difference =" << final_difference  << "\n";
	
	while (_motor_left.running() || _motor_right.running())
	{
		distance_difference = start_distance - _sensor_ultrasonic.value();
		
		wheel_turning_speed = (_sensor_ultrasonic.value() - final_difference)*5;
		
		_motor_left.set_pulses_per_second_setpoint(wheel_turning_speed);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed);	
		
		if (wheel_turning_speed < 50)
			wheel_turning_speed = 50;
		
		if(distance_difference >= drive_distance)
		{
			cout << "distance_difference = " << distance_difference << "\n";
			cout << "_sensor_ultrasonic.value() = " << _sensor_ultrasonic.value() << "\n";
			_motor_left.stop();
			_motor_right.stop();
			break;
	}
	*/
	
	double gyroCorrection = 0;
	double gyroGain = 0.1;
	
	while (_motor_left.running() || _motor_right.running())
	{
		wheel_turning_speed_left = (turn_degrees - _motor_left.position());
		wheel_turning_speed_right = (turn_degrees - _motor_right.position());

		if(wheel_turning_speed_left > 800)
			wheel_turning_speed_left = 800;
		
		if(wheel_turning_speed_right > 800)
			wheel_turning_speed_right = 800;
		
		if(wheel_turning_speed_left < 60)
			wheel_turning_speed_left = 60;
		
		if(wheel_turning_speed_right < 60)
			wheel_turning_speed_right = 60;

		gyroCorrection = gyroCorrection + (_sensor_gyro.value()*gyroGain)*(wheel_turning_speed_left*1.0)/800;

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

	_motor_right.reset();
	_motor_left.reset();
	
	_motor_left.set_run_mode(motor::run_mode_forever);
	_motor_right.set_run_mode(motor::run_mode_forever);
	
	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);
	
	_motor_left.run();
	_motor_right.run();

	
	_state = state_turning;
	while (_motor_left.running() || _motor_right.running())
	{
		angle_difference  = start_angle - _sensor_gyro.value();
			
		int wheel_turning_speed = int(turn_angle - angle_difference)*5;
		
		if(wheel_turning_speed  > 800)
			wheel_turning_speed = 800;
		if(wheel_turning_speed < 50)
			wheel_turning_speed = 50;
		
		_motor_left.set_pulses_per_second_setpoint(-wheel_turning_speed);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed);
	
		if(angle_difference > turn_angle)
		{
				_motor_left.stop();
				_motor_right.stop();
				break;
		}
	}

	robot_coordinates.angle = robot_coordinates.angle + turn_angle;
	control::correct_angle();
	_state = state_idle;
}

void control::open_and_close(int angle)
{


_motor_dropper.set_position_setpoint(angle);
_motor_dropper.run();
_state = state_turning;
while(_motor_dropper.running());

_motor_dropper.set_position_setpoint(-angle);
_motor_dropper.run();
_state = state_turning;
while(_motor_dropper.running());

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
	_motor_dropper.set_pulses_per_second_setpoint(900);
	_motor_dropper.set_regulation_mode(motor::mode_on);
	_motor_dropper.set_run_mode(motor::run_mode_position);
	
	cout << "Initialization done! \n";
	
	//Initialize sensors
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	
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

	cout << "corrected_angle = " << corrected_angle << "\n";
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


	battery_status();
	control lego_robot;

	cout << "Initializing \n";

	lego_robot.initialize();
	lego_robot.initialized();

//	lego_robot.robot_info legoRobotInfo;

	cout << "Press the touch sensor to start \n";



	while (!lego_robot.return_sensor_value(TOUCH));

	int mode = 3;

	cout << "Touch sensor pressed !! \n";

	printRobotStatus(lego_robot);

	//lego_robot.turn_gyro(90);		
	this_thread::sleep_for(chrono::milliseconds(1000));
	lego_robot.drive_ultrasonic(1000);
	printRobotStatus(lego_robot);
	this_thread::sleep_for(chrono::milliseconds(500));
	lego_robot.turn_gyro(180);

	//lego_robot.drive_ultrasonic(100);	
	//printRobotStatus(lego_robot);	

	//lego_robot.turn_gyro(270);
	//printRobotStatus(lego_robot);

	//lego_robot.stop();


	return 0;
}

