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

  
class control
{
public:
  control();
  ~control();
 
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
	
	cout << "turn_degrees = " << turn_degrees << "\n";	
	
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
	
	cout << "_motor_left.position() = " << _motor_left.position() << "\n";
	cout << "_motor_right.position() = " << _motor_right.position() << "\n";
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
		
		_motor_left.set_pulses_per_second_setpoint(wheel_turning_speed_left);
		_motor_right.set_pulses_per_second_setpoint(wheel_turning_speed_right);
	
	}
	
	_state = state_idle;
}

void control::turn_gyro(int turn_angle)
{
	if (_state != state_idle)
		stop();

	if (turn_angle	 == 0)
		return;
	int angle_difference = 0;
	int start_angle = _sensor_gyro.value();

	_sensor_gyro.set_mode(gyro_sensor::mode_angle);

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

int main()
{

	battery_status();
	control c;

	cout << "Initializing \n";

	c.initialize();
	c.initialized();

	cout << "Press the touch sensor to start \n";

	while (!c.return_sensor_value(TOUCH));

	int mode = 3;

	cout << "Touch sensor pressed !! \n";

	c.turn_gyro(90);
	
		
	c.drive_ultrasonic(500);
	
	c.turn_gyro(180);
	
	c.drive_ultrasonic(100);
	
	c.turn_gyro(270);
	
	c.stop();


	return 0;
}

