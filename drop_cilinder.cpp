/*
 *Copyright (c) 2014 - Franz Detro
 *
 * Some real world test program for motor control
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "ev3dev.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

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
  void turn_gyro(int angle, int turn_speed);
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


void control::turn_gyro(int final_angle,int turn_speed)
{
	if (_state != state_idle)
		stop();

	if (final_angle == 0)
		return;
	int current_angle = _sensor_gyro.value();
	int start_angle = _sensor_gyro.value();

	_sensor_gyro.set_mode(gyro_sensor::mode_angle);

	_motor_left.set_run_mode(motor::run_mode_forever);
	_motor_right.set_run_mode(motor::run_mode_forever);

	_motor_left.set_regulation_mode(motor::mode_on);
	_motor_right.set_regulation_mode(motor::mode_on);


	_motor_left.set_pulses_per_second_setpoint(turn_speed);
	_motor_right.set_pulses_per_second_setpoint(-turn_speed);


	_state = state_turning;

	_motor_left.run();
	_motor_right.run();

	while (_motor_left.running() || _motor_right.running())
	{
		current_angle = _sensor_gyro.value();
	
		if(abs(start_angle - current_angle) > final_angle)
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
	/*
	//Initialize sensors
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	*/
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

	cout << "Press the touch sensor to make robot turn\n";

	while (!c.return_sensor_value(TOUCH));

	int mode = 3;


	c.turn_gyro(30, 600);
	c.stop();


	return 0;
}

