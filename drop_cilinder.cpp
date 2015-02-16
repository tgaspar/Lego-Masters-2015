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
  
  void drive(int speed, int time=0);
  void turn(int direction);
  void turn_gyro(int angle);
  void open_and_close(int angle);
  
  void stop();
  void reset();
  
  bool initialized() const;
  bool initialize();
  
  
  void terminate_on_key();
  void panic_if_touched();
  
  void remote_loop();
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

void control::turn(int direction)
{
  if (_state != state_idle)
    stop();

  if (direction == 0)
    return;
	
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	int angle = _sensor_gyro.value();
	int startAngle = _sensor_gyro.value();
	int distance = _sensor_ultrasonic.value();
  
  _motor_left.set_run_mode(motor::run_mode_position);
  //_motor_left.set_position_mode(motor::position_mode_relative);
  _motor_left.set_position(0);
  _motor_left .set_position_setpoint(direction);
  //_motor_left.set_regulation_mode(motor::mode_on);
  _motor_left.set_duty_cycle_setpoint(50);

  _motor_right.set_run_mode(motor::run_mode_position);
  //_motor_right.set_position_mode(motor::position_mode_relative);
  _motor_right.set_position(0);
  _motor_right.set_position_setpoint(-direction);
  //_motor_right.set_regulation_mode(motor::mode_on);
  _motor_right.set_duty_cycle_setpoint(50);
  
  _state = state_turning;

  _motor_left .run();
  _motor_right.run();

  while (_motor_left.running() || _motor_right.running())
	{
		angle = _sensor_gyro.value();
		distance = _sensor_ultrasonic.value();
		this_thread::sleep_for(chrono::milliseconds(10));
		sensorlog << angle << "," << distance << "\n";
		cout <<"startAngle - angle: " << startAngle - angle << "\n";
		if(abs(startAngle - angle) > 360)
		{
				cout << "The motor should be stopping by now! \n";
				_motor_left.stop();
				_motor_right.stop();
				break;
		}
	}

  _state = state_idle;
}

void control::turn_gyro(int final_angle)
{
  if (_state != state_idle)
    stop();

  if (final_angle == 0)
    return;
	
	_sensor_gyro.set_mode(gyro_sensor::mode_angle);
	_sensor_ultrasonic.set_mode(ultrasonic_sensor::mode_dist_cm);
	int angle = _sensor_gyro.value();
	int startAngle = _sensor_gyro.value();
	int distance = _sensor_ultrasonic.value();
	
  _motor_left .set_run_mode(motor::run_mode_forever);
  _motor_right.set_run_mode(motor::run_mode_forever);
  
  _motor_left.set_regulation_mode(motor::mode_on);
  _motor_right.set_regulation_mode(motor::mode_on);
  
  
  _motor_left.set_pulses_per_second_setpoint(100);
  _motor_right.set_pulses_per_second_setpoint(-100);
  
  cout << "Left motor PPS set point: " << _motor_left.pulses_per_second_setpoint() << "\n";
  cout << "Right motor PPS set point: " << _motor_right.pulses_per_second_setpoint() << "\n";
  
  _state = state_turning;

  _motor_left.run();
  _motor_right.run();

  while (_motor_left.running() || _motor_right.running())
	{
		angle = _sensor_gyro.value();
		
		if (_sensor_ultrasonic.value() > 2500)
		{
			distance = distance;
			//cout << "The proximity value is weird, mate! \n";
			//_motor_left.stop();
			//_motor_right.stop();
			//break;
		}
		else
		{
			distance = _sensor_ultrasonic.value();
		}
		
		distance = _sensor_ultrasonic.value();
	
		//this_thread::sleep_for(chrono::milliseconds(10));
		sensorlog << startAngle - angle << "," << distance << "\n";
		//cout <<"distance =  " << distance << "\n";
		if(abs(startAngle - angle) > final_angle)
		{
				cout << "The motor should be stopping by now! \n";
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
  return (_motor_left .connected() &&
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

void control::remote_loop()
{
  remote_control r(_sensor_ir);
  
  if (!r.connected())
  {
    cout << "no infrared sensor found!" << endl;
    return;
  }
  
  const int speed = 70;
  const int ninety_degrees = 260;
  
  r.on_red_up = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        drive(speed);
    }
    else
      stop();
  };
  
  r.on_red_down = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        drive(-speed);
    }
    else
      stop();
  };

  r.on_blue_up = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        turn(-ninety_degrees);
    }
  };
  
  r.on_blue_down = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        turn(ninety_degrees);
    }
  };
  
  r.on_beacon = [&] (bool state)
  {
    if (state)
      terminate();
  };
  
  while (!_terminate)
  {
    if (!r.process())
    {
      this_thread::sleep_for(chrono::milliseconds(10));
    }
  }
  
  reset();
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

	sensorlog.open ("sensorlog.csv");

	cout << "Initializing \n";

	c.initialize();

	//if (c.initialized())
	//{

	// change mode to 1 to get IR remote mode
	// change mode to 3 for wooot?
	int mode = 3;

	//c.turn_gyro(380);
	cout << "It should go to the open_and_close function now! \n";
	c.open_and_close(40);
	c.stop();

	c.terminate_on_key(); // we terminate if a button is pressed
	c.panic_if_touched(); // we panic if the touch sensor is triggered


	//}
	/*
	else
	{
	cout << "you need to connect an infrared sensor and large motors to ports B and C!" << endl;
	return 1;
	}
	*/
	sensorlog.close(); 

	return 0;
}

