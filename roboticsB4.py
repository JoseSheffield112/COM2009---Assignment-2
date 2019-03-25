#!/usr/bin/env python3
'''COM2009 ev3dev Assignment1'''

import os
import sys
import time
import ev3dev.ev3 as ev3

'''Provided Functions'''

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font
    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)

''' End provided functions '''


def debug_printer(var_name, var_val):
    debug_print(var_name)
    debug_print(var_val)


def actuate(m_left, m_right, v, dv_left, dv_right):
    ''' Function to actuate motors 
    Takes 2 motors as inputs, v for base speed, dv for diff 
    returns nil
    '''
    left = v-dv_left
    right = v+dv_right

    debug_printer("Fed into motor left:", (left))
    debug_printer("Fed into right motor:", (right))

    #flip

    left =-left
    right=-right

    m_left.run_direct(duty_cycle_sp=left)
    m_right.run_direct(duty_cycle_sp=right)


def get_response(kp, ki, kd, dt, cur_error, prev_error, integral):
    '''Get PID response'''
    integral = 0
    integral+= cur_error * dt
    derivative = (cur_error - prev_error) / dt

    return (kp * cur_error) + (ki * integral) + (kd * derivative)


''' hk error metric - difference of sensors '''
def hk_metric(l_dist, r_dist):
    return (l_dist-r_dist)

def main_program(l_dist, r_dist, min_dist):
    # If we're too close to left side, increase speed there
    if ((l_dist<min_dist) and (r_distM>=min_dist)):
        error_l = (min_dist-l_dist)# if broken use :(l_dist-min_dist) 
        erro_r = -1*error_l
    # If we're too close to right side, increase speed there
    elif ((l_dist>=min_dist) and (r_dist<min_dist)):
        error_r = (r_dist-min_dist)# if broken use :(min_dist-r_dist)
        error_l = -1*error_r
    # Otherwise go at constant speed
    else:
        error_l = error_r = 0

    return(error_l, error_r)

def main():
    '''Main calling Function'''

    # Sampling rate
    sampling_rate = 0.0001

    # Motor vars
    m_left = ev3.LargeMotor('outB')
    m_right = ev3.LargeMotor('outC')

    motor = ev3.MediumMotor('outA')

    # Sensors
    us_left = ev3.UltrasonicSensor('in3')
    us_right = ev3.UltrasonicSensor('in2')

    # motor speed range
    max_speed = 100
    base_speed = 90

    # Set distance
    min_dist = 250

    # PID - Initialise
    integral = 0
    last_error_left = 0
    last_error_right = 0
    ku = 8
    tu = 0.1

    # Zieger Nichols Calculated value
    kp = 1
    ki = 0.8
    kd = 0.168

    ''' Actual dt does not equal our sampling rate. Code may take longer to run.
        So we time everytime the code runs, this is inaccurate at first but should be better 
        in the long run '''
    time_then = time.time()

    # Control - Loop
    while True:
        time.sleep(sampling_rate)
        #getting the left & right side values
        left_value, right_value = us_left.value(), us_right.value()

        time_now = time.time()
        dt = time_now - time_then
        time_then = time_now

        #If we're in a tight space, we use the maze runner metric
        if ((left_value<min_dist) and (right_value<min_dist)):
            error_left = error_right = hk_metric(left_value,right_value)
        #Otherwise we use the open space metric
        else:
            error_left, error_right = main_program(left_value, right_value, min_dist)

        debug_printer("Error from Metric used:", error)

        dv_left = get_response(kp, ki, kd, dt, error_left, last_error_left, integral)
        dv_right = get_response(kp, ki, kd, dt, error_right, last_error_right, integral)

        # Overspeed stopper
        if dv_left > max_speed - base_speed:
            dv_left = max_speed - base_speed
        elif dv_right > max_speed - base_speed:
            dv_right = max_speed - base_speed
        elif dv_left < base_speed-max_speed:
            dv_left = base_speed - max_speed
        elif dv_right < base_speed-max_speed:
            dv_right = base_speed - max_speed

        actuate(m_left, m_right, base_speed, dv_left, dv_right)

        # Copy previous error rate
        last_error_left = error_left
        last_error_right = error_right

if __name__ == '__main__':
main()


# Uses color sensor to measure ambient light percentage
#def color_sensor(self):
#    self._ensure_mode(self.MODE_COL_AMBIENT)
#    return self.value(0)

# to stop motors
#off(brake=True)
