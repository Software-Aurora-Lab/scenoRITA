import math
import argparse
import sys

from automation.auxiliary.record.read_record import read_by_path
from automation.auxiliary.oracles.speeding import calculate_speed

VIOLATION_DETECTED = 0
ORACLE_TYPE = None
SAMPLE_GAP = 1.0
SPEED_LIST = list()


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'record_path',
        help='path of the record to test',
        type=str
    )
    parser.add_argument(
        'acceleration_value',
        help='the target value of the acceleration/hard braking',
        type=int
    )
    args = parser.parse_args()
    return args


def calculate_acceleration(linear_acceleration, linear_velocity, verbose=False):
    accel_x = linear_acceleration.x
    accel_y = linear_acceleration.y
    accel_z = linear_acceleration.z

    magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

    projection = (accel_x * linear_velocity.x) + \
        (accel_y * linear_velocity.y) + (accel_z * linear_velocity.z)

    if projection < 0:
        return magnitude * -1
    return magnitude


def test_acceleration(messages, accel_target, verbose=False):
    max_accel = -1
    can_begin = False
    init_time = None
    current_time = None

    violation_coord = None      # adc coordinate of the violation
    violation_value = None      # accel/deaccel value of the first violation
    adc_heading = None          # adc heading of the violation

    duration_flag = False
    duration_start = None       # start time of the violation
    duration_end = None         # end time of the violation

    if ORACLE_TYPE == "hard braking":
        max_accel = sys.maxsize

    for channel_name, _, parsed_msg, _, _ in messages:
        if channel_name == '/apollo/localization/pose':
            current_time = parsed_msg.header.timestamp_sec
            current_speed = calculate_speed(parsed_msg.pose.linear_velocity)

            if current_speed == 0:
                can_begin = True

            if not can_begin:
                continue

            if init_time is None:
                init_time = current_time

            # accel in meters per square second
            accel = calculate_acceleration(
                parsed_msg.pose.linear_acceleration,
                parsed_msg.pose.linear_velocity,
            )

            if ORACLE_TYPE == "hard braking":
                if accel < accel_target:
                    # The first violation happens if duration start is None
                    if duration_start is None:
                        # Record the coordinate, heading of the first occurance of violation
                        violation_coord = (
                            parsed_msg.pose.position.x, parsed_msg.pose.position.y)
                        adc_heading = parsed_msg.pose.heading
                        duration_flag = True    # can start counting duration
                        duration_start = current_time
                        violation_value = accel
                    if duration_flag:
                        duration_end = current_time
                else:
                    # stop counting duration as the violation ends (or didnt occur)
                    duration_flag = False
                max_accel = min(max_accel, accel)
            else:
                if accel >= accel_target:
                    if duration_start is None:
                        violation_coord = (
                            parsed_msg.pose.position.x, parsed_msg.pose.position.y)
                        adc_heading = parsed_msg.pose.heading
                        duration_flag = True
                        duration_start = current_time
                        violation_value = accel
                    if duration_flag:
                        duration_end = current_time
                else:
                    duration_flag = False
                max_accel = max(max_accel, accel)

    if verbose:
        print(max_accel, violation_coord)

    duration = None
    if duration_start is not None:
        duration = duration_end - duration_start

    return max_accel, violation_value, violation_coord, adc_heading, duration


def get_oracle_type(accel_value):
    return 'hard braking' if accel_value < 0 else 'fast acceleration'


def walk_messages(record_path, accel_value, verbose=False, return_dict=None):
    global ORACLE_TYPE
    ORACLE_TYPE = get_oracle_type(accel_value)

    if verbose:
        print(f'\nStart checking {ORACLE_TYPE}')

    messages = read_by_path(record_path)
    max_accel, violation_value, coord, heading, duration = test_acceleration(
        messages, accel_value, verbose=verbose)

    if return_dict is not None:
        if ORACLE_TYPE == "hard braking":
            return_dict['hardbreak'] = (
                max_accel, violation_value, coord, heading, duration)
        else:
            return_dict['accl'] = (
                max_accel, violation_value, coord, heading, duration)

    if verbose:
        print(f'Finished checking {ORACLE_TYPE}\n')


def main():
    global ORACLE_TYPE

    args = get_args()
    record_path = args.record_path
    accel_value = args.acceleration_value

    ORACLE_TYPE = get_oracle_type(accel_value)

    walk_messages(record_path, accel_value, verbose=True)


if __name__ == '__main__':
    VERBOSE = True
    main()
