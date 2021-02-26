import math
import argparse
import sys

from automation.auxiliary.record.read_record import read_by_path
from automation.grading_metrics.speeding import calculate_speed

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

    magnitutde = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

    projection = (accel_x * linear_velocity.x) + \
        (accel_y * linear_velocity.y) + (accel_z * linear_velocity.z)

    # print(accel_x, linear_velocity.x, accel_y, linear_velocity.y, magnitutde)

    if projection < 0:
        return magnitutde * -1
    return magnitutde


def test_acceleration(messages, accel_target, verbose=False):
    max_accel = -1
    can_begin = False
    init_time = None
    current_time = None
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

            accel = calculate_acceleration(
                parsed_msg.pose.linear_acceleration,
                parsed_msg.pose.linear_velocity,
            )

            if ORACLE_TYPE == "hard braking":
                max_accel = min(max_accel, accel)
            else:
                max_accel = max(max_accel, accel)

    if verbose:
        print(max_accel)

    return max_accel


def get_oracle_type(accel_value):
    return 'hard braking' if accel_value < 0 else 'fast acceleration'


def walk_messages(record_path, accel_value, verbose=False, return_dict=None):
    global ORACLE_TYPE
    ORACLE_TYPE = get_oracle_type(accel_value)

    if verbose:
        print(f'\nStart checking {ORACLE_TYPE}')

    messages = read_by_path(record_path)
    output = test_acceleration(messages, accel_value, verbose=verbose)

    if return_dict is not None:
        if ORACLE_TYPE == "hard braking":
            return_dict['hardbreak'] = output
        else:
            return_dict['accl'] = output

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
