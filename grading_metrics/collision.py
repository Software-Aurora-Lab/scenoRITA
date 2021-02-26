import sys
import math
import argparse
from multiprocessing import Queue
from shapely.geometry import Polygon
from modules.tools.perception.replay_perception import generate_polygon
from automation.auxiliary.record.read_record import print_msg_num, read_by_path

COLLISION_DETECTED = 0


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'record_path',
        help='path of the record to test'
    )
    args = parser.parse_args()
    return args


def get_sample_range(messages):
    max_gap = 0
    prev_time = 0
    for channel_name, _, _, _, timestamp in messages:
        if channel_name == '/apollo/localization/pose':
            # Update the max range if we found larger range
            if prev_time != 0:
                gap = timestamp - prev_time
                if gap > max_gap:
                    max_gap = gap
            prev_time = timestamp   # update prev_time to current time stamp
    # print(f'the max gap is {max_gap}')
    return max_gap


def construct_obs_polygon(obstacle):
    """
    Description: Given the obstacle message, construct and return its boundary polygon

    Input: An obstacle message

    Output: A Shapely Polygon object representing the obstacle boundary 
    """
    polygon_points = []
    for point in obstacle.polygon_point:
        polygon_points.append((point.x, point.y))  # ignore the z values
    return Polygon(polygon_points)


def construct_adc_polygon(adc_pose):
    """
    Description: Given the adc pose message, construct and return its boundary polygon

    Input: An adc pose message

    Output: A Shapely Polygon object representing the ego car boundary 
    """
    adc_point = adc_pose.position
    polygon_points = generate_polygon(
        adc_point, adc_pose.heading, 4.933, 2.11)
    return Polygon([(point.x, point.y) for point in polygon_points])


def test_collisions(messages):
    # The most recent localization info
    adc_pose = None
    min_dist = {}

    init_timestamp = None
    for channel_name, _, parsed_msg, _, _ in messages:
        current_timestamp = parsed_msg.header.timestamp_sec
        if init_timestamp is None:
            init_timestamp = current_timestamp
            continue
        if current_timestamp - init_timestamp <= 5:
            continue

        # Update the localization states if a new localization message arrives
        if channel_name == '/apollo/localization/pose':
            adc_pose = parsed_msg.pose
        # Compare the perception states with the most recent
        # localization states (if the time gap is within max range)
        elif (channel_name == '/apollo/perception/obstacles'):
            for obstacle in parsed_msg.perception_obstacle:
                # Skip if no localization currently
                if adc_pose is None:
                    continue

                ego_car_polygon = construct_adc_polygon(adc_pose)
                obs_car_polygon = construct_obs_polygon(obstacle)
                dist = ego_car_polygon.distance(obs_car_polygon)

                if str(obstacle.id) in min_dist:
                    if min_dist[str(obstacle.id)] > dist:
                        min_dist[str(obstacle.id)] = dist
                else:
                    min_dist[str(obstacle.id)] = dist
    return min_dist


def walk_messages(record_path, verbose=False, return_dict=None):
    if verbose:
        print('\nStart checking collision...')

    messages = read_by_path(record_path)    # fetch all messages from record
    # find and report collisions
    min_dist = test_collisions(messages)

    if return_dict is not None:
        return_dict['min_dist'] = min_dist

    if verbose:
        print(f'min_dist = {min_dist}')
        print('Finished checking collision\n')

    return min_dist


def main():
    args = get_args()
    record_path = args.record_path

    walk_messages(record_path, verbose=True)


if __name__ == '__main__':
    main()
