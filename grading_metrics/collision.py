import sys
import math
import argparse
from multiprocessing import Queue
from shapely.geometry import Polygon, LineString
from modules.tools.perception.replay_perception import generate_polygon
from automation.auxiliary.record.read_record import print_msg_num, read_by_path
from automation.auxiliary.oracles.speeding import calculate_speed

COLLISION_DETECTED = 0

DEFAULT_ADC_LENGTH = 4.933
DEFAULT_ADC_WIDTH = 2.11


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
        adc_point, adc_pose.heading, DEFAULT_ADC_LENGTH, DEFAULT_ADC_WIDTH)
    return Polygon(sorted([(point.x, point.y) for point in polygon_points]))


def get_adc_sides(adc_pose, length=DEFAULT_ADC_LENGTH, width=DEFAULT_ADC_WIDTH):
    """
    Description: Gievn the adc pose message, return the vertices for each side of the adc

    Input: An adc pose message

    Ouput: An array containing 4 tuples, each tuple containing the two vertices of the adc contained in a tuple
        e.g. [ ((left1x, left1y), (left2x, left2y))... ]
    """
    result_sides = []
    center_point = adc_pose.position
    heading = adc_pose.heading

    if center_point is None or heading is None:
        return None

    half_length = length / 2.0
    half_width = width / 2.0
    sin_h = math.sin(heading)
    cos_h = math.cos(heading)

    # each element: (x, y)
    rotated_points = [(half_length * cos_h - half_width * sin_h,
                       half_length * sin_h + half_width * cos_h),
                      (-half_length * cos_h - half_width * sin_h,
                       - half_length * sin_h + half_width * cos_h),
                      (-half_length * cos_h + half_width * sin_h,
                       - half_length * sin_h - half_width * cos_h),
                      (half_length * cos_h + half_width * sin_h,
                       half_length * sin_h - half_width * cos_h)]

    for point in rotated_points:
        result_sides.append(
            (center_point.x + point[0], center_point.y + point[1]))

    front_side = [result_sides[0], result_sides[3]]
    rear_side = [result_sides[1], result_sides[2]]
    right_side = [result_sides[3], result_sides[2]]
    left_side = [result_sides[0], result_sides[1]]
    return [front_side, rear_side, left_side, right_side]


def get_adc_rear_vertices(adc_pose, length=DEFAULT_ADC_LENGTH, width=DEFAULT_ADC_WIDTH):
    """
    Description: Gievn the adc pose message, return the rear vertices of the adc

    Input: An adc pose message

    Ouput: An array containing two points (each a tuple) representing the rear vertices
    """
    # Check if necessary fields are present
    center_point = adc_pose.position
    heading = adc_pose.heading
    if center_point is None or heading is None:
        return None

    half_length = length / 2.0
    half_width = width / 2.0
    sin_h = math.sin(heading)
    cos_h = math.cos(heading)

    # each element: (x, y)
    result = []
    rotated_points = [
        (-half_length * cos_h - half_width * sin_h,
         - half_length * sin_h + half_width * cos_h),
        (-half_length * cos_h + half_width * sin_h,
         - half_length * sin_h - half_width * cos_h)
    ]
    for point in rotated_points:
        result.append((center_point.x + point[0], center_point.y + point[1]))
    return result


def is_rear_end_collision(adc_pose, obs_polygon):
    """
    Description: Given the adc pose message and obstable polygon, determine if there is a rear-end collison

    Input:
        1. adc pose message
        2. Obstacle Polygon

    Output:
        Boolean value indicating if there is a rear-end collision
    """
    # Fecth rear edge and make sure data are present
    if adc_pose is None:
        return None
    rear_vertices = get_adc_rear_vertices(adc_pose)
    if rear_vertices is None:
        return None

    rear_linestring = LineString(rear_vertices)
    return obs_polygon.intersects(rear_linestring)


def is_collision(adc_pose, obs_polygon):
    """
    Description: Given the adc pose message and obstable polygon, determine if there is a collison

    Input:
        1. adc pose message
        2. Obstacle Polygon

    Output:
        A list of collided sides
    """
    if adc_pose is None:
        return None
    vertices = get_adc_sides(adc_pose)
    if vertices is None:
        return None

    [front, rear, left, right] = vertices

    left_linestring = LineString(left)
    right_linestring = LineString(right)
    rear_linestring = LineString(rear)
    front_linestring = LineString(front)

    collided_sides = []

    if obs_polygon.intersects(rear_linestring):
        collided_sides.append('rear')
    if obs_polygon.intersects(front_linestring):
        collided_sides.append('front')
    if obs_polygon.intersects(left_linestring):
        collided_sides.append('left')
    if obs_polygon.intersects(right_linestring):
        collided_sides.append('right')
    return collided_sides


def fetch_obs_states(obs_msg):
    obs_id = obs_msg.id
    obs_speed = calculate_speed(obs_msg.velocity)
    heading = obs_msg.theta
    return obs_id, obs_speed, heading


def test_collisions(messages):
    # The most recent localization info
    init_pose = None
    adc_pose = None
    min_dist = {}
    # all the obstacle (id) rear-ended the adc
    ignored_obs = set()
    # first obstacle (id) rear-ended the adc when adc is above 5km/h
    collision_flag = True
    init_timestamp = None

    collision = None

    for channel_name, _, parsed_msg, _, _ in messages:
        current_timestamp = parsed_msg.header.timestamp_sec
        if init_timestamp is None:
            init_timestamp = current_timestamp
            continue
        if current_timestamp - init_timestamp <= 5:
            continue

        # Update the localization states if a new localization message arrives
        if channel_name == '/apollo/localization/pose':
            # Update the initial pose of the adc
            if init_pose is None:
                init_pose = parsed_msg.pose
            adc_pose = parsed_msg.pose

        # Compare the perception states with the most recent
        # localization states (if the time gap is within max range)
        elif (channel_name == '/apollo/perception/obstacles'):
            for obstacle in parsed_msg.perception_obstacle:
                # Skip if no localization currently
                if adc_pose is None or str(obstacle.id) in ignored_obs:
                    continue

                current_x = adc_pose.position.x
                current_y = adc_pose.position.y

                ego_car_polygon = construct_adc_polygon(adc_pose)
                obs_car_polygon = construct_obs_polygon(obstacle)

                dist = ego_car_polygon.distance(obs_car_polygon)

                if str(obstacle.id) in min_dist:
                    if min_dist[str(obstacle.id)] > dist:
                        min_dist[str(obstacle.id)] = dist
                else:
                    min_dist[str(obstacle.id)] = dist

                # Update the collided sides of a collision
                collided_sides = is_collision(adc_pose, obs_car_polygon)
                if len(collided_sides) > 0:
                    ignored_obs.add(str(obstacle.id))
                    if (calculate_speed(adc_pose.linear_velocity) * 3.6 > 5) and collision_flag: 
                        collision = [
                            obstacle.id,
                            current_x,
                            current_y,
                            collided_sides,
                            adc_pose.heading,
                            calculate_speed(adc_pose.linear_velocity) * 3.6,
                            calculate_speed(obstacle.velocity) * 3.6,
                            obstacle.theta
                        ]
                        collision_flag = False
                    continue

    # Naive way of deciding if the adc has moved by comparing the initial pose and end pose
    adc_moved = False
    if init_pose is not None and adc_pose is not None:
        init_poly = construct_adc_polygon(init_pose)
        adc_poly = construct_adc_polygon(adc_pose)
        adc_moved = not init_poly.equals_exact(adc_poly, tolerance=10.0)
    return ((min_dist if adc_moved else set()), collision)


def walk_messages(record_path, verbose=False, return_dict=None):
    if verbose:
        print('\nStart checking collision...')

    messages = read_by_path(record_path)    # fetch all messages from record
    # find and report collisions
    min_dist, collision = test_collisions(messages)

    if return_dict is not None:
        return_dict['min_dist'] = min_dist
        return_dict['collision_states'] = collision

    if verbose:
        for obs_id in min_dist.keys():
            dist = min_dist[obs_id]
            print(
                f'collision: obstacle id = {obs_id},\t\tdist = {dist}')

        print(collision)
        print('Finished checking collision\n')

    return min_dist


def main():
    args = get_args()
    record_path = args.record_path

    walk_messages(record_path, verbose=True)


if __name__ == '__main__':
    main()
