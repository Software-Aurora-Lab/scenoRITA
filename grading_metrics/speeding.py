import sys
import math
import time
import argparse
from shapely.geometry import Point
from multiprocessing import Process, Manager

from automation.auxiliary.map import map_tools
from automation.auxiliary.oracles.speeding import calculate_speed
from automation.auxiliary.record.read_record import read_by_path
from automation.grading_metrics.collision import construct_adc_polygon

VERBOSE = False
SKIP_NUM = 100    # Skip messages to print
OFF_LANE_THRESHOLD = 5


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'record_path',
        help='path to the record file to test',
        type=str
    )
    args = parser.parse_args()
    return args


def report_offroad_violation(x, y, timestamp, verbose=False):
    """
    Output off-road violations to a file (or terminal if specified)
    """
    output_str = f'Off-road violation:\n'
    output_str += f'x: {x}, y: {y}, time: {timestamp}\n'
    if verbose:
        print(output_str)


def v_print(*str_tuple, endl='\n'):
    if VERBOSE:
        for str in str_tuple:
            print(str, end=' ')
        print(endl, end='')


def get_next_lanes(routing_response_msg, current_lane_id=None):
    """
    Description:
        Given the current lane id, return a list of lanes that the adc might go next.
        Algorithm: walk through all the passages in the routing response,
                    try to locate the passage(s) containing the current lane id,
                    and return the lane ids after (including) it from the located
                    passages.
    Input:
        A routing response message.
        (optional) Current lane_id of the adc.
    Output:
        A list of lane (ids) after current lane id in the routing response message.
    """
    next_lanes = []
    is_hit = False
    for road in routing_response_msg.road:
        for passage in road.passage:
            for segment in passage.segment:
                lane_id = segment.id
                if lane_id == current_lane_id:
                    is_hit = True
                if is_hit or current_lane_id is None:
                    next_lanes.append(lane_id)
    return next_lanes


def walk_msg_section(messages: list, lanes: dict, lanes_set: set, speed_list: list, dist_list: list):
    # Violation coordinates for speeding
    speeding_coord = None

    traveled_lanes = set()
    current_lane = None     # the lane id based on the latest localization
    priority_lanes = None   # the lanes most probably for adc to reside
    # the lane adc resides when the adc polygon intersects the lane boundary
    off_road_candidate_lane = None
    intersect_start_time = None
    next_lanes = []

    speeding_duration_flag = False
    speeding_duration_start = None      # start time for speeding violation
    speeding_duration_end = None        # end time for speeding violation
    speeding_heading = None             # adc heading when speeding
    speeding_x = None
    speeding_y = None
    speeding_value = None

    ulc_duration_flag = False
    ulc_duration_start = None
    ulc_duration_end = None
    ulc_heading = None                  # adc heading when unsafe lane-change
    ulc_x = None
    ulc_y = None

    min_speed = sys.maxsize
    dist_to_boundary = sys.maxsize
    speed_diff = 0

    localization_num = 0
    init_timestamp = None
    for channel_name, _, parsed_msg, _, _ in messages:
        current_time = parsed_msg.header.timestamp_sec
        if channel_name == '/apollo/planning':
            routing_response_msg = parsed_msg.debug.planning_data.routing
            next_lanes = get_next_lanes(routing_response_msg, current_lane)

        if channel_name == '/apollo/localization/pose':
            localization_num += 1

            # v_print(localization_num)

            current_pos = parsed_msg.pose.position
            current_x = current_pos.x
            current_y = current_pos.y

            # linear_velocity in meters per second
            # current_speed in kilometers per hour
            current_speed = calculate_speed(
                parsed_msg.pose.linear_velocity) * 3.6

            init_time = time.time()
            current_lanes = map_tools.efficient_fetch_lane(
                current_x, current_y, current_lane, lanes, priority_lanes=priority_lanes)
            '''
            Note: at this point, if len(current_lanes) == 0, the adc is off-lane
            '''
            if len(current_lanes) == 1:
                current_lane = current_lanes[0]
            elif len(current_lanes) > 1:
                back_lane = None
                loc = None
                for suspect_lane in current_lanes:
                    try:
                        loc = next_lanes.index(suspect_lane)
                    except ValueError:
                        if suspect_lane in traveled_lanes:
                            back_lane = suspect_lane
                if loc is not None:
                    current_lane = next_lanes[loc]
                elif back_lane is not None:
                    current_lane = back_lane
                else:
                    current_lane = current_lanes[0]
            else:
                current_lane = None

            end_time = time.time()
            # v_print(f'elasped {end_time-init_time} seconds')

            # Ignore the first 5 seconds for the adc to get ready
            if init_timestamp is None:
                init_timestamp = current_time
                continue
            if current_time - init_timestamp <= 5:
                continue

            # Do nothing when the adc is off-lane
            if current_lane is None:
                continue

            # Speeding detection
            traveled_lanes.add(current_lane)

            # Fetch speed limit and unify the unit
            # proto speed_limit in meters per second
            # convert to speed_limit in kilometers per hour
            speed_limit = (lanes[current_lane].speed_limit) * 3.6

            if speed_limit != 0 or len(current_lanes) > 1:
                # Calculating the speed difference is unfair when there's no speed limit
                if (
                    localization_num % SKIP_NUM == 0
                    or speed_limit < current_speed
                    or localization_num < 5
                ):
                    v_print(current_lane, speed_limit,
                            current_speed, end_time - init_time,
                            current_lanes)

                speed_diff = speed_limit - current_speed

                # Calculate the duration of the first speeding violation
                if speed_diff <= -5:
                    # speeding_duration_start is None when the first violation occurs
                    if speeding_duration_start is None:
                        speeding_duration_start = current_time
                        speeding_duration_flag = True
                        speeding_heading = parsed_msg.pose.heading
                        speeding_value = abs(speed_diff)
                        speeding_x = current_x
                        speeding_y = current_y
                    if speeding_duration_flag:
                        speeding_duration_end = current_time
                else:
                    speeding_duration_flag = False

                if speed_diff < min_speed:
                    min_speed = speed_diff
                    speeding_coord = (current_x, current_y)

            # Off-road (lane) detection
            # lane_linestring = map_tools.construct_lane_linestring(
            #     lanes[current_lane])
            lane_boundaries = map_tools.construct_lane_boundary_linestring(
                lanes[current_lane]
            )
            ego_car_polygon = construct_adc_polygon(parsed_msg.pose)
            # dist_to_boundary = min(ego_car_polygon.distance(
            #     lane_boundaries[0]), ego_car_polygon.distance(lane_boundaries[1]))
            dist1 = ego_car_polygon.distance(lane_boundaries[0])
            dist2 = ego_car_polygon.distance(lane_boundaries[1])
            dist_to_boundary = min(dist1, dist2)
            # print(f"dist1 = {dist1}, dis2 = {dist2}")

            if off_road_candidate_lane is None and dist_to_boundary == 0:
                # Register the first instance of lane intersection
                off_road_candidate_lane = current_lane
                intersect_start_time = current_time
            elif intersect_start_time is not None:
                # When the first instance of lane intersection is registered
                time_gap = current_time - intersect_start_time
                if time_gap >= OFF_LANE_THRESHOLD:
                    if dist_to_boundary == 0:
                        # adc_center_point = Point(current_x, current_y)
                        # dist_list.append(
                        #     adc_center_point.distance(lane_linestring))
                        dist_list.append((0, (current_x, current_y)))

                        if ulc_duration_start is None:
                            ulc_duration_start = intersect_start_time
                            ulc_heading = parsed_msg.pose.heading
                            ulc_x = current_x
                            ulc_y = current_y

                            ulc_duration_flag = True
                        if ulc_duration_flag:
                            ulc_duration_end = current_time
                    else:
                        off_road_candidate_lane = None
                        intersect_start_time = None
                        ulc_duration_flag = False
                elif time_gap < OFF_LANE_THRESHOLD and dist_to_boundary != 0:
                    # Remove the registration if the adc moved back/forward to lane
                    off_road_candidate_lane = None
                    intersect_start_time = None
            else:
                # When no intersection is registered
                # adc_center_point = Point(current_x, current_y)
                dist_list.append((dist_to_boundary, (current_x, current_y)))

    speeding_duration = None
    speeding_state = None
    if speeding_duration_start is not None:
        speeding_duration = speeding_duration_end - speeding_duration_start
        speeding_state = ((speeding_x, speeding_y), speeding_value,
                          speeding_duration, speeding_heading)

    ulc_duration = None
    ulc_state = None
    if ulc_duration_start is not None:
        ulc_duration = ulc_duration_end - ulc_duration_start
        ulc_state = ((ulc_x, ulc_y), ulc_heading, ulc_duration)
    # dist_list.append(((ulc_x, ulc_y), ulc_heading, ulc_duration))

    lanes_set.update(traveled_lanes)
    speed_list.append((min_speed,
                       speeding_coord)
                      )
    # speed_list.append((speeding_x, speeding_y),
    #                   speeding_value,
    #                   speeding_duration,
    #                   speeding_heading
    #                   )

    return ulc_state, speeding_state


def walk_messages(record_path, return_dict=None):
    '''
    Iterate through localizaiton messages, check
    if the ego car exceeds the speed limit of current lane
    '''
    messages = read_by_path(record_path)

    map_msg = map_tools.load_mapbin()
    lanes = map_tools.cache_lanes(map_msg)

    speed_list = list()
    dist_list = list()

    speeding_coord = None
    dist_coord = None
    # Output variables
    traveled_lanes = set()
    min_speed = sys.maxsize
    min_dist = sys.maxsize

    ulc_state, speeding_state = walk_msg_section(
        messages, lanes, traveled_lanes, speed_list, dist_list)

    for (speed, coord) in speed_list:
        if speed < min_speed:
            min_speed = speed
            speeding_coord = coord
    for (dist, coord) in dist_list:
        if dist < min_dist:
            min_dist = dist
            dist_coord = coord

    return_lanes = set()

    if return_dict is None:
        print('traveled_lanes:')
        for lane in traveled_lanes:
            print(
                f'lane id: {lane}\t\tspeed limit: {round(lanes[lane].speed_limit * 3.6, 1)}')
        # print(f'min_speed = {min_speed}, speeding_coord = {speeding_coord}')
        # print(f'min_dist (to boundary) = {min_dist}, coord = {dist_coord}')
        # print(f'speeding_state = {speeding_state}')
        # print(f'ulc state = {ulc_state}')
        print(f'min_speed = {(min_speed, speeding_coord, speeding_state)}')
        print(f'boundary_dist = {(min_dist, dist_coord, ulc_state)}')
        return

    for lane in traveled_lanes:
        # lane_str = lane.replace('id: ', '').replace('\"', '').strip('\n')
        speed_limit = lanes[lane].speed_limit * 3.6
        return_lanes.add((lane, round(speed_limit, 3),))

    return_dict['traveled_lanes'] = return_lanes
    return_dict['min_speed'] = (min_speed, speeding_coord, speeding_state)
    return_dict['boundary_dist'] = (min_dist, dist_coord, ulc_state)


def main():
    global VERBOSE
    VERBOSE = True
    args = get_args()
    record_path = args.record_path

    start_time = time.time()
    walk_messages(record_path)
    end_time = time.time()

    print(f'speeding elapsed time: {end_time-start_time} seconds')


if __name__ == '__main__':
    main()
