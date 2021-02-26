'''
A collection of map related tools
'''
import sys
import numpy as np
from modules.map.proto import map_pb2
from shapely.geometry import Point, Polygon, LineString


DEFAULT_SIM_MAP_PATH = '/apollo/modules/map/data/sunnyvale_loop/sim_map.bin'


def load_mapbin(sim_map_path=None):
    """
    Description: Parse the map binary into a protobuf message.

    Input: Path to the sim map binary.

    Ouput: The map protobuf message read from the input binary.
    """
    if sim_map_path is None:
        sim_map_path = DEFAULT_SIM_MAP_PATH

    map_file = open(sim_map_path, 'rb')
    map_data = map_file.read()
    map_file.close()

    map_msg = map_pb2.Map()
    map_msg.ParseFromString(map_data)

    return map_msg


def cache_lanes(map_msg: map_pb2.Map):
    """
    Description: Cache all the lanes from map proto into a
    dictionary for faster reference.

    Input: A map message.

    Output: A dictionary containing all lanes in the map message
            as values, whose keys are their ids.
    """
    lanes = dict()
    for lane_msg in map_msg.lane:
        lanes[str(lane_msg.id.id)] = lane_msg
    return lanes


def cache_roads(map_msg: map_pb2.Map):
    """
    Description: Cache all the road messages from map
    proto into a dictionary for faster reference

    Input: A map message.

    Output: A dictionary containing all roads in the map message
            as values, whose keys are their ids.
    """
    roads = dict()
    for road_msg in map_msg.road:
        roads[str(road_msg.id)] = road_msg
    return roads


def points_dist(x1: float, y1: float, x2: float, y2: float):
    """
    Description: Given two points, calculate their distance

    Input: 4 float numbers: the x, y coordinate of point 1, 
           the x and y coordinate of point 2

    Output: The distance between point 1 and point 2
    """
    p1 = np.array((x1, y1))
    p2 = np.array((x2, y2))
    return np.linalg.norm(p1 - p2)


def dist_to_lane_center(x: float, y: float, lane_msg):
    """
    Given the x and y coordinate of current position,
    calculate the min distance between the current position
    and any of the center point of the argument lane

    Input: The x, y coordinate of the ego car, and the 
           lane_msg to calculate the distance

    Output: The shortest distance between the adc point and the 
            center points
    """
    min_dist = sys.maxsize

    for s in lane_msg.central_curve.segment:
        for p in s.line_segment.point:
            dist = points_dist(x, y, p.x, p.y)

            if min_dist > dist:
                min_dist = dist

    return min_dist


def get_lane_id(x: float, y: float, lanes: dict):
    '''
    Return the lane id of the x and y coordinate. None if
    the point does not resides in any lane from the map.
    '''
    residing_ids = []

    for lane_id in lanes.keys():
        if is_point_in_lane(x, y, lanes[lane_id]):
            residing_ids.append(lane_id)

    return residing_ids


def efficient_fetch_lane(x, y, current_lane, lanes, priority_lanes=None):
    """
    Description: Fetch current lane id efficiently (avoids complete scan
    of all lanes when possible)

    Input: The x, y value of the ego car coordinate,
           the lane id of the most recent lane, the cached lane dict,
           and priority lanes, which are the most likely lanes

    Output: The lane id of the lane that the ego car resides
    """
    residing_lanes = []

    if priority_lanes is None:
        priority_lanes = []

    if is_rescan_lane(x, y, current_lane, lanes):
        adjacent_lanes = get_adjacent_lanes(current_lane, lanes)
        priority_lanes += adjacent_lanes
        for priority_lane in priority_lanes:
            if is_point_in_lane(x, y, lanes[priority_lane]):
                residing_lanes.append(priority_lane) 
        if len(residing_lanes) > 0:
            return residing_lanes
        # Have to do a complete rescan since the ego car
        # is not in adjacent lanes
        return get_lane_id(x, y, lanes)
    # Either we get a lane id from all lanes or
    # None because the vehicle is off-road
    return [current_lane]


def is_rescan_lane(current_x, current_y, current_lane, lanes):
    '''
    Check if perform complete scan of the lanes based on
    last oberved lane and current coordinate
    '''
    if current_lane is None:
        return True
    at_different_lane = not is_point_in_lane(
        current_x, current_y, lanes[current_lane])
    return at_different_lane


def get_adjacent_lanes(from_lane_id, lanes):
    '''
    Given a lane id, fetches all the lane ids adjacent to it
    '''
    adjacent_lanes = []

    if from_lane_id is None:
        return adjacent_lanes

    from_lane_msg = lanes[from_lane_id]

    for s_id in from_lane_msg.successor_id:
        adjacent_lanes.append(str(s_id.id))

    for l_id in from_lane_msg.left_neighbor_forward_lane_id:
        adjacent_lanes.append(str(l_id.id))

    for r_id in from_lane_msg.right_neighbor_forward_lane_id:
        adjacent_lanes.append(str(r_id.id))

    return adjacent_lanes


def get_lane_boundary_points(boundary):
    '''
    Given a lane boundary (left/right), return a list of x, y 
    coordinates of all points in the boundary 
    '''
    boundary_points = []
    for segment in boundary.curve.segment:
        for segment_point in segment.line_segment.point:
            boundary_points.append((segment_point.x, segment_point.y))
    return boundary_points


def get_road_boundary_points(road_msg):
    '''
    Given a road boundary, return the points composing
    its outer polygon
    '''
    boundary_points = []
    for section in road_msg.section:
        for edge in section.boundary.outer_polygon.edge:
            for segment in edge.curve.segment:
                for point in segment.line_segment.point:
                    boundary_points.append((point.x, point.y))
    return boundary_points


def construct_lane_polygon(lane_msg):
    '''
    Construct the lane polygon based on their boundaries
    '''
    left_points = get_lane_boundary_points(lane_msg.left_boundary)
    right_points = get_lane_boundary_points(lane_msg.right_boundary)
    right_points.reverse()
    all_points = left_points + right_points
    return Polygon(all_points)


def fetch_residing_lanes(adc_polygon: Polygon, lanes: dict):
    """
    Description: Return all lanes that the adc polygon resides as a set

    Input:
        adc_polygon: A Shapely Polygon describing the adc boundaries.
        lanes: The cached lanes of the map.

    Output:
        A set containing all the lanes (id) the adc polygon resides.
    """
    result_set = set()
    for (lane_id, lane_msg) in lanes.items():
        lane_linestring = construct_lane_linestring(lane_msg)
        if adc_polygon.intersects(lane_linestring):
            result_set.add(lane_id)
    return result_set


def construct_lane_linestring(lane_msg):
    """
    Description: Construct the polygon boundary line string from lane message.

    Input: A lane message.

    Output: A shapely LineString containing the lane boundary points.
    """
    left_points = get_lane_boundary_points(lane_msg.left_boundary)
    right_points = get_lane_boundary_points(lane_msg.right_boundary)
    right_points.reverse()
    right_points.append(list(left_points[0]))
    return LineString(left_points + right_points)


def is_point_in_lane(x, y, lane_msg):
    '''
    Given the x and y coordinate of the ego car, check 
    if the car is within the argument lane
    '''
    lane_polygon = construct_lane_polygon(lane_msg)
    current_point = Point(x, y)
    return current_point.within(lane_polygon)


def count_lane_num(map_msg) -> int:
    """
    Description: 
        Count the total number of lanes in a map.
    Input:
        A map protobuf message.
    Output:
        The total number of lanes.
    """
    lanes = cache_lanes(map_msg)
    return len(lanes)


def count_lane_length(lanes) -> float:
    """
    Description:
        Count the total length of the argument lanes.
    Input:
        A collection of lane_id and lane message pairs.
    Output:
        The length of all the lanes in meters.
    """
    total_length = 0
    for lane_id in lanes:
        total_length += lanes[lane_id].length
    return total_length
