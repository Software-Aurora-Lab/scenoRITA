# includes protobuf messages, channels, and their mappings

import time
import sys
import os

# sys.path.insert(1, '/apollo/py_proto/modules/canbus/proto')
import modules.canbus.proto.chassis_pb2 as chassis_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/localization/proto')
import modules.localization.proto.localization_pb2 as localization_pb2
import modules.localization.proto.imu_pb2 as imu_pb2
import modules.localization.proto.gps_pb2 as gps_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/perception/proto')
import modules.perception.proto.traffic_light_detection_pb2 as traffic_light_detection_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/prediction/proto')
import modules.prediction.proto.prediction_obstacle_pb2 as prediction_obstacle_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/routing/proto')
import modules.routing.proto.routing_pb2 as routing_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/planning/proto')
import modules.planning.proto.planning_pb2 as planning_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/perception/proto')
import modules.perception.proto.perception_obstacle_pb2 as perception_obstacle_pb2
# sys.path.insert(1, '/apollo/py_proto/modules/storytelling/proto')
import modules.storytelling.proto.story_pb2 as story_pb2

# define the mapping from message data type to class name
# supports messages from only the following:
# 'apollo.canbus.Chassis', 'apollo.localization.LocalizationEstimate',
# 'apollo.perception.PerceptionObstacles', 'apollo.perception.TrafficLightDetection',
# 'apollo.planning.ADCTrajectory', 'apollo.prediction.PredictionObstacles'
# 'apollo.routing.RoutingResponse', 'apollo.storytelling.Stories'
type_to_class = {
    'apollo.canbus.Chassis': chassis_pb2.Chassis,
    'apollo.localization.LocalizationEstimate': localization_pb2.LocalizationEstimate,
    'apollo.perception.PerceptionObstacles': perception_obstacle_pb2.PerceptionObstacles,
    'apollo.perception.TrafficLightDetection': traffic_light_detection_pb2.TrafficLightDetection,
    'apollo.planning.ADCTrajectory': planning_pb2.ADCTrajectory,
    'apollo.prediction.PredictionObstacles': prediction_obstacle_pb2.PredictionObstacles,
    'apollo.routing.RoutingResponse': routing_pb2.RoutingResponse,
    'apollo.storytelling.Stories': story_pb2.Stories
}

class_to_channel = {
    chassis_pb2.Chassis: '/apollo/canbus/chassis',
    localization_pb2.LocalizationEstimate: '/apollo/localization/pose',
    perception_obstacle_pb2.PerceptionObstacle: '/apollo/perception/obstacles',
    traffic_light_detection_pb2.TrafficLightDetection: '/apollo/perception/traffic_light',
    planning_pb2.ADCTrajectory: '/apollo/planning',
    prediction_obstacle_pb2.PredictionObstacles: '/apollo/prediction',
    routing_pb2.RoutingResponse: '/apollo/routing_response',
    story_pb2.Stories: '/apollo/storytelling'
}

channel_to_class = {
    '/apollo/canbus/chassis': chassis_pb2.Chassis,
    '/apollo/localization/pose': localization_pb2.LocalizationEstimate,
    '/apollo/perception/obstacles': perception_obstacle_pb2.PerceptionObstacle,
    '/apollo/perception/traffic_light': traffic_light_detection_pb2.TrafficLightDetection,
    '/apollo/planning': planning_pb2.ADCTrajectory,
    '/apollo/prediction': prediction_obstacle_pb2.PredictionObstacles,
    '/apollo/routing_response': routing_pb2.RoutingResponse,
    '/apollo/storytelling': story_pb2.Stories
}
