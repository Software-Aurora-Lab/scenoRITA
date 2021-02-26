load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "message_types",
    srcs = ["message_types.py"],
    deps = [
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/localization/proto:imu_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/perception/proto:traffic_light_detection_py_pb2",
        "//modules/prediction/proto:prediction_obstacle_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/routing/proto:routing_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/storytelling/proto:story_py_pb2",
        "//cyber/python/cyber_py3:record",
    ],
)

py_library(
    name = "read_record",
    srcs = ["read_record.py"],
    deps = [
        "//automation/auxiliary/record:message_types",
        "//cyber/python/cyber_py3:record",
    ],
)

py_library(
    name = "write_record",
    srcs = ["write_record.py"],
    deps = [
        "//automation/auxiliary/record:message_types",
        "//automation/auxiliary/record:read_record",
        "//automation/auxiliary/record:write_txt",  
        "//cyber/python/cyber_py3:record",
    ],
)

py_library(
    name = "write_txt",
    srcs = ["write_txt.py"],
)

py_library(
    name = "write_json",
    srcs = ["write_json.py"],
)