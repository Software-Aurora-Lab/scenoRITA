load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "run_automation",
    srcs = ["run_automation.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
        "//automation/auxiliary/record:read_record",
        "//automation/auxiliary/routing:send_routing_request",
        "//automation/grading_metrics:acceleration",
        "//automation/grading_metrics:collision",
        "//automation/grading_metrics:speeding",
        "//automation/grading_metrics:off_road",
        "//automation/auxiliary/map:map_tools",
    ],
)
