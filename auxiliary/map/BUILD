load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "map_tools",
    srcs = ["map_tools.py"],
    deps = [
        "//modules/map/proto:map_py_pb2",
    ],
)