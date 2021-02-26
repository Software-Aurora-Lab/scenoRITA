load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "modules",
    srcs = ["modules.py"]
)

py_binary(
    name = "perception_generator",
    srcs = ["perception_generator.py"],
    deps = [
        "//modules/tools/perception:replay_perception"
    ]
)