load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "bag_extract",
    srcs = ["bag_extract.py"],
    deps = [
        "//automation/record:message_types",
        "//automation/record:read_record",
        "//automation/record:write_txt",
        "//automation/record:write_json",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
    ],
)

py_binary(
    name = "extract_initial_states",
    srcs = ["extract_initial_states.py"],
    deps = [
        "//automation/record:message_types",
        "//automation/record:read_record",
        "//automation/record:write_record",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
    ]
)