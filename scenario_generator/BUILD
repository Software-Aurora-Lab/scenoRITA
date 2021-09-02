load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
	name = "scenoRITA_mut",
	srcs = ["scenoRITA_mut.py"],
	deps = [
		"//automation/auxiliary/map:map_tools",
	],
)

py_binary(
   name = "scenoRITA_immut",
   srcs = ["scenoRITA_immut.py"],
   deps = [
       "//automation/auxiliary/map:map_tools",
   ],
)

py_binary(
   name = "scenoRITA_random",
   srcs = ["scenoRITA_random.py"],
   deps = [
       "//automation/auxiliary/map:map_tools",
   ],
)
