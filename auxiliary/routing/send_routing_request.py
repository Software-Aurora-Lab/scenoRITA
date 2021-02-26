#!/usr/bin/env python3

# send the routing message specified in the argument
# to the channel /apollo/routing request

import sys
import time
import argparse

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto.routing_pb2 import RoutingRequest


class CyberShutdown(Exception):
    pass


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'init_x',
        help='the initial x position',
        type=float
    )
    parser.add_argument(
        'init_y',
        help='the initial y position',
        type=float
    )
    parser.add_argument(
        'dest_x',
        help='the destination x position',
        type=float
    )
    parser.add_argument(
        'dest_y',
        help='the destination y position',
        type=float
    )
    args = parser.parse_args()
    return args


def create_node(node_name='automation_routing_request_node'):
    # generate node
    return cyber.Node(node_name)


def create_writer(
        node,
        channel_name='/apollo/routing_request',
        message_type=RoutingRequest):
    # generate writer
    return node.create_writer('/apollo/routing_request', message_type)


def process_message(writer, init_x, init_y, dest_x, dest_y, sequence_num=0, verbose=False):
    request = RoutingRequest()

    # define header
    request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    request.header.module_name = "automation routing"
    request.header.sequence_num = sequence_num

    # define way points (start and end)
    start_waypoint = request.waypoint.add()
    start_waypoint.pose.x = init_x
    start_waypoint.pose.y = init_y

    end_waypoint = request.waypoint.add()
    end_waypoint.pose.x = dest_x
    end_waypoint.pose.y = dest_y

    if verbose:
        print(
            f'routing request from: {init_x}, {init_y} to: {dest_x}, {dest_y} sent!')

    if not cyber.is_shutdown():
        writer.write(request)
    else:
        raise CyberShutdown

    cyber.shutdown()


def request_routing(init_x, init_y, dest_x, dest_y, verbose=False):
    cyber.init()

    if not cyber.ok():
        print('cyber error')
        sys.exit(1)

    # create node and writer
    node = create_node()
    writer = create_writer(node)

    time.sleep(2.0)
    # process message
    process_message(writer, init_x, init_y, dest_x,
                    dest_y, sequence_num=0, verbose=False)


def main():
    # capture arguments
    args = get_args()
    # store init and dest points
    init_x = args.init_x
    init_y = args.init_y
    dest_x = args.dest_x
    dest_y = args.dest_y

    request_routing(init_x, init_y, dest_x, dest_y)


if __name__ == '__main__':
    main()
