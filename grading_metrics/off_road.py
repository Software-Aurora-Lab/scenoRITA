import argparse
from os import walk
from automation.auxiliary.map import map_tools
from automation.auxiliary.record.read_record import read_by_path


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'record_path',
        help='path to the record file to test',
        type=str
    )
    args = parser.parse_args()
    return args


def report_violation(x, y, timestamp):
    print('Off-road violation:')
    print(f'x: {x}, y: {y}, time: {timestamp}')


def walk_messages(record_path):
    '''
    Loop through localization messages in record, 
    find the messages that are off-road.
    '''
    messages = read_by_path(record_path)

    map_msg = map_tools.load_mapbin()
    roads = map_tools.cache_roads(map_msg)

    current_road = None
    for channel_name, _, parsed_msg, _, timestamp in messages:
        if channel_name == '/apollo/localization/pose':
            current_pos = parsed_msg.pose.position
            current_x = current_pos.x
            current_y = current_pos.y

            current_road = map_tools.efficient_fetch_road(
                current_x, current_y, current_road, roads)

            if current_road is None:
                report_violation(current_x, current_y, timestamp)


def main():
    args = get_args()
    record_path = args.record_path
    walk_messages(record_path)


if __name__ == '__main__':
    main()
