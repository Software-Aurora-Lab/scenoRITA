# provides functions that help reading a record

from collections import defaultdict
import time
import os

from cyber.python.cyber_py3 import record

from automation.auxiliary.record.message_types import type_to_class

DEFAULT_CHANNELS = {
    '/apollo/canbus/chassis',
    '/apollo/localization/pose',
    '/apollo/perception/obstacles',
    '/apollo/perception/traffic_light',
    '/apollo/planning',
    '/apollo/prediction',
    '/apollo/routing_response',
    '/apollo/storytelling'
}


def read_record(record_name, record_dir):
    messages = read_by_channel(record_name=record_name, record_dir=record_dir)

    return messages


def read_by_path(record_path):
    # get the base dir and file name from record_path
    dir_name = os.path.dirname(record_path) + '/'
    file_name = os.path.basename(record_path)

    # return the messages
    return read_record(file_name, dir_name)


def print_msg_num(msg_dict: defaultdict):
    print(f"{'channel_name':<30}\t{'message count':>10}")
    for channel_name in msg_dict.keys():
        print(
            f'{channel_name:<30}\t{msg_dict[channel_name]:>10}')


def read_by_channel(record_name, record_dir, channels=DEFAULT_CHANNELS, verbose=False):
    # open the file to read
    freader = record.RecordReader(record_dir + record_name)
    time.sleep(1)

    # iterate through the messages in the record
    # store them in the messages collection
    messages = []

    empty_messages = defaultdict(int)
    parsed_messages = defaultdict(int)

    for channel_name, msg, datatype, timestamp in freader.read_messages():
        if channel_name in channels:
            try:
                parsed_msg = type_to_class[datatype]()
                parsed_msg.ParseFromString(msg)
                messages.append(
                    (channel_name, msg, parsed_msg, datatype, timestamp))
                parsed_messages[channel_name] += 1
            except:
                empty_messages[channel_name] += 1

    assert(len(empty_messages) == 0)     # make sure the recording is finished
    if verbose:
        if len(parsed_messages) > 0:
            print(f'Successfully parsed messages:')
            print_msg_num(parsed_messages)
        else:
            print('All messages failed to parse')

    return messages
