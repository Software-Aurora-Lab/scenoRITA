'''
extract the initial states from a record file
initial state extracted:
    perception
    routing
'''

import time
import sys
import os
import argparse

from automation.record.read_record import read_by_channel
from automation.record.write_record import write_record

from collections import defaultdict

DEFAULT_CHANNELS = {
    '/apollo/routing_response',
    '/apollo/perception/obstacles',
}

def extract_first_messages(messages, channels=DEFAULT_CHANNELS):
    '''
    Get the first messages of each message type

    Args:
        messages ([message]): The original messages to be extracted in a list.
        channels ({str}, optional): All messages that went through the channel in
            the channels argument will be extracted

    Returns:
        [messages]: A list of extracted messages
    '''

    captured = defaultdict(lambda: False)

    selected_messages = []

    for channel_name, msg, parsed_msg, datatype, timestamp in messages:
        if captured[channel_name] is False:
            print(channel_name)
            selected_messages.append((channel_name, msg, parsed_msg, datatype, timestamp))
            captured[channel_name] = True

    return selected_messages



if __name__ == '__main__':
    # define the required arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_path', help='The path of the input bag to extract', type=str)
    parser.add_argument('-o', '--output_path', help='The path of the extracted bag to store', type=str)
    args = parser.parse_args()

    # process the input &o utput directory
    in_filename = os.path.basename(args.input_path)
    in_filedir = os.path.dirname(args.input_path)+'/'
    out_filename = os.path.basename(args.output_path)
    out_filedir = os.path.dirname(args.output_path)+'/'

    # read all messages from bag
    messages = read_by_channel(in_filename, in_filedir, DEFAULT_CHANNELS)
    # retain only the first messages in each channel
    extracted_messages = extract_first_messages(messages)
    # print summary
    print(f'{len(extracted_messages)} messages are extracted as the first message')
    # store the extracted messages in a new bag
    write_record(extracted_messages, record_name=out_filename, record_dir=out_filedir)
    