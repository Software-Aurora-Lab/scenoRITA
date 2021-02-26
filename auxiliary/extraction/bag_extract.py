# provided with the input and output path, this script
# captures the messages in the input bag and output them
# in text format to the output path

import time
import sys
import os
import argparse

# includes protobuf types such as planning and perception
from automation.record.read_record import read_by_path
from automation.record.write_txt import save_messages_as_txt
from automation.record.write_json import save_messages_as_json

VALID_OUTPUT_TYPES = ['json', 'txt']

if __name__ == '__main__':
    # define the required arguments
    parser = argparse.ArgumentParser()
    # requires the input (the original bag) and
    # output path (extracted messages as txt/json)
    parser.add_argument(
        'output_type',
        help="[json|txt] output",
        type=str
    )
    parser.add_argument(
        "input_path",
        help="The path of the input bag/record to extract",
        type=str)
    parser.add_argument(
        "output_path",
        help="The path of the extracted messages to store",
        type=str
    )
    args = parser.parse_args()

    output_type = args.output_type
    if output_type not in VALID_OUTPUT_TYPES:
        print(
            f'''Invalid output type {output_type}. 
            Valid output types: {VALID_OUTPUT_TYPES}''')
        exit()

    bag_location = args.input_path
    output_path = args.output_path

    # get all messages from bag and save them as txt
    messages = read_by_path(bag_location)
    if output_type == 'txt':
        save_messages_as_txt(messages, output_path)
    elif output_type == 'json':
        save_messages_as_json(messages, output_path, verbose=True)
