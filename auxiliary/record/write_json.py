import os
import json
from google.protobuf import json_format


def save_messages_as_json(messages, output_path: str, verbose=False):
    if os.path.exists(output_path):
        os.remove(output_path)

    f = open(output_path, 'a')
    if f is None:
        print(f"Error, cannot write to file by path {output_path}")
        return

    # For progress checking when running the script
    progress_gap = int(len(messages) / 10)
    i = 0
    next_i = 0

    for _, _, parsed_msg, _, _ in messages:
        if verbose and i == next_i:
            print(f'{int(i / progress_gap * 10)}%')
            next_i = i + progress_gap

        data = json_format.MessageToDict(
            parsed_msg,
            preserving_proto_field_name=True
        )
        json.dump(data, f, indent=2)
        i += 1

    f.close()
