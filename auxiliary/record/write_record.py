# provides functions that help writing a record
import os
from os import path

from google.protobuf.descriptor_pb2 import FileDescriptorProto
from cyber.python.cyber_py3 import record
from automation.auxiliary.record.message_types import type_to_class

def write_record(messages, record_name='sunnyvale.record', record_dir='/apollo/mutated_records/'):
    """
    Write the messages params to the specified record
        record_name: the name of the record to be written
        record_dir: the absolute path of the record to be written
        messages: messages that will be written to the record
    """
    
    # remove the old file if existed
    if path.exists(record_dir+record_name):
        os.remove(record_dir+record_name)

    fwriter = record.RecordWriter()
    fwriter.set_size_fileseg(0)
    fwriter.set_intervaltime_fileseg(0)

    fwriter.open(record_dir+record_name)

    for channel_name, msg, parsed_msg, datatype, timestamp in messages:
        # capture message information
        if datatype in type_to_class.keys():
            msg_content = parsed_msg
            # write the  message to record
            file_desc = msg_content.DESCRIPTOR.file
            proto = FileDescriptorProto()
            file_desc.CopyToProto(proto)
            proto.name = file_desc.name
            desc_str = proto.SerializeToString()
            fwriter.write_channel(channel_name, msg_content.DESCRIPTOR.full_name, desc_str)
            fwriter.write_message(channel_name, msg_content.SerializeToString(), timestamp)
            
if __name__ == '__main__':
    messages = read_record('sunnyvale_12.bag', '/apollo/selected_scenarios/')
    write_record(messages)
