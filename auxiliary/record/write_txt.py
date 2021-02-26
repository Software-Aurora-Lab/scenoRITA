# provide functions that help writing protobuf messages
# into a txt file

from google.protobuf import text_format

# writes txt_format protobuf messages into file
def write_to_file(label='', content='', f=None):
    '''
    Write content to file

    Args:
        label (str): The type of content to write
        content (str): The content to write
        f (stream): The stream that the content writes to

    Returns:
        None
    '''
    if f is None:
        print('cannot write to file of None type')
        return
    if label == 'message content':
        f.write(content + '\n\n')
    else:
        f.write(f'{label} = {content}\n')

def save_messages_as_txt(messages, output_path: str):
    '''
    Save messages to a txt file
    
    Args:
        messages ([message]): A list of messages to be parsed and stored as text
        output_path (str): the path of the stored txt file

    Returns:
        None
    '''
    # specify the file storing message content
    f = open(output_path, 'a')

    # iterate through the messages to be stored
    # parse them based on their types and write to a txt file
    for channel_name, _, parsed_msg, data_type, timestamp in messages:
        write_to_file('channel name', channel_name, f)
        write_to_file('time stamp', timestamp, f)
        write_to_file('data type', data_type, f)
        write_to_file('message content', text_format.MessageToString(parsed_msg), f)

    f.close()
