import argparse
import subprocess

VALID_OPTIONS = ['start', 'stop']

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'operation',
        help='[start/stop] prediction, planning, and routing module',
        type=str
    )
    args = parser.parse_args()
    return args

def main():
    args = get_args()
    operation = args.operation
    
    # Avoid unspecified operation
    if operation not in VALID_OPTIONS:
        print(f'Invalid operation. Operation must be one of: {VALID_OPTIONS}')
    
    if operation == 'start':
        subprocess.run('bash /apollo/automation/modules/start_modules.sh', shell=True)
    elif operation == 'stop':
        subprocess.run('bash /apollo/automation/modules/stop_modules.sh', shell=True)

if __name__ == '__main__':
    main()
