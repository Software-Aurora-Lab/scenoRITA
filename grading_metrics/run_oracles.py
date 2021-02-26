import csv
import time
import argparse
from multiprocessing import Process, Manager

from automation.grading_metrics import collision, acceleration, speeding

RECORD_DIR = '/apollo/automation/results/recorded_scenarios/'
CSV_PATH = '/apollo/automation/oracles/file_names.txt'


def generate_record_name(generation_num: int, scenario_num: int):
    return f'{generation_num}_{scenario_num}.00000'


def run_oracles(record_path):
    processes = []
    manager = Manager()
    return_dict = manager.dict()

    processes.append(
        Process(target=acceleration.walk_messages,
                args=(record_path, 4),
                kwargs={'return_dict': return_dict}))
    processes.append(
        Process(target=acceleration.walk_messages,
                args=(record_path, -4),
                kwargs={'return_dict': return_dict}))
    processes.append(
        Process(target=collision.walk_messages,
                args=(record_path,),
                kwargs={'return_dict': return_dict}))
    processes.append(
        Process(target=speeding.walk_messages,
                args=(record_path,),
                kwargs={'return_dict': return_dict}))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

    accl = return_dict['accl']
    hardbreak = return_dict['hardbreak']
    min_dist = return_dict['min_dist']
    min_speed = return_dict['min_speed']
    traveled_lanes = return_dict['traveled_lanes']
    boundary_dist = return_dict['boundary_dist']

    return min_dist, traveled_lanes, min_speed, boundary_dist, accl, hardbreak


def main():
    paths = list()
    print('Reading the record description...')
    with open(CSV_PATH) as file:
        reader = csv.reader(file)
        for row in reader:
            record_name = generate_record_name(row[0], row[1])
            paths.append(RECORD_DIR + record_name)

    print('Running the oracles...')
    with open('/apollo/automation/oracles/oracle_output.csv', mode='w') as output_file:
        output_writer = csv.writer(
            output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        output_writer.writerow(
            ['record_name', 'Speeding', 'FastAccel', 'HardBraking'])

        record_count = len(paths)
        for i, oracle_path in enumerate(paths):
            start_time = time.time()
            # if i % int(record_count / 10) == 0:
            #     print(f'{int(i/record_count)}% finished')
            print(
                f'Running the {i+1}/{record_count} record ({oracle_path})', end=',\t')
            min_dist, traveled_lanes, min_speed, boundary_dist, accl, hardbreak = run_oracles(
                oracle_path)
            # output_writer.writerow([oracle_path, min_speed, accl, hardbreak])
            end_time = time.time()
            output_writer.writerow([end_time, oracle_path])
            print(f'finished in {end_time-start_time} seconds.')


if __name__ == '__main__':
    init_time = time.time()
    main()
    end_time = time.time()
    print(f'run_oracles finished in {end_time-init_time} seconds')
