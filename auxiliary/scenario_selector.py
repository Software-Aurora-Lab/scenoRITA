import re
import random
from enum import Enum
from random import shuffle
from dataclasses import dataclass
from collections import defaultdict

import pandas as pd

random.seed(0)


class ScenarioSelectionLimitType(Enum):
    PERCENTAGE = 0
    COUNT = 1


@dataclass
class ScenarioCounter:
    collision: int
    speeding: int
    uslc: int       # unsafe lane change
    fastAccl: int   # fast acceleration
    hardBrake: int

    def __hash__(self):
        return hash(
            (self.collision, self.speeding, self.uslc, self.fastAccl, self.hardBrake)
        )


@dataclass
class Scenario:
    generationId: int
    scenarioId: int
    counter: ScenarioCounter
    totalVio: int   # total violation

    def __hash__(self):
        return hash((self.generationId, self.scenarioId))

    def to_dict(self):
        return {
            'record_name': f"Generation{self.generationId}_Scenario{self.scenarioId}",
            'c_counter': self.counter.collision,
            'speeding_counter': self.counter.speeding,
            'uslc_counter': self.counter.uslc,
            'fastAccl_counter': self.counter.fastAccl,
            'hardBrake_counter': self.counter.hardBrake,
            'Total Vio': self.totalVio
        }


def parse_record_name(record_name: str) -> (int, int):
    """
    Giving a record name, parse its generation id and scenario id
    """
    match = re.match(r"^Generation(\d+)_Scenario(\d+)$", record_name)
    assert match is not None
    return tuple(int(x) for x in match.groups())


def parse_spreadsheet(filename: str) -> [Scenario]:
    """
    Giving a spreadsheet containing scenario data, parse the data and produce
    a list of 'Scenario'
    """
    df = pd.read_csv(filename)
    cols = ['record_name', 'c_counter', 'speeding_counter', 'uslc_counter',
            'fastAccl_counter', 'hardBrake_counter', 'Total Vio']
    counter_cols = ['c_counter', 'speeding_counter',
                    'uslc_counter', 'fastAccl_counter', 'hardBrake_counter']
    scenarios = list()
    for index, data in df[cols].iterrows():
        generationId, scenarioId = parse_record_name(data['record_name'])
        counter = ScenarioCounter(*[data[x] for x in counter_cols])
        scenario = Scenario(generationId, scenarioId,
                            counter, data['Total Vio'])
        scenarios.append(scenario)
    return scenarios


def select_scenarios(S: [Scenario], limit: float, limitType: ScenarioSelectionLimitType):
    """
    Given a list of 'Scenario', select up to limit number of scenarios.
    limit can be a percentage or specific number, specified by limitType
    """
    assert type(limitType) == ScenarioSelectionLimitType
    if limitType == ScenarioSelectionLimitType.PERCENTAGE:
        limit = int(len(S) * limit / 100 + 0.5)
    else:
        limit = int(limit)

    scenarios = list(S)  # make a copy
    shuffle(scenarios)  # shuffle the copy

    selected = defaultdict(lambda: set())
    scenario_dict = defaultdict(lambda: list())

    selected_combination = defaultdict(lambda: 0)
    generationIds = set()
    for s in scenarios:
        generationIds.add(s.generationId)
        if selected_combination[s.counter] == 0:
            selected[s.generationId].add(s)
            selected_combination[s.counter] += 1
        else:
            scenario_dict[s.generationId].append(s)

    while sum(len(selected[x]) for x in selected) < limit:
        # generations with unselected scenario
        gids = [x for x in generationIds if len(scenario_dict[x]) > 0]
        # sort scenario from the least selected generation
        gids.sort(key=lambda x: len(selected[x]))
        gid = gids[0]
        scenario_dict[gid].sort(key=lambda x: (
            x.totalVio, selected_combination[x.counter]))

        _selected = scenario_dict[gid].pop(0)
        selected_combination[_selected.counter] += 1
        selected[gid].add(_selected)

    result = list()
    for gid in selected:
        for s in selected[gid]:
            result.append(s)
    result.sort(key=lambda x: (x.generationId, x.scenarioId))
    return result


def print_selection_statistic(scenarios: [Scenario]):
    """
    Prints statistical information of selected scenarios, 
    e.g. number of scenarios from each generation, etc.
    """
    counter = defaultdict(lambda: defaultdict(lambda: 0))
    for scenario in scenarios:
        counter['generation'][scenario.generationId] += 1
        counter['totalVio'][scenario.totalVio] += 1
        if scenario.counter.collision > 0:
            counter['counter']['collision'] += 1
        if scenario.counter.speeding > 0:
            counter['counter']['speeding'] += 1
        if scenario.counter.uslc > 0:
            counter['counter']['uslc'] += 1
        if scenario.counter.fastAccl > 0:
            counter['counter']['fastAccl'] += 1
        if scenario.counter.hardBrake > 0:
            counter['counter']['hardBrake'] += 1

    print('=' * 50)
    print('Statistics')
    print("  Generation count")
    for gid in counter['generation']:
        print(f"\t{counter['generation'][gid]}\tGeneration {gid}")

    print("  Violation Type Count")
    for vtype in counter['counter']:
        print(f"\t{counter['counter'][vtype]}\t{vtype}")


if __name__ == '__main__':
    input_filename = 'data.csv'
    output_filename = 'selected.csv'

    scenarios = parse_spreadsheet(input_filename)
    selected = select_scenarios(
        scenarios, 20, ScenarioSelectionLimitType.PERCENTAGE)
    df = pd.DataFrame([x.to_dict() for x in selected])
    df.to_csv(output_filename)
    print_selection_statistic(selected)
