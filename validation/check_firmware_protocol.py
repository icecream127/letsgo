import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'lidar_ws/src/ros2_to_stm32'))
from ros2_to_stm32.motor_protocol import (
    MODE_STATUS, StatusParser, TIMED_ACTIONS, encode_key, parse_durations, validate_action,
)


HARNESS = r'''
#include <iostream>
#include "remote.h"
#include "action.h"
int main() {
    RemoteProtocol parser;
    unsigned int count;
    std::cin >> count;
    for (unsigned int index = 0; index < count; ++index) {
        unsigned int raw;
        std::cin >> raw;
        uint8_t key = 0;
        if (parser.FeedByte(static_cast<uint8_t>(raw), key))
            std::cout << "key " << static_cast<unsigned int>(key) << '\n';
    }
    for (unsigned int mode_key = 9; mode_key <= 18; ++mode_key) {
        ActionMode mode;
        if (!Action_DecodeMode(mode_key, mode)) return 2;
        for (unsigned int key = 1; key <= 23; ++key) {
            const auto action = Action_DecodeKey(mode, key);
            ActionRunner runner;
            const bool started = runner.Start(action, mode, 0);
            const bool timed = started && runner.IsBusy();
            unsigned int elapsed = 0;
            ActionOutput output;
            while (runner.IsBusy() && elapsed < 60000) {
                elapsed += 5;
                runner.Update(elapsed, mode, output);
            }
            std::cout << "action " << mode_key << ' ' << key << ' '
                      << (action != ImportedAction::None) << ' ' << timed << ' '
                      << elapsed << ' ' << runner.IsBusy() << '\n';
        }
    }
    const RemoteStatus modes[] = {RemoteStatus::NormalMode, RemoteStatus::LittleSlope,
        RemoteStatus::BridgeB, RemoteStatus::BridgeA, RemoteStatus::BigSlope,
        RemoteStatus::Crawl, RemoteStatus::Bunker, RemoteStatus::Pole,
        RemoteStatus::HighWall, RemoteStatus::StepMode};
    for (unsigned int index = 0; index < 10; ++index) {
        uint8_t frame[ROBOT_STATUS_SIZE];
        RobotStatus_Encode(frame, static_cast<uint8_t>(modes[index]), 0, 0, 0);
        std::cout << "status " << index + 9;
        for (const auto byte : frame) std::cout << ' ' << static_cast<unsigned int>(byte);
        std::cout << '\n';
    }
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('firmware', type=Path)
    parser.add_argument('--output', type=Path)
    arguments = parser.parse_args()
    firmware = arguments.firmware.resolve()
    user = firmware / 'User'
    remote_path = user / 'Remote/Uart/remote.cpp'
    source = remote_path.read_text(encoding='utf-8')
    end = source.index('\nnamespace', source.index('bool RemoteProtocol::FeedByte'))
    protocol = source[:end].replace('#include "usart.h"', '').replace('#include "tim.h"', '')
    invalid = b'noiseS2E\r\nS18E\r\n' + bytes([0x53, 0, 0x45, 0x53, 24, 0x45, 0x53, 2, 0])
    stream = invalid + b''.join(encode_key(key) for key in range(1, 24))
    stream += bytes([0x53, 0x53, 2, 0x45])
    with tempfile.TemporaryDirectory(prefix='letsgo_firmware_check_') as temporary:
        work = Path(temporary)
        (work / 'protocol.cpp').write_text(protocol, encoding='utf-8')
        (work / 'main.cpp').write_text(HARNESS, encoding='utf-8')
        include_dirs = sorted({path.parent for path in user.rglob('*.h')})
        command = ['g++', '-std=c++17', '-O0', '-o', str(work / 'check'),
                   str(work / 'main.cpp'), str(work / 'protocol.cpp'),
                   str(user / 'Motion/Action/action_keys.cpp'), str(user / 'Motion/Action/action.cpp')]
        for directory in include_dirs:
            command.extend(['-I', str(directory)])
        subprocess.run(command, check=True, capture_output=True, text=True)
        result = subprocess.run([str(work / 'check')],
                                input=f'{len(stream)} ' + ' '.join(map(str, stream)),
                                check=True, capture_output=True, text=True)
    rows = [line.split() for line in result.stdout.splitlines()]
    keys = [int(row[1]) for row in rows if row[0] == 'key']
    assert keys == list(range(1, 24)) + [2], keys
    config = yaml.safe_load((PROJECT / 'lidar_ws/src/ros2_to_stm32/config/control.yaml').read_text())
    durations = parse_durations(config['motor_serial']['ros__parameters']['action_durations'])
    margin = config['motor_serial']['ros__parameters']['action_margin']
    observed = []
    for row in rows:
        if row[0] == 'status':
            mode_key = int(row[1])
            statuses = StatusParser().feed(bytes(map(int, row[2:])))
            assert statuses == [dict(mode=MODE_STATUS[mode_key], fault=0, motor=0, detail=0)]
        if row[0] != 'action':
            continue
        mode_key, key, supported, timed, elapsed, busy = map(int, row[1:])
        try:
            validate_action(mode_key, key)
            allowed = True
        except ValueError:
            allowed = False
        assert allowed == bool(supported), row
        assert bool(timed) == ((mode_key, key) in TIMED_ACTIONS), row
        assert not busy, row
        if timed:
            assert elapsed / 1000 <= durations[mode_key, key] + margin, row
            observed.append(dict(mode=mode_key, key=key, simulated_ms=elapsed,
                                 host_guard_s=durations[mode_key, key] + margin))
    reference_files = [remote_path, user / 'Remote/Uart/robot_status_protocol.h',
                       user / 'Motion/Action/action.cpp', user / 'Motion/Action/action_keys.cpp',
                       user / 'APP/Remote/app_remote.cpp']
    report = dict(command_parser='compiled original Reset/FeedByte without hardware section',
                  command_vectors='23 valid keys, ASCII/invalid frames rejected, resync passed',
                  action_combinations=230, status_modes=10, timed_actions=observed,
                  simulation='original ActionRunner, ideal 5ms updates; not physical completion or UART ACK',
                  firmware={str(path.relative_to(firmware)): hashlib.sha256(path.read_bytes()).hexdigest()
                            for path in reference_files})
    output = json.dumps(report, ensure_ascii=False, indent=2)
    if arguments.output:
        arguments.output.write_text(output + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
