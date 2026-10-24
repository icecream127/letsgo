import math


MODE_STATUS = {9: 'a', 10: 'c', 11: 'd', 12: 'e', 13: 'f',
               14: 'g', 15: 'h', 16: 'j', 17: 'k', 18: 'b'}
TIMED_ACTIONS = {(9, 6), (9, 7), (18, 6), (18, 7), (11, 6),
                 (11, 7), (13, 6), (15, 6), (15, 7), (17, 6)}
EXTRA_ACTIONS = {9: {6, 7, 8, 19, 20}, 18: {6, 7, 19, 20},
                 11: {6, 7}, 13: {6, 7}, 15: {6, 7}, 17: {6}}


def encode_key(key):
    if not isinstance(key, int) or not 1 <= key <= 23:
        raise ValueError('按键必须是 1..23 的整数')
    return bytes((0x53, key, 0x45))


def validate_action(mode, key):
    if mode not in MODE_STATUS or key not in set(range(1, 6)) | EXTRA_ACTIONS.get(mode, set()):
        raise ValueError(f'电控不支持模式 {mode} / 动作 {key}')


def crc8(data):
    result = 0
    for byte in data:
        result ^= byte
        for bit_index in range(8):
            result = ((result << 1) ^ (0x07 if result & 0x80 else 0)) & 0xFF
    return result


class StatusParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        statuses = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            if len(self.buffer) < 8:
                break
            frame = self.buffer[:8]
            if (frame[7] != 0x0D or crc8(frame[:6]) != frame[6]
                    or chr(frame[2]) not in set(MODE_STATUS.values()) | {'z'}):
                del self.buffer[0]
                continue
            statuses.append(dict(mode=chr(frame[2]), fault=frame[3],
                                 motor=frame[4], detail=frame[5]))
            del self.buffer[:8]
        return statuses


def parse_request(data):
    if len(data) != 4 or not all(math.isfinite(value) and value == int(value) for value in data):
        raise ValueError('需要 [动作键, 1, 模式键, 请求序号] 四个整数')
    key, repeat, mode, request_id = map(int, data)
    validate_action(mode, key)
    if repeat != 1 or not 1 <= request_id < 2 ** 24:
        raise ValueError('禁止重复发送或无效请求序号')
    return dict(key=key, mode=mode, request_id=request_id)


def parse_durations(entries):
    durations = {}
    for entry in entries:
        mode, key, seconds = entry.split(':')
        duration = float(seconds)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('动作时长必须为有限正数')
        durations[int(mode), int(key)] = duration
    if set(durations) != TIMED_ACTIONS:
        raise ValueError('一次性动作时长表必须与电控映射一致')
    return durations
