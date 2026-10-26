import math


# 模式键 → 马回传的模式字符（与电控 action_keys.cpp、遥控器屏幕一致）
MODE_STATUS = {9: 'a', 10: 'c', 11: 'd', 12: 'e', 13: 'f',
               14: 'g', 15: 'h', 16: 'j', 17: 'k', 18: 'b'}
# 一次性动作（跳跃、离家）：电控执行期间忽略新动作键，上位机要按时长等它做完
TIMED_ACTIONS = {(9, 6), (9, 7), (18, 6), (18, 7), (11, 6),
                 (11, 7), (13, 6), (15, 6), (15, 7), (17, 6)}


def encode_key(key):
    """与遥控器相同的三字节帧：53 键值 45，中间是原始数值。"""
    if not isinstance(key, int) or not 1 <= key <= 23:
        raise ValueError('按键必须是 1..23 的整数')
    return bytes((0x53, key, 0x45))


def crc8(data):
    result = 0
    for byte in data:
        result ^= byte
        for bit_index in range(8):
            result = ((result << 1) ^ (0x07 if result & 0x80 else 0)) & 0xFF
    return result


class StatusParser:
    """解析马回传的状态帧 A5 5A 模式 故障 电机 详情 CRC8 0D（与遥控器收到的完全相同）。"""

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
    """解析 [动作键, 1, 模式键, 请求序号]。

    格式不对返回 None，由调用者丢弃这一条，不暂停自动。
    不再检查“这个模式有没有这个动作”：电控收到没定义的组合会自己忽略，
    以后电控加了新动作也不用改这里。
    """
    if len(data) != 4 or not all(math.isfinite(value) and value == int(value) for value in data):
        return None
    key, _repeat, mode, request_id = map(int, data)
    if mode not in MODE_STATUS or not 1 <= key <= 20:
        return None
    return dict(key=key, mode=mode, request_id=request_id)


def parse_durations(entries):
    """解析 '模式:动作:秒' 列表。

    只要求每一条格式正确；可以只写一部分、也可以多写。
    没写时长的动作按连续动作处理（发出后不等待）。
    """
    durations = {}
    for entry in entries:
        mode, key, seconds = entry.split(':')
        duration = float(seconds)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError(f'动作时长必须为有限正数：{entry}')
        durations[int(mode), int(key)] = duration
    return durations
