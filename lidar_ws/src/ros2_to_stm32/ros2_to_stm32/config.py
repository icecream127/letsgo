"""

                               _(\_/)
                             ,((((^`\
                             ((((  (6 \
                          ,((((( ,    \
       ,,,_              ,(((((  /"._  ,`,
      ((((\\ ,...       ,((((   /    `-.-'
      )))  ;'    `"'"'""((((   (      
    (((  /            (((      \
      )) |                      |
     ((  |        .       '     |
     ))  \     _ '      `t   ,.')
     (   |   y;- -,-""'"-.\   \/
    )   / ./  ) /         `\  \
        |./   ( (           / /'
        ||     \\          //'|
        ||      \\       _//'||
        ||       ))     |_/  ||
        \_\     |_/          ||
        `'"                  \_\
                             `'"
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# generate_zone_config.py
import json
import os

def generate_json_file():
    # 1. 初始数据 (注意 func 改成了字符串)
    zones = {
        "木桥A": {"begin_x": 3.42, "begin_y": 0.0, "func_name": "handle_bridge_A"},
        "木桥B": {"func_name": "handle_bridge_B"},
        "大斜坡": {"func_name": "handle_big_slope"},
      #   "找限高栏": {"func_name": "handle_limit_pole_seek"},
        "限高栏": {"func_name": "handle_limit_pole"},
        "沙坑": {"begin_x": -5.34, "begin_y": 3.94, "func_name": "handle_bunker"},
        "绕杆": {"begin_x": -6.995, "begin_y": 1.175, "func_name": "handle_pole"},
        "高墙": {"begin_x": -2.28, "begin_y": 0.06, "func_name": "handle_hill_wall"},
        "台阶": {"begin_x": 0.0, "begin_y": 0.0, "func_name": "handle_step"}
    }
    
    # 2. 动态计算所有边界
    zones["木桥A"]["x_min"] = zones["木桥A"]["begin_x"]
    zones["木桥A"]["x_max"] = zones["木桥A"]["begin_x"] + 2.0
    zones["木桥A"]["y_min"] = zones["木桥A"]["begin_y"] - 0.6
    zones["木桥A"]["y_max"] = zones["木桥A"]["begin_y"] + 4.9

    zones["木桥B"]["begin_x"] = zones["木桥A"]["begin_x"]
    zones["木桥B"]["begin_y"] = zones["木桥A"]["begin_y"] +4.4 - 0.35 + 0.08
    zones["木桥B"]["x_min"] = zones["木桥B"]["begin_x"] - 1.5 - 0.2
    zones["木桥B"]["x_max"] = zones["木桥A"]["x_min"]
    zones["木桥B"]["y_min"] = zones["木桥B"]["begin_y"] - 0.6
    zones["木桥B"]["y_max"] = zones["木桥B"]["begin_y"] + 0.4

    zones["大斜坡"]["begin_x"] = zones["木桥B"]["begin_x"] - 2.37
    zones["大斜坡"]["begin_y"] = zones["木桥B"]["begin_y"] - 0.15
    zones["大斜坡"]["x_min"] = zones["大斜坡"]["begin_x"] - 3.2
    zones["大斜坡"]["x_max"] = zones["木桥B"]["x_min"]
    zones["大斜坡"]["y_min"] = zones["大斜坡"]["begin_y"] - 1.6
    zones["大斜坡"]["y_max"] = zones["大斜坡"]["begin_y"] + 0.6

    zones["沙坑"]["x_min"] = zones["沙坑"]["begin_x"] - 3.0
    zones["沙坑"]["x_max"] = zones["沙坑"]["begin_x"] + 0.5
    zones["沙坑"]["y_min"] = zones["沙坑"]["begin_y"] - 1.8
    zones["沙坑"]["y_max"] = zones["沙坑"]["begin_y"] + 1.0

    zones["限高栏"]["begin_x"] = zones["大斜坡"]["begin_x"] - 4.0
    zones["限高栏"]["begin_y"] = zones["大斜坡"]["begin_y"]
    zones["限高栏"]["x_min"] = zones["沙坑"]["x_max"]
    zones["限高栏"]["x_max"] = zones["大斜坡"]["x_min"]
    zones["限高栏"]["y_min"] = zones["大斜坡"]["y_min"] - 1.0
    zones["限高栏"]["y_max"] = zones["大斜坡"]["y_max"] + 1.0

#     zones["找限高栏"]["begin_x"] = zones["限高栏"]["begin_x"]
#     zones["找限高栏"]["begin_y"] = zones["限高栏"]["begin_y"] - 1.8
#     zones["找限高栏"]["x_min"] = zones["限高栏"]["begin_x"] - 1.0
#     zones["找限高栏"]["x_max"] = zones["大斜坡"]["x_min"] + 2.0
#     zones["找限高栏"]["y_min"] = zones["大斜坡"]["y_min"] - 0.75
#     zones["找限高栏"]["y_max"] = zones["大斜坡"]["y_min"]

    zones["绕杆"]["x_min"] = zones["沙坑"]["x_min"]
    zones["绕杆"]["x_max"] = zones["高墙"]["begin_x"] - 1.7
    zones["绕杆"]["y_min"] = -2.0
    zones["绕杆"]["y_max"] = zones["沙坑"]["y_min"]

    zones["高墙"]["x_min"] = zones["绕杆"]["x_max"]
    zones["高墙"]["x_max"] = zones["高墙"]["begin_x"] + 0.1
    zones["高墙"]["y_min"] = -1.0
    zones["高墙"]["y_max"] = 1.0

    zones["台阶"]["x_min"] = zones["高墙"]["x_max"]
    zones["台阶"]["x_max"] = 1.4
    zones["台阶"]["y_min"] = -1.0
    zones["台阶"]["y_max"] = 1.0

    zone = {
        "区域1": {"x_min": 1.4, "x_max": zones["木桥A"]["x_min"],
                  "y_min": -0.6, "y_max": 2.0, "func_name": "木桥A"}
    }
    
    # ... 此处省略 zon 字典的代码，按同样方式替换即可 ...
    zon = {
            "0": {"x_min": zones["木桥A"]["begin_x"], "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] - 0.5, "y_max": zones["木桥A"]["begin_y"] + 0.5},
            "1": {"x_min": zones["木桥A"]["begin_x"] + 0.8, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 0.65, "y_max": zones["木桥A"]["begin_y"] + 1.05},
            "2": {"x_min": zones["木桥A"]["begin_x"] + 0.8, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 1.2, "y_max": zones["木桥A"]["begin_y"] + 1.60},
            "3": {"x_min": zones["木桥A"]["begin_x"] + 0.8, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 1.75, "y_max": zones["木桥A"]["begin_y"] + 2.15},
            "4": {"x_min": zones["木桥A"]["begin_x"] + 0.8, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 2.3, "y_max": zones["木桥A"]["begin_y"] + 2.7},
            "5": {"x_min": zones["木桥A"]["begin_x"] + 0.8, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 2.85, "y_max": zones["木桥A"]["begin_y"] + 3.25},
            "6": {"x_min": zones["木桥A"]["begin_x"] - 0.2, "x_max": zones["木桥A"]["begin_x"] + 1.8,
                  "y_min": zones["木桥A"]["begin_y"] + 3.4, "y_max": zones["木桥A"]["begin_y"] + 4.4},
            "7": {"x_min": zones["木桥A"]["begin_x"] - 1.7, "x_max": zones["木桥A"]["begin_x"] - 0.2,
                  "y_min": zones["木桥A"]["begin_y"] + 3.5, "y_max": zones["木桥A"]["begin_y"] + 4.3},
            "8": {"x_min": zones["木桥A"]["begin_x"] - 2.7, "x_max": zones["木桥A"]["begin_x"] - 1.7,
                  "y_min": zones["木桥A"]["begin_y"] + 3.4, "y_max": zones["木桥A"]["begin_y"] + 4.4},
            "9": {"x_min": zones["木桥A"]["begin_x"] - 5.7, "x_max": zones["木桥A"]["begin_x"] - 1.7,
                  "y_min": zones["木桥A"]["begin_y"] + 2.4, "y_max": zones["木桥A"]["begin_y"] + 3.4},
            ".": {"x_min": zones["限高栏"]["begin_x"] - 0.05, "x_max": zones["限高栏"]["begin_x"],
                  "y_min": zones["限高栏"]["begin_y"] - 0.5, "y_max": zones["限高栏"]["begin_y"] + 0.5},
            "10": {"x_min": zones["沙坑"]["begin_x"] - 2, "x_max": zones["沙坑"]["begin_x"],
                   "y_min": zones["沙坑"]["begin_y"] - 0.5, "y_max": zones["沙坑"]["begin_y"] + 0.5},
            "11": {"x_min": zones["沙坑"]["begin_x"] - 2, "x_max": zones["沙坑"]["begin_x"] - 1.0,
                   "y_min": zones["沙坑"]["begin_y"] - 1.5, "y_max": zones["沙坑"]["begin_y"] + 0.5},
            "12": {"x_min": zones["沙坑"]["begin_x"] - 1.55, "x_max": zones["沙坑"]["begin_x"] - 1.45,
                   "y_min": -0.05, "y_max": 0.05},
            "13": {"x_min": zones["沙坑"]["begin_x"] - 1.55, "x_max": zones["沙坑"]["begin_x"] - 1.45,
                   "y_min": 0.95, "y_max": 1.05},
            "14": {"x_min": zones["沙坑"]["begin_x"] - 0.55, "x_max": zones["沙坑"]["begin_x"] - 0.45,
                   "y_min": -0.05, "y_max": 0.05},
            "15": {"x_min": zones["沙坑"]["begin_x"] + 0.45, "x_max": zones["沙坑"]["begin_x"] + 0.55,
                   "y_min": -0.05, "y_max": 0.05},
            "16": {"x_min": zones["高墙"]["begin_x"] - 0.05, "x_max": zones["高墙"]["begin_x"],
                   "y_min": -0.5, "y_max": 0.5},
            "17": {"x_min": zones["台阶"]["begin_x"], "x_max": zones["台阶"]["begin_x"] + 0.3,
                   "y_min": -0.5, "y_max": 0.5},
            "18": {"x_min": zones["台阶"]["begin_x"] + 0.3, "x_max": zones["台阶"]["begin_x"] + 0.6,
                   "y_min": -0.5, "y_max": 0.5},
            "19": {"x_min": zones["台阶"]["begin_x"] + 0.6, "x_max": zones["台阶"]["begin_x"] + 0.9,
                   "y_min": -0.5, "y_max": 0.5}
        }

    # 3. 将所有数据打包成一个字典
    all_data = {
        "zones": zones,
        "zone": zone,
        "zon": zon
    }

    # 4. 写入到 JSON 文件中
    with open(os.path.join(os.path.dirname(__file__), 'map_zones_config.json'), 'w', encoding='utf-8') as f:
        # indent=4 让 json 文件具备良好的可读性，ensure_ascii=False 允许显示中文
        json.dump(all_data, f, indent=4, ensure_ascii=False)
    
    print("成功生成配置文件: map_zones_config.json")

if __name__ == '__main__':
    generate_json_file()
