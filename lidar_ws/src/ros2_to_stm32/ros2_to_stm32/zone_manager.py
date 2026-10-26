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
# zone_manager.py
import json
import os

def zone_config_path():
    return os.environ.get('LEGACY_ZONE_CONFIG', os.path.join(os.path.dirname(__file__), 'map_zones_config.json'))


class ZoneManager:
    def __init__(self, filepath='map_zones_config.json'):
        self.filepath = filepath
        # 如果文件不存在，给个提示
        if not os.path.exists(self.filepath):
            raise FileNotFoundError(f"找不到配置文件: {self.filepath}，请先运行生成脚本。")

    def load_data(self):
        """读取文件并返回字典数据"""
        with open(self.filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data

    def save_data(self, data):
        """
        写入数据到文件
        (为防止多节点同时写入导致冲突，先写入临时文件，再重命名覆盖)
        """
        tmp_filepath = self.filepath + ".tmp"
        with open(tmp_filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        # os.replace 是原子操作，保证了多节点写文件的安全性
        os.replace(tmp_filepath, self.filepath)

    def bind_functions(self, target_instance, zones_data):
        """
        把 json 中的 func_name 还原成类里面的实际函数对象
        """
        for key, value in zones_data.items():
            if "func_name" in value:
                func_name = value["func_name"]
                # 检查类中是否存在该方法
                if hasattr(target_instance, func_name):
                    # 将方法重新绑定到字典中
                    value["func"] = getattr(target_instance, func_name)
                else:
                    print(f"警告：未在 {target_instance.__class__.__name__} 中找到函数 {func_name}")
        return zones_data
