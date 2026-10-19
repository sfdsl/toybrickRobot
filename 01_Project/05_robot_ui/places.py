"""Map-bound named places. Atomic writes; never silently reuse points on a new map."""
import hashlib
import json
import math
import os
import tempfile
import threading
import time


class PlaceError(ValueError):
    pass


class PlaceStore:
    def __init__(self, map_prefix):
        self.prefix = os.path.expanduser(map_prefix)
        self.path = self.prefix + '.places.json'
        self.lock = threading.Lock()

    def map_id(self):
        digest = hashlib.sha256()
        for suffix in ('.yaml', '.pgm'):
            path = self.prefix + suffix
            if not os.path.isfile(path):
                raise PlaceError('地图文件缺失：' + path)
            with open(path, 'rb') as src:
                for chunk in iter(lambda: src.read(65536), b''):
                    digest.update(chunk)
        return digest.hexdigest()

    def _read(self):
        current = self.map_id()
        if not os.path.exists(self.path):
            return {'schema': 1, 'map_id': current, 'frame': 'map', 'places': {}}
        with open(self.path, encoding='utf-8') as src:
            data = json.load(src)
        if data.get('map_id') != current:
            raise PlaceError('地点文件属于另一版地图；请先迁移地点，原文件未修改')
        if data.get('schema') != 1 or data.get('frame') != 'map' or not isinstance(data.get('places'), dict):
            raise PlaceError('地点文件格式无效')
        return data

    def list(self):
        with self.lock:
            return self._read()

    def add(self, name, x, y):
        name = str(name).strip()
        if not name or len(name) > 40 or any(c in name for c in '\r\n\t'):
            raise PlaceError('地点名称须为 1～40 个字符，不能包含换行或制表符')
        x, y = float(x), float(y)
        if not (math.isfinite(x) and math.isfinite(y)):
            raise PlaceError('地点坐标须为有限数值')
        with self.lock:
            data = self._read()
            data['places'][name] = {'x': x, 'y': y, 'updated_at': int(time.time())}
            self._write(data)
            return data['places'][name]

    def delete(self, name):
        with self.lock:
            data = self._read()
            if name not in data['places']:
                raise PlaceError('地点不存在：' + name)
            del data['places'][name]
            self._write(data)

    def get(self, name):
        with self.lock:
            data = self._read()
            if name not in data['places']:
                raise PlaceError('地点不存在：' + name)
            return data['places'][name].copy(), data['map_id']

    def _write(self, data):
        directory = os.path.dirname(self.path)
        fd, tmp = tempfile.mkstemp(prefix='.places-', suffix='.tmp', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as dst:
                json.dump(data, dst, ensure_ascii=False, indent=2, sort_keys=True)
                dst.write('\n')
                dst.flush()
                os.fsync(dst.fileno())
            os.replace(tmp, self.path)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)
