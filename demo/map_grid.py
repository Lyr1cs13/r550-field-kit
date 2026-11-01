"""Read-only static-map geometry; offline planning is never used on the car."""
import yaml
import hashlib
import heapq
import json
import math
from pathlib import Path
import struct
import zlib


def pgm(path):
    data = Path(path).read_bytes()
    pos = 0
    def token():
        nonlocal pos
        while True:
            while data[pos:pos+1] in (b' ', b'\t', b'\r', b'\n'):
                pos += 1
            if data[pos:pos+1] != b'#':
                break
            pos = data.index(b'\n', pos) + 1
        start = pos
        while data[pos:pos+1] and data[pos:pos+1] not in (b' ', b'\t', b'\r', b'\n'):
            pos += 1
        return data[start:pos]
    kind, width, height, maximum = token(), int(token()), int(token()), int(token())
    if maximum != 255 or width * height > 4_000_000:
        raise ValueError('Unsupported PGM depth/size')
    if kind == b'P5':
        pos += 2 if data[pos:pos+2] == b'\r\n' else 1
        pixels = data[pos:pos+width*height]
    elif kind == b'P2':
        pixels = bytes(int(token()) for _ in range(width*height))
    else:
        raise ValueError('Only P5/P2 maps are supported')
    if len(pixels) != width*height:
        raise ValueError('Truncated PGM')
    return width, height, pixels


class MapGrid:
    def __init__(self, directory):
        self.directory = Path(directory)
        description = (self.directory / 'map.yaml').read_text(encoding='utf-8')
        fields = yaml.safe_load(description)
        self.resolution = float(fields['resolution'])
        self.origin = [float(v) for v in fields['origin']]
        if self.resolution <= 0 or abs(self.origin[2]) > 1e-8:
            raise ValueError('This demo expects the saved axis-aligned map')
        self.width, self.height, self.pixels = pgm(self.directory / fields['image'])
        free, occupied = float(fields['free_thresh']), float(fields['occupied_thresh'])
        negate = int(fields.get('negate', '0'))
        self.cells = []
        for y in range(self.height):
            for x in range(self.width):
                shade = self.pixels[(self.height-y-1)*self.width+x] / 255
                probability = shade if negate else 1-shade
                self.cells.append(100 if probability > occupied else 0 if probability < free else -1)
        mw, mh, self.mask = pgm(self.directory / 'threshold_keepout.pgm')
        if (mw,mh) != (self.width,self.height):
            raise ValueError('Keepout mask dimensions differ from this map')
        self.map_id = hashlib.sha256(description.encode() + self.pixels + self.mask).hexdigest()
        self.max_x = self.origin[0] + self.width*self.resolution
        self.max_y = self.origin[1] + self.height*self.resolution
        bounds = json.loads((self.directory/'boundary.json').read_text())['rectangle']
        self.focus_bounds = [max(self.origin[0],bounds[0]-.5),min(self.max_x,bounds[1]+.5),
                             max(self.origin[1],bounds[2]-.5),min(self.max_y,bounds[3]+.5)]
        self._blocked = None
        metadata_path = self.directory/'map_metadata.json'
        self.version = (json.loads(metadata_path.read_text(encoding='utf-8'))['version']
                        if metadata_path.exists() else '2026-10-10 · 定位恢复版')

    def cell_xy(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def world(self, ix, iy):
        return [self.origin[0]+(ix+.5)*self.resolution,
                self.origin[1]+(iy+.5)*self.resolution]

    def value(self, ix, iy):
        return self.cells[iy*self.width+ix] if 0 <= ix < self.width and 0 <= iy < self.height else -1

    def forbidden(self, ix, iy):
        return (not (0 <= ix < self.width and 0 <= iy < self.height)
                or self.mask[(self.height-iy-1)*self.width+ix] < 128)

    def validate(self, x, y, clearance=.20):
        ix, iy = self.cell_xy(x, y)
        if self.value(ix, iy) != 0:
            raise ValueError('请在已建图的白色空地上设点，不能选择墙体或未知灰色区域')
        if self.forbidden(ix, iy):
            raise ValueError('该位置位于门槛禁行区域，请选择绕行走廊')
        radius = math.ceil(clearance/self.resolution)
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if math.hypot(dx, dy)*self.resolution <= clearance:
                    if self.value(ix+dx, iy+dy) != 0 or self.forbidden(ix+dx, iy+dy):
                        raise ValueError('该停留点离墙、禁区或未知区域太近，请移到走廊中间')

    def info(self):
        return {'width': self.width, 'height': self.height, 'resolution': self.resolution,
                'origin': self.origin, 'max_x': self.max_x, 'max_y': self.max_y,
                'map_id': self.map_id, 'version': self.version,
                'focus_bounds': self.focus_bounds,
                'free_cells': self.cells.count(0), 'unknown_cells': self.cells.count(-1)}

    def png(self):
        # Rotate display only: +X up, +Y left, as the dock-start view used on site.
        rows = []
        for row in range(self.width):
            ix = self.width-row-1
            pixels = bytearray()
            for col in range(self.height):
                iy = self.height-col-1
                value = self.value(ix, iy)
                color = (20, 35, 49) if value == 100 else (109, 130, 135) if value < 0 else (244, 246, 242)
                if self.forbidden(ix, iy):
                    color = (246, 179, 79)
                pixels.extend(color)
            rows.append(b'\0' + pixels)
        def chunk(kind, payload):
            return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind+payload)&0xffffffff)
        return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', self.height, self.width, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(b''.join(rows))) + chunk(b'IEND', b''))

    def offline_path(self, start, goal):
        """Simulation-only A*. Live mode calls native Nav2, never this method."""
        if self._blocked is None:
            blocked = set()
            offsets = [(dx, dy) for dx in range(-4, 5) for dy in range(-4, 5)
                       if math.hypot(dx, dy)*self.resolution <= .18]
            for iy in range(self.height):
                for ix in range(self.width):
                    if self.value(ix, iy) != 0 or self.forbidden(ix, iy):
                        for dx, dy in offsets:
                            if 0 <= ix+dx < self.width and 0 <= iy+dy < self.height:
                                blocked.add((ix+dx, iy+dy))
            self._blocked = blocked
        source, destination = self.cell_xy(*start[:2]), self.cell_xy(*goal[:2])
        if source in self._blocked or destination in self._blocked:
            raise ValueError('演练起点或目标没有足够通行空间')
        queue = [(0, 0, source)];cost = {source: 0};parent = {}
        while queue:
            _, g, node = heapq.heappop(queue)
            if node == destination:
                cells = [node]
                while cells[-1] != source:
                    cells.append(parent[cells[-1]])
                return [self.world(*p) for p in reversed(cells)]
            if g > cost[node]:
                continue
            x, y = node
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = x+dx, y+dy
                if not (0 <= nxt[0] < self.width and 0 <= nxt[1] < self.height):
                    continue
                if nxt in self._blocked or (dx and dy and ((x+dx,y) in self._blocked or (x,y+dy) in self._blocked)):
                    continue
                ng = g + math.hypot(dx, dy)
                if ng < cost.get(nxt, float('inf')):
                    cost[nxt] = ng;parent[nxt] = node
                    heapq.heappush(queue, (ng+math.hypot(nxt[0]-destination[0], nxt[1]-destination[1]), ng, nxt))
        raise ValueError('演练地图内没有可通行路径')
