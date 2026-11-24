"""Static aisle clearance from the robot's map and keep-out overlay."""
import math
import struct
import zlib


class AisleClearance:
    def __init__(self, metadata, image, margin):
        self.origin = metadata['origin']
        self.resolution = metadata['resolution']
        self.width, self.height = metadata['width'], metadata['height']
        self.margin = margin
        if image[:8] != b'\x89PNG\r\n\x1a\n' or min(self.width, self.height) <= 0:
            raise ValueError('走廊净空地图格式无效')
        offset, chunks = 8, []
        while offset+12 <= len(image):
            size, kind = struct.unpack_from('!I4s', image, offset)
            data = image[offset+8:offset+8+size]
            if kind == b'IHDR' and data != struct.pack('!2I5B', self.height, self.width, 8, 2, 0, 0, 0):
                raise ValueError('走廊净空地图尺寸或像素格式不一致')
            if kind == b'IDAT':
                chunks.append(data)
            offset += size+12
        row_size = 1+3*self.height
        pixels = zlib.decompress(b''.join(chunks))
        if len(pixels) != row_size*self.width or any(pixels[::row_size]):
            raise ValueError('走廊净空地图数据无效')
        # MapGrid.png displays +X up and +Y left. White cells alone are traversable.
        free = b'\xf4\xf6\xf2'
        self.rows = tuple(tuple(pixels[i:i+3] == free for i in range(start+1, start+row_size, 3))
                          for start in range(0, len(pixels), row_size))
        self.radius = math.ceil((margin+.08)/self.resolution)

    def distance(self, point):
        scale = self.resolution
        x, y = ((value-origin)/scale for value, origin in zip(point[:2], self.origin))
        ix, iy = math.floor(x), math.floor(y)
        distance = self.margin+.08
        for i in range(ix-self.radius, ix+self.radius+1):
            dx = max(i-x, x-i-1, 0.) * scale
            if dx >= distance:
                continue
            for j in range(iy-self.radius, iy+self.radius+1):
                if 0 <= i < self.width and 0 <= j < self.height and self.rows[self.width-1-i][self.height-1-j]:
                    continue
                dy = max(j-y, y-j-1, 0.) * scale
                distance = min(distance, math.hypot(dx, dy))
        return distance

    def allows(self, path, *, rejoin=False, departing=False):
        if not path or self.distance(path[-1]) < self.margin:
            return False
        minimum = self.distance(path[0])
        if minimum <= 0. or (minimum < self.margin and not (rejoin or departing)):
            return False
        traveled = 0.
        for first, last in zip(path, path[1:]):
            distance = math.dist(first[:2], last[:2])
            count = max(1, math.ceil(distance/.025))
            for i in range(1, count+1):
                step = i/count
                current = self.distance(tuple(a+(b-a)*step for a, b in zip(first[:2], last[:2])))
                if current+1e-6 >= self.margin:
                    minimum = self.margin
                elif (not (rejoin or departing) or current <= 0. or current+.015 < minimum
                      or traveled+distance*step > (.65 if departing else .55)):
                    return False
                else:
                    minimum = max(minimum, current)
            traveled += distance
        return not rejoin or traveled <= 1.10
