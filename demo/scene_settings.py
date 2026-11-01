"""Scene-specific acquisition bounds; independent of vehicle and USB transport."""
import json
import math


def settings(grid):
    path = grid.directory / 'survey_settings.json'
    value = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    default = [grid.origin[0], grid.max_x, grid.origin[1], grid.max_y]
    bounds = value.get('bounds', default)
    display = value.get('display_bounds', bounds)
    for rect in (bounds, display):
        if len(rect) != 4 or not all(math.isfinite(v) for v in rect) or rect[0] >= rect[1] or rect[2] >= rect[3]:
            raise ValueError('采集范围应为 [xmin, xmax, ymin, ymax]')
    return {**value, 'bounds': bounds, 'display_bounds': display,
            'bin_origin': value.get('bin_origin', [0., 0.])}


def contains(bounds, pose, margin=0.):
    a, b, c, d = bounds
    return a-margin <= pose[0] <= b+margin and c-margin <= pose[1] <= d+margin
