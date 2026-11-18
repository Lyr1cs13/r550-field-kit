"""Onboard limits for operating two localized robots in separate areas."""
from dataclasses import dataclass
import json
import math
from pathlib import Path


def separated(first, second, gap=.25):
    """Rectangles describe each vehicle's entire permitted operating area."""
    if first is None or second is None:
        return False
    return max(first[0]-second[1], second[0]-first[1],
               first[2]-second[3], second[2]-first[3]) >= gap


@dataclass(frozen=True)
class FleetZone:
    robot_id: str
    map_id: str
    bounds: tuple | None

    @classmethod
    def load(cls, path, map_id):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        bounds = data['bounds']
        if data['robot_id'] not in ('a','b') or data['map_id'] != map_id:
            raise ValueError('双车区域配置与本车或地图不一致')
        if bounds is not None:
            if (not isinstance(bounds, list) or len(bounds) != 4
                    or not all(type(v) in (int,float) and math.isfinite(v) for v in bounds)
                    or min(bounds[1]-bounds[0],bounds[3]-bounds[2]) <= .65):
                raise ValueError('双车运行区尺寸或坐标无效')
            bounds = tuple(bounds)
        return cls(data['robot_id'],map_id,bounds)

    def contains(self, point, margin):
        if self.bounds is None or point is None:
            return False
        x,y = point[:2]
        xmin,xmax,ymin,ymax = self.bounds
        return xmin+margin <= x <= xmax-margin and ymin+margin <= y <= ymax-margin

    def motion_reason(self, pose, pose_age, covariance, match, path):
        if self.bounds is None:
            return '双车运行区待核对'
        if (pose is None or not 0 <= pose_age <= .4 or covariance is None
                or len(covariance) < 2 or any(v is None or not math.isfinite(v) or v < 0 for v in covariance[:2])
                or match is None or not math.isfinite(match) or match < .80):
            return '双车模式等待新鲜定位与雷达校验'
        sigma = math.sqrt(max(covariance[:2]))
        if sigma > .12:
            return '定位不确定性过大，双车模式保持停车'
        margin = .32 + 2*sigma
        if not self.contains(pose,margin):
            return '车辆接近运行区边界，双车模式保持停车'
        if any(not self.contains(point,margin) for point in path):
            return '规划路线超出本车运行区'
        return ''
