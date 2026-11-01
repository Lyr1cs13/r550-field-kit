"""One stationary AMCL initialization per verified charging visit. No motion."""
import math


def localization_inputs_ready(now, *, map_matches, mapping, last_scan, lifecycle):
    # Nav2 costmaps can wait for map->base TF during configuration. AMCL must
    # receive the dock seed before those navigation nodes can become active.
    return (map_matches and not mapping and now-last_scan < 2
            and all(lifecycle.get(name,(0,0))[0] == 3
                    and now-lifecycle[name][1] < 8
                    for name in ('amcl','map_server')))


class DockLocalization:
    def __init__(self, reference):
        self.reference = tuple(reference)
        self.count = 0
        self.arm()

    def arm(self):
        self.status = 'waiting'
        self.stable_since = None
        self.started = None
        self.aligned_since = None
        self.away_since = None

    def suppress(self):
        # A manual non-dock pose must not be overwritten by charging telemetry.
        self.status = 'manual'
        self.stable_since = self.started = self.aligned_since = None

    def manual_dock(self, now):
        self.arm()
        self.status = 'applying'
        self.started = now

    def step(self, now, *, charging, stationary, sensors_ready, busy, pose,
             amcl_updated, pose_request_pending):
        if self.status == 'confirmed' and not charging:
            if self.away_since is None:
                self.away_since = now
            if now-self.away_since >= 5:
                self.arm()
            return None
        if charging:
            self.away_since = None
        if self.status in ('confirmed', 'manual', 'attention'):
            return None
        if busy:
            self.arm()
            return None
        if self.status in ('waiting','alignment_required') and pose is not None:
            angle=math.atan2(math.sin(pose[2]-self.reference[2]),math.cos(pose[2]-self.reference[2]))
            # Electrical contact alone does not establish a mechanically seated
            # pose. Never erase a visible side/angle error by seeding the dock.
            if (not all(math.isfinite(v) for v in pose) or
                    math.dist(pose[:2],self.reference[:2])>.06 or abs(angle)>math.radians(5)):
                self.status='alignment_required';self.stable_since=None
                return None
            if self.status=='alignment_required':self.status='waiting'
        eligible = charging and stationary and sensors_ready
        if not eligible:
            self.stable_since = self.aligned_since = None
            if self.status == 'applying' and now-self.started > 15:
                self.status = 'attention'
            return None
        if self.stable_since is None:
            self.stable_since = now
        if now-self.stable_since < 3:
            return None
        if self.status == 'waiting':
            if pose_request_pending:
                return None
            self.status = 'applying'
            self.started = now
            return 'initialize'
        if now-self.started > 15:
            self.status = 'attention'
            return None
        aligned = False
        if pose and all(math.isfinite(v) for v in pose):
            angle = math.atan2(math.sin(pose[2]-self.reference[2]),
                               math.cos(pose[2]-self.reference[2]))
            aligned = math.dist(pose[:2], self.reference[:2]) <= .10 and abs(angle) <= math.radians(10)
        if pose_request_pending or not amcl_updated or not aligned:
            self.aligned_since = None
            return None
        if self.aligned_since is None:
            self.aligned_since = now
        if now-self.aligned_since >= 1:
            self.status = 'confirmed'
            self.count += 1
            return 'confirm'
        return None

    def snapshot(self):
        return {'status': self.status, 'calibrations': self.count,
                'reference': list(self.reference)}
