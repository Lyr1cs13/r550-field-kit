"""One stationary AMCL initialization per verified charging visit. No motion."""
import math

from directed_route import dock_offset

DOCK_MATCH_RADIUS = .14
DOCK_SCAN_START_YAW_DEG = 6.
SCAN_FIT_TIMEOUT = 60.
AMCL_VERIFY_TIMEOUT = 15.


def localization_inputs_ready(now, *, map_matches, mapping, last_scan, lifecycle):
    # Nav2 costmaps can wait for map->base TF during configuration. AMCL must
    # receive the dock seed before those navigation nodes can become active.
    return (map_matches and not mapping and now-last_scan < 2
            and all(lifecycle.get(name,(0,0))[0] == 3
                    and now-lifecycle[name][1] < 8
                    for name in ('amcl','map_server')))


def in_dock_capture(pose, reference):
    if not pose or not all(math.isfinite(v) for v in pose):
        return False
    along,side = dock_offset(pose,reference)
    angle = math.remainder(pose[2]-reference[2],2*math.pi)
    return abs(along)<=.28 and abs(side)<=DOCK_MATCH_RADIUS and abs(angle)<=math.radians(DOCK_SCAN_START_YAW_DEG)


def matches_dock(pose, reference):
    return (bool(pose) and all(math.isfinite(v) for v in pose)
            and math.dist(pose[:2],reference[:2])<=DOCK_MATCH_RADIUS
            and abs(math.remainder(pose[2]-reference[2],2*math.pi))<=math.radians(10))


def staging_scan_supported(pose, approach, dock, scan_window, covariance):
    """Accept a lower scan score only in the well-localized approach to this dock."""
    if (not pose or scan_window is None or not .70<=scan_window<=1.
            or covariance is None or len(covariance)!=3
            or any(v is None or not math.isfinite(v) or v<0. for v in covariance)
            or covariance[2]>math.radians(8)**2):
        return False
    forward, side = dock_offset(pose,dock)
    # Bound lateral uncertainty conservatively even when the cross-covariance is unavailable.
    yaw=dock[2]
    lateral_sigma=(abs(math.sin(yaw))*math.sqrt(covariance[0])
                   +abs(math.cos(yaw))*math.sqrt(covariance[1]))
    return (forward>=.30 and abs(side)<=.20 and abs(side)+2*lateral_sigma<=.28
            and math.dist(pose[:2],approach)<=.28)


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
             amcl_updated, pose_request_pending, scan_pending=False, scan_started=False):
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
            # The search gate admits longitudinal encoder overrun. The fitted
            # scan and the resulting AMCL pose must still match this dock.
            if not in_dock_capture(pose,self.reference):
                self.status='alignment_required';self.stable_since=None
                return None
            if self.status=='alignment_required':self.status='waiting'
        deadline = SCAN_FIT_TIMEOUT if scan_pending else AMCL_VERIFY_TIMEOUT
        # Once a verified charging visit triggered a scan, complete pose
        # verification at rest even if the electrical contact bounces.
        eligible = stationary and sensors_ready and (charging or (self.status=='applying' and scan_started))
        if not eligible:
            self.stable_since = self.aligned_since = None
            if self.status == 'applying' and now-self.started > deadline:
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
        if now-self.started > deadline:
            self.status = 'attention'
            return None
        if pose_request_pending or not amcl_updated or not matches_dock(pose,self.reference):
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
