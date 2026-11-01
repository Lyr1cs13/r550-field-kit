"""Robust affine device/host clock alignment from fresh device heartbeats.

RX queue samples never train this clock. Actual old RSSI remains old even while
STATUS keeps arriving; UART bursts and excessive drift are not silently accepted.
"""
from collections import deque
import statistics

class DeviceClock:
    def __init__(self):self.observations=deque();self.reference=None;self.bias=None;self.skew=0.;self.jitter=0.
    def observe(self,device_seconds,host_seconds):
        self.observations.append((device_seconds,host_seconds-device_seconds))
        while self.observations and device_seconds-self.observations[0][0]>90:self.observations.popleft()
        if self.reference is None:self.reference=device_seconds
        # Low envelope in 4-second blocks suppresses scheduling/read delays.
        blocks={}
        for u,o in self.observations:
            key=int((u-self.reference)//4)
            if key not in blocks or o<blocks[key][1]:blocks[key]=(u,o)
        points=list(blocks.values())
        slopes=[(b[1]-a[1])/(b[0]-a[0]) for i,a in enumerate(points) for b in points[i+1:] if b[0]-a[0]>=12]
        if slopes:self.skew=max(-.002,min(.002,statistics.median(slopes)))
        residuals=sorted(o-self.skew*(u-self.reference) for u,o in self.observations)
        self.bias=residuals[int((len(residuals)-1)*.10)]
        self.jitter=statistics.median(residuals)-self.bias
    def map_time(self,device_seconds):
        if self.bias is None:return None
        return device_seconds+self.bias+self.skew*(device_seconds-self.reference)
    def summary(self):return dict(clock_skew_ppm=round(self.skew*1e6,1),clock_jitter_ms=round(self.jitter*1000,1),clock_heartbeats=len(self.observations))
