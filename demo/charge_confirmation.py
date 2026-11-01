"""Confirm sustained charging contact, not a single bouncing contact frame."""




class ChargeConfirmation:
    def __init__(self, duration=2.0, minimum_current=.10, full_voltage=12.3):
        self.duration=duration;self.minimum_current=minimum_current;self.full_voltage=full_voltage
        self.since=None;self.confirmed=False

    def update(self,now,*,charging,stationary,current,voltage):
        electrical=(current is not None and current>=self.minimum_current) or (
            charging and voltage is not None and voltage>=self.full_voltage)
        if not (stationary and electrical):
            self.since=None;self.confirmed=False;return False
        if self.since is None:self.since=now
        self.confirmed=now-self.since>=self.duration
        return self.confirmed


class NativeDockingWatchdog:
    """Bound abnormal OEM motion without splitting a healthy docking sequence."""
    def __init__(self,near_travel_limit=.28):
        # 18 cm capture distance plus IR alignment/contact compression. The
        # previous 20 cm encoder budget stopped an aligned approach just
        # before electrical contact. Keep the independent 20 s deadline and total 45 s deadline.
        self.near_travel_limit=near_travel_limit;self.reset(0.,0.)

    def reset(self,now,travel):
        self.started=now;self.start_travel=travel
        self.near_since=None;self.near_travel=None

    def update(self,now,*,travel,near,fresh):
        if not fresh:return '回充传感器反馈中断，已停止原生对接'
        if near and self.near_since is None:
            self.near_since=now;self.near_travel=travel
        # Latch the near-dock budget even if encoder drift moves map pose away.
        if self.near_since is not None:
            if travel-self.near_travel>=self.near_travel_limit:
                return '桩旁轮程已达到保护上限但未检测到充电，已停止对接'
            if now-self.near_since>=20.:
                return '桩旁连续对接未取得充电反馈，已停止对接'
        if travel-self.start_travel>=1.5 or now-self.started>=45.:
            return '原生对接达到行程或时间上限但未充电，已停止对接'
        return None
