"""Charge/resume policy with explicit calibrated hysteresis, no estimated SOC."""
import math


class Endurance:
    def __init__(self, config):
        self.config = config
        self.low_since = self.charging_since = self.ready_since = None
        self.pending = False
        self.resuming = False
        if config.get('enabled'):
            lo, hi = config.get('low_voltage'), config.get('resume_voltage')
            if not all(isinstance(v, (int,float)) and math.isfinite(v) for v in (lo,hi)) or hi <= lo+.5:
                raise ValueError('自动充电续作需要实测的低电压与恢复电压，恢复电压须至少高0.5V')
            for key in ('low_hold_seconds','charge_hold_seconds'):
                value=config.get(key)
                if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                    raise ValueError('充电策略的持续时间必须为有效正数')
            minimum=config.get('min_charge_seconds',0.)
            if not isinstance(minimum,(int,float)) or not math.isfinite(minimum) or minimum<0:
                raise ValueError('最短充电时间无效')

    def allows_resume(self, repeat):
        return self.config.get('enabled',False) and (repeat or not self.config.get('repeat_only',False))

    def observe(self, now, health, *, repeat=False):
        voltage = health.get('voltage')
        finite = isinstance(voltage,(int,float)) and math.isfinite(voltage)
        low = finite and voltage < self.config['low_voltage']
        self.low_since = (self.low_since if self.low_since is not None else now) if low else None
        if self.allows_resume(repeat) and self.low_since is not None and now-self.low_since>=self.config['low_hold_seconds']:
            self.pending = True
        charging=self.resuming and health.get('charging_stable') is True and health.get('stationary')
        self.charging_since=(self.charging_since if self.charging_since is not None else now) if charging else None
        dock=health.get('dock_localization')
        good = (charging and finite and voltage >= (self.config.get('resume_voltage') or math.inf)
                and health.get('ready') and health.get('localization_confirmed')
                and (dock is None or dock.get('status')=='confirmed'))
        self.ready_since = (self.ready_since if self.ready_since is not None else now) if good else None
        return (self.ready_since is not None and now-self.ready_since>=self.config['charge_hold_seconds']
                and now-self.charging_since>=self.config.get('min_charge_seconds',0.))

    def cancel(self):
        self.pending=self.resuming=False
        self.low_since=self.charging_since=self.ready_since=None
