"""Fruit-hitting state machine. No ROS in here, so it can be unit tested.

    IDLE ──start──► PREPARE ────────► DRIVE ◄──────────────────────┐
      ▲             gantry → ready     forward at forward_pwm       │
      │                                  │ line_hit = 1             │ line_hit = 0
      │                                  ▼                          │ for ready_delay_s
      │                                slower by hit_slowdown_pct,  │
      │                                gantry → "hit" ──────────────┘ gantry → "ready"
      └──── stop / fault ◄──── (from any mode)

The robot never stops for a fruit: it keeps driving, only slower, while the
gantry reaches out to the "hit" position, and pulls the gantry back to
"ready" once the fruit has left the hit line. The gantry always completes the
reach first: it goes back only after /mega/status confirms it arrived at "hit",
even if the fruit left the line sooner. If it does not arrive within
move_timeout_s the run aborts.

IDLE is the only mode where the user may teleop the base, jog the gantry,
move the gripper and save positions, so a manual command never fights the
automatic run.

Positions are in gantry steps from mega_bridge's home (pos1, pos2 in
/mega/status). Drive speed is a wheel PWM (0..255): there is no wheel
feedback, so PWM is the honest unit. It is sent as linear.x = pwm / k_lin,
which wheel_control turns back into the same base PWM. W/S teleop uses the
same forward_pwm, so what the user tries by hand is what START drives at.
Time is any monotonic clock in seconds, passed in by the caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

IDLE = 'idle'
PREPARE = 'prepare'
DRIVE = 'drive'

READY = 'ready'
HIT = 'hit'
POSITION_NAMES = (READY, HIT)
LEGACY_NAMES = {'pick': HIT}      # settings files from the pick-and-grip version

# Statuses to wait for after a command before trusting it finished. Same
# reason as in mega_bridge: a status the Mega sent before it received the
# command already reads "0 steps left" at the new target.
FRESH_STATUS = 2

Command = Tuple[str, object]     # ('goto', (x, y)) | ('step', (s1, s2)) | ('gripper', deg)
Base = Optional[Tuple[float, float]]   # (linear.x, angular.z), None = publish nothing


@dataclass
class Config:
    k_lin: float = 150.0              # wheel_control's PWM per unit linear.x - must match
    forward_pwm: float = 50.0         # base PWM: W/S teleop AND the autonomous run
    max_forward_pwm: float = 255.0    # upper limit of the web UI slider
    hit_slowdown_pct: float = 20.0    # while hitting, drive this % slower than forward_pwm
    ready_delay_s: float = 0.3        # line_hit must stay 0 this long before the gantry retracts
    teleop_angular: float = 0.6       # angular.z for a held left/right key
    teleop_timeout_s: float = 0.3     # teleop deadman: browser must refresh within this
    zero_burst_s: float = 0.3         # keep sending 0 this long after teleop/stop
    vision_timeout_s: float = 1.0     # no valid FruitLineHit -> pause driving
    status_timeout_s: float = 1.0     # no /mega/status -> pause driving
    move_timeout_s: float = 20.0      # gantry must reach ready (START) / hit within this
    gripper_open: float = 90.0        # the web UI's Open / Close buttons
    gripper_closed: float = 20.0


@dataclass
class GantryStatus:
    rem: Tuple[int, int]
    servo: int
    failsafe: bool
    pos: Tuple[int, int]

    @classmethod
    def from_array(cls, data: Sequence[int]) -> 'GantryStatus':
        """[steps_left1, steps_left2, servo_deg, wheel_failsafe, pos1, pos2]."""
        if len(data) < 6:
            raise ValueError(
                f'/mega/status needs 6 values (mega_bridge with pos1/pos2), got {len(data)}'
            )
        return cls((int(data[0]), int(data[1])), int(data[2]), bool(data[3]),
                   (int(data[4]), int(data[5])))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PickController:

    def __init__(self, config: Config,
                 positions: Optional[Dict[str, Sequence[int]]] = None,
                 forward_pwm: Optional[float] = None) -> None:
        self.cfg = config
        self.positions: Dict[str, Optional[Tuple[int, int]]] = {READY: (0, 0), HIT: None}
        for name, value in (positions or {}).items():
            name = LEGACY_NAMES.get(name, name)
            if name in POSITION_NAMES and value is not None:
                self.positions[name] = (int(value[0]), int(value[1]))
        self.forward_pwm = _clamp(
            config.forward_pwm if forward_pwm is None else float(forward_pwm),
            0.0, config.max_forward_pwm)

        self.mode = IDLE
        self.running = False
        self.hits = 0
        self.message = 'idle'
        self.error = False

        self.status: Optional[GantryStatus] = None
        self._status_t = -math.inf
        self._status_n = 0
        self._vision_t = -math.inf
        self.line_hit = False

        self.gantry_target: Optional[str] = None   # READY / HIT while running
        self.drive_pwm = 0.0                        # PWM the run is driving at now
        self._clear_since: Optional[float] = None   # line_hit has been 0 since
        self._prepare_t = 0.0
        self._prepare_n = 0
        self._hit_t = 0.0                            # when the gantry was sent to HIT
        self._hit_n = 0                              # _status_n at that moment
        self.at_hit = False                          # confirmed at HIT since it was sent

        self._manual_n = -FRESH_STATUS   # _status_n when the last jog/goto was sent
        self._teleop = (0.0, 0.0)
        self._teleop_t = -math.inf
        self._zero_until = -math.inf
        self._out: List[Command] = []

    # ------------------------------------------------------------ inputs

    def on_status(self, status: GantryStatus, now: float) -> None:
        self.status = status
        self._status_t = now
        self._status_n += 1

    def on_fruit(self, valid: bool, hit: bool, now: float) -> None:
        if valid:
            self._vision_t = now
        self.line_hit = bool(valid and hit)

    def mega_ok(self, now: float) -> bool:
        return self.status is not None and now - self._status_t <= self.cfg.status_timeout_s

    def vision_ok(self, now: float) -> bool:
        return now - self._vision_t <= self.cfg.vision_timeout_s

    def gantry_idle(self) -> bool:
        return self.status is not None and self.status.rem == (0, 0)

    # ------------------------------------------------------------ user commands
    # Each returns (ok, message) for the web UI.

    def start(self, now: float) -> Tuple[bool, str]:
        if self.mode != IDLE:
            return False, 'already running'
        if not self.mega_ok(now):
            return False, 'no /mega/status - is mega_bridge running?'
        if not self.vision_ok(now):
            return False, 'no fruit vision - is the vision node in fruit_color mode?'
        if self.positions[HIT] is None:
            return False, 'save a hit position first'
        if self.forward_pwm <= 0.0:
            return False, 'drive PWM is 0 - try W/S to find a PWM, then Set speed'
        self.running = True
        self.hits = 0
        self._teleop_t = -math.inf
        self.mode = PREPARE
        self._prepare_t = now
        self._prepare_n = self._status_n
        self._send_gantry(READY)
        return self._say('started: gantry to ready')

    def stop(self, now: float, reason: str = 'stopped', error: bool = False) -> Tuple[bool, str]:
        was_moving = self.mode != IDLE
        self.running = False
        self.mode = IDLE
        self.gantry_target = None
        self.drive_pwm = 0.0
        self._teleop_t = -math.inf
        self._zero_until = now + self.cfg.zero_burst_s
        if was_moving and self.mega_ok(now) and not self.gantry_idle():
            # Retarget the gantry to where it was last reported, which halts it
            # (it decelerates, then settles back by at most ~0.1 s of travel).
            self._out.append(('goto', self.status.pos))
        return self._say(reason, error=error)

    def set_forward_pwm(self, pwm: float) -> Tuple[bool, str]:
        self.forward_pwm = _clamp(float(pwm), 0.0, self.cfg.max_forward_pwm)
        return True, f'drive PWM {self.forward_pwm:.0f}'

    def drive(self, linear: float, angular: float, now: float) -> Tuple[bool, str]:
        """Teleop. linear/angular are -1..1, scaled by forward_pwm / teleop_angular.

        W/S uses the same PWM as the autonomous run, so what you try by hand
        is exactly what START will drive at.
        """
        if self.mode != IDLE:
            return False, 'stop the run first'
        linear = _clamp(float(linear), -1.0, 1.0) * self._linear_x(self.forward_pwm)
        angular = _clamp(float(angular), -1.0, 1.0) * self.cfg.teleop_angular
        if linear == 0.0 and angular == 0.0:
            if now - self._teleop_t <= self.cfg.teleop_timeout_s:
                self._zero_until = now + self.cfg.zero_burst_s
            self._teleop_t = -math.inf
        else:
            self._teleop = (linear, angular)
            self._teleop_t = now
        return True, ''

    def jog(self, s1: int, s2: int, now: float) -> Tuple[bool, str]:
        ok, message = self._manual_ok(now)
        if ok:
            self._manual_n = self._status_n
            self._out.append(('step', (int(s1), int(s2))))
        return ok, message

    def goto(self, name: str, now: float) -> Tuple[bool, str]:
        ok, message = self._manual_ok(now)
        if not ok:
            return ok, message
        if name not in POSITION_NAMES or self.positions[name] is None:
            return False, f'no {name} position saved'
        self._manual_n = self._status_n
        self._out.append(('goto', self.positions[name]))
        return True, f'gantry to {name} {self.positions[name]}'

    def gripper(self, deg: float, now: float) -> Tuple[bool, str]:
        ok, message = self._manual_ok(now)
        if ok:
            self._out.append(('gripper', _clamp(float(deg), 0.0, 180.0)))
        return ok, message

    def save_position(self, name: str, now: float) -> Tuple[bool, str]:
        ok, message = self._manual_ok(now)
        if not ok:
            return ok, message
        if name not in POSITION_NAMES:
            return False, f'unknown position {name!r}'
        if not self.gantry_idle() or self._status_n - self._manual_n < FRESH_STATUS:
            return False, 'wait for the gantry to stop'
        self.positions[name] = self.status.pos
        return self._say(f'{name} position saved: {self.status.pos[0]}, {self.status.pos[1]}')

    # ------------------------------------------------------------ periodic

    def tick(self, now: float) -> Tuple[Base, List[Command]]:
        """Advance the state machine. Returns the base command and gantry commands."""
        if self.mode == PREPARE:
            self._prepare(now)
            base: Base = (0.0, 0.0)
        elif self.mode == DRIVE:
            base = self._drive(now)
        elif now - self._teleop_t <= self.cfg.teleop_timeout_s:
            base = self._teleop
        elif self._teleop_t != -math.inf:
            # deadman: the browser stopped refreshing a held key
            self._teleop_t = -math.inf
            self._zero_until = now + self.cfg.zero_burst_s
            base = (0.0, 0.0)
        elif now < self._zero_until:
            base = (0.0, 0.0)
        else:
            base = None
        if self.mode != IDLE:
            base = base if base is not None else (0.0, 0.0)
        out, self._out = self._out, []
        return base, out

    def snapshot(self, now: float) -> dict:
        status = self.status
        return {
            'mode': self.mode,
            'running': self.running,
            'message': self.message,
            'error': self.error,
            'hits': self.hits,
            'gantry_target': self.gantry_target,
            'at_hit': self.at_hit,
            'drive_pwm': self.drive_pwm,
            'line_hit': self.line_hit,
            'vision_ok': self.vision_ok(now),
            'mega_ok': self.mega_ok(now),
            'failsafe': bool(status and status.failsafe),
            'pos': list(status.pos) if status else None,
            'rem': list(status.rem) if status else None,
            'servo': status.servo if status else None,
            'positions': {k: (list(v) if v else None) for k, v in self.positions.items()},
            'forward_pwm': self.forward_pwm,
            'max_forward_pwm': self.cfg.max_forward_pwm,
            'hit_slowdown_pct': self.cfg.hit_slowdown_pct,
            'gripper_open': self.cfg.gripper_open,
            'gripper_closed': self.cfg.gripper_closed,
        }

    # ------------------------------------------------------------ internals

    def _say(self, message: str, error: bool = False) -> Tuple[bool, str]:
        self.message = message
        self.error = error
        return not error, message

    def _linear_x(self, pwm: float) -> float:
        """linear.x that wheel_control turns into this base PWM."""
        return pwm / self.cfg.k_lin

    def _manual_ok(self, now: float) -> Tuple[bool, str]:
        if self.mode != IDLE:
            return False, 'stop the run first'
        if not self.mega_ok(now):
            return False, 'no /mega/status - is mega_bridge running?'
        return True, ''

    def _send_gantry(self, name: str) -> None:
        self.gantry_target = name
        self._out.append(('goto', self.positions[name]))

    def _arrived(self, name: str, sent_n: int) -> bool:
        """/mega/status confirms the gantry stopped at a position sent at status count sent_n."""
        return (self.status is not None
                and self._status_n - sent_n >= FRESH_STATUS
                and self.status.rem == (0, 0)
                and self.status.pos == self.positions[name])

    def _prepare(self, now: float) -> None:
        """Wait at a standstill until the gantry has reached ready."""
        if self._arrived(READY, self._prepare_n):
            self.mode = DRIVE
            self._clear_since = now
            self._say('driving')
        elif now - self._prepare_t > self.cfg.move_timeout_s:
            self.stop(now, f'gantry did not reach ready within {self.cfg.move_timeout_s:.0f} s',
                      error=True)

    def _drive(self, now: float) -> Base:
        if not self.mega_ok(now):
            self.drive_pwm = 0.0
            self._say('paused: no /mega/status')
            return (0.0, 0.0)

        vision = self.vision_ok(now)
        # stale vision counts as "no fruit", so the gantry retracts
        hit = self.line_hit and vision
        if hit:
            self._clear_since = None
            if self.gantry_target != HIT:
                self.hits += 1
                self._send_gantry(HIT)
                self._hit_t, self._hit_n, self.at_hit = now, self._status_n, False
        else:
            if self._clear_since is None:
                self._clear_since = now
        if self.gantry_target == HIT and not self.at_hit:
            # the reach always completes before the gantry may go back
            self.at_hit = self._arrived(HIT, self._hit_n)
            if not self.at_hit and now - self._hit_t > self.cfg.move_timeout_s:
                self.stop(now, f'gantry did not reach hit within {self.cfg.move_timeout_s:.0f} s',
                          error=True)
                return (0.0, 0.0)
        if (not hit and self.gantry_target == HIT and self.at_hit
                and now - self._clear_since >= self.cfg.ready_delay_s):
            self._send_gantry(READY)

        if not vision:
            self.drive_pwm = 0.0
            self._say('paused: no fruit vision')
            return (0.0, 0.0)
        if self.gantry_target == HIT:
            slow = 1.0 - _clamp(self.cfg.hit_slowdown_pct, 0.0, 100.0) / 100.0
            self.drive_pwm = self.forward_pwm * slow
            self._say(f'hitting fruit #{self.hits} at PWM {self.drive_pwm:.0f}')
        else:
            self.drive_pwm = self.forward_pwm
            self._say(f'driving at PWM {self.drive_pwm:.0f}, looking for fruit')
        return (self._linear_x(self.drive_pwm), 0.0)
