"""Record a teleop drive as a scenario, and play it back. No ROS in here, so it can be unit tested.

    IDLE ──record──► RECORDING ──stop──► IDLE  (the take becomes a Scenario, saved by the node)
      │                (teleop is recorded)
      └───start────► PLAYING ──end of script / stop / fault──► IDLE
                       (the loaded Scenario drives the base, teleop is locked)

Teleop is W/A/S/D or an on-screen joystick (control_mode). Both send
linear/angular in -1..1, scaled by two PWMs:

    linear_pwm   base PWM at full forward         sent as linear.x  = pwm / k_lin
    angular_pwm  differential PWM at full turn    sent as angular.z = pwm / k_ff

k_lin and k_ff are wheel_control's, so wheel_control turns the command back
into the same base PWM and the same turn feedforward. Its heading hold and
yaw-rate loop still run, during teleop and playback alike.

A scenario is what the base was actually commanded, in PWM, as change points
in time:

    t [s]   linear (base PWM, − = back)   angular (turn PWM, + = left)
    0.00    60                            0
    2.35    60                            20
    3.10    0                             0      <- the last step is always 0, 0: the end

Each step holds until the next one. Idle time before the first command and
after the last release is trimmed, so a scenario starts moving as soon as
START is pressed.

Time is any monotonic clock in seconds, passed in by the caller.
"""

from __future__ import annotations

import bisect
import json
import math
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

IDLE = 'idle'
RECORDING = 'recording'
PLAYING = 'playing'

WASD = 'wasd'
JOYSTICK = 'joystick'
CONTROL_MODES = (WASD, JOYSTICK)

FORMAT_VERSION = 2
NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')

Base = Optional[Tuple[float, float]]   # (linear.x, angular.z), None = publish nothing


@dataclass
class Config:
    k_lin: float = 150.0              # wheel_control's PWM per unit linear.x - must match
    k_ff: float = 33.0                # wheel_control's PWM per rad/s (feedforward) - must match
    linear_pwm: float = 50.0          # base PWM at full W/S or full joystick forward
    angular_pwm: float = 20.0         # turn PWM at full A/D or full joystick sideways
    max_linear_pwm: float = 255.0     # slider limit, also clamps playback
    max_angular_pwm: float = 255.0
    control_mode: str = WASD          # wasd | joystick, until the UI changes it
    teleop_timeout_s: float = 0.3     # teleop deadman: browser must refresh within this
    zero_burst_s: float = 0.3         # keep sending 0 this long after teleop/stop
    status_timeout_s: float = 1.0     # no /mega/status -> refuse START, abort playback


@dataclass(frozen=True)
class Step:
    t: float
    linear: float      # base PWM, − = backwards
    angular: float     # turn PWM, + = left

    @property
    def moving(self) -> bool:
        return self.linear != 0.0 or self.angular != 0.0


@dataclass
class Scenario:
    name: str
    steps: List[Step]
    created: str = ''

    def __post_init__(self) -> None:
        if not valid_name(self.name):
            raise ValueError(f'bad name {self.name!r}: use letters, digits, _ and -')
        if not self.steps:
            raise ValueError('no steps')
        last_t = 0.0
        for s in self.steps:
            if not all(math.isfinite(v) for v in (s.t, s.linear, s.angular)):
                raise ValueError(f'step at t={s.t} is not a number')
            if s.t < last_t:
                raise ValueError(f'step times must not go backwards (t={s.t})')
            last_t = s.t
        if self.steps[-1].moving:
            raise ValueError('the last step must be linear 0, angular 0 (it marks the end)')
        self._times = [s.t for s in self.steps]

    @property
    def duration(self) -> float:
        return self.steps[-1].t

    def at(self, t: float) -> Step:
        """The step in force at time t (the first one before it starts)."""
        i = bisect.bisect_right(self._times, t) - 1
        return self.steps[max(i, 0)]

    def to_dict(self) -> dict:
        return {
            'version': FORMAT_VERSION,
            'name': self.name,
            'created': self.created,
            'duration_s': round(self.duration, 3),
            'steps': [{'t': round(s.t, 3), 'linear': round(s.linear, 1),
                       'angular': round(s.angular, 1)} for s in self.steps],
        }

    @classmethod
    def from_dict(cls, data: dict, k_ff: float) -> 'Scenario':
        """k_ff converts version 1 files, whose angular was in rad/s."""
        if not isinstance(data, dict) or not isinstance(data.get('steps'), list):
            raise ValueError('not a scenario file (no "steps" list)')
        if int(data.get('version', 1)) == 1:
            steps = [Step(float(s['t']), float(s['pwm']), float(s['angular']) * k_ff)
                     for s in data['steps']]
        else:
            steps = [Step(float(s['t']), float(s['linear']), float(s['angular']))
                     for s in data['steps']]
        return cls(str(data.get('name', '')), steps, str(data.get('created', '')))

    def summary(self) -> dict:
        return {'name': self.name, 'created': self.created,
                'duration': self.duration, 'steps': len(self.steps)}


def valid_name(name: str) -> bool:
    return bool(NAME_RE.match(name or ''))


class ScenarioStore:
    """One JSON file per scenario in a directory: <dir>/<name>.json."""

    def __init__(self, directory: str) -> None:
        self.directory = os.path.expanduser(directory)

    def path(self, name: str) -> str:
        if not valid_name(name):
            raise ValueError(f'bad name {name!r}: use letters, digits, _ and -')
        return os.path.join(self.directory, name + '.json')

    def names(self) -> List[str]:
        try:
            files = os.listdir(self.directory)
        except FileNotFoundError:
            return []
        return sorted(f[:-5] for f in files if f.endswith('.json') and valid_name(f[:-5]))

    def exists(self, name: str) -> bool:
        return os.path.exists(self.path(name))

    def load(self, name: str, k_ff: float) -> Scenario:
        with open(self.path(name)) as f:
            scenario = Scenario.from_dict(json.load(f), k_ff)
        scenario.name = name           # the file name wins over a hand-edited "name"
        return scenario

    def save(self, scenario: Scenario) -> str:
        path = self.path(scenario.name)
        os.makedirs(self.directory, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(scenario.to_dict(), f, indent=1)
        os.replace(tmp, path)
        return path

    def delete(self, name: str) -> None:
        os.remove(self.path(name))


class ScenarioController:

    def __init__(self, config: Config, linear_pwm: Optional[float] = None,
                 angular_pwm: Optional[float] = None,
                 control_mode: Optional[str] = None) -> None:
        self.cfg = config
        self.linear_pwm = _clamp(
            config.linear_pwm if linear_pwm is None else float(linear_pwm),
            0.0, config.max_linear_pwm)
        self.angular_pwm = _clamp(
            config.angular_pwm if angular_pwm is None else float(angular_pwm),
            0.0, config.max_angular_pwm)
        mode = config.control_mode if control_mode is None else control_mode
        self.control_mode = mode if mode in CONTROL_MODES else WASD
        self.mode = IDLE
        self.message = 'ready'
        self.error = False
        self.loaded: Optional[Scenario] = None
        self.failsafe = False

        self._status_t = -math.inf
        self._teleop = (0.0, 0.0)        # (linear, angular) PWM being commanded by the user
        self._teleop_t = -math.inf
        self._zero_until = -math.inf
        self._command = (0.0, 0.0)       # (linear, angular) PWM sent on the last tick, for the UI

        self._rec_name = ''
        self._rec_created = ''
        self._rec_t0 = 0.0
        self._rec_steps: List[Step] = []
        self._recorded: Optional[Scenario] = None
        self._play_t0 = 0.0
        self._elapsed = 0.0

    # ------------------------------------------------------------ inputs

    def on_status(self, failsafe: bool, now: float) -> None:
        self.failsafe = failsafe
        self._status_t = now

    def mega_ok(self, now: float) -> bool:
        return now - self._status_t <= self.cfg.status_timeout_s

    # ------------------------------------------------------------ user commands
    # Each returns (ok, message) for the web UI.

    def set_pwm(self, linear: float, angular: float) -> Tuple[bool, str]:
        if self.mode == PLAYING:
            return False, 'playback uses the recorded PWM - stop it first'
        self.linear_pwm = _clamp(float(linear), 0.0, self.cfg.max_linear_pwm)
        self.angular_pwm = _clamp(float(angular), 0.0, self.cfg.max_angular_pwm)
        return True, f'PWM linear {self.linear_pwm:.0f}, angular {self.angular_pwm:.0f}'

    def set_control_mode(self, mode: str, now: float) -> Tuple[bool, str]:
        if mode not in CONTROL_MODES:
            return False, f'unknown control mode {mode!r}'
        if self.mode == PLAYING:
            return False, 'a scenario is playing - stop it first'
        self.control_mode = mode
        self.drive(0.0, 0.0, now)      # never carry a held command across the switch
        return True, f'control: {mode}'

    def drive(self, linear: float, angular: float, now: float) -> Tuple[bool, str]:
        """Teleop. linear/angular are -1..1 (+ = forward / left), scaled by the PWMs."""
        if self.mode == PLAYING:
            return False, 'a scenario is playing - stop it first'
        lin = round(_clamp(float(linear), -1.0, 1.0) * self.linear_pwm)
        ang = round(_clamp(float(angular), -1.0, 1.0) * self.angular_pwm)
        if lin == 0 and ang == 0:
            if now - self._teleop_t <= self.cfg.teleop_timeout_s:
                self._zero_until = now + self.cfg.zero_burst_s
            self._teleop_t = -math.inf
        else:
            self._teleop = (float(lin), float(ang))
            self._teleop_t = now
        return True, ''

    def record(self, name: str, created: str, now: float) -> Tuple[bool, str]:
        if self.mode != IDLE:
            return False, f'already {self.mode}'
        if not valid_name(name):
            return False, f'bad name {name!r}: use letters, digits, _ and -'
        self.mode = RECORDING
        self._rec_name = name
        self._rec_created = created
        self._rec_t0 = now
        self._rec_steps = []
        self._elapsed = 0.0
        return self._say(f'recording "{name}" - drive, then STOP to save')

    def load(self, scenario: Scenario) -> Tuple[bool, str]:
        if self.mode != IDLE:
            return False, f'stop first ({self.mode})'
        self.loaded = scenario
        return self._say(f'loaded "{scenario.name}" ({scenario.duration:.1f} s)')

    def start(self, now: float) -> Tuple[bool, str]:
        if self.mode != IDLE:
            return False, f'already {self.mode}'
        if self.loaded is None:
            return False, 'load a scenario first'
        if not self.mega_ok(now):
            return False, 'no /mega/status - is mega_bridge running?'
        self.mode = PLAYING
        self._teleop_t = -math.inf
        self._play_t0 = now
        self._elapsed = 0.0
        return self._say(f'playing "{self.loaded.name}"')

    def stop(self, now: float, reason: str = 'stopped', error: bool = False) -> Tuple[bool, str]:
        """Stop the base. Ends a recording (pop_recorded() then has it) or a playback."""
        was = self.mode
        self.mode = IDLE
        self._teleop_t = -math.inf
        self._zero_until = now + self.cfg.zero_burst_s
        if was == RECORDING:
            self._rec_add(now, 0.0, 0.0)
            try:
                self._recorded = self._finish_recording()
            except ValueError as exc:
                return self._say(f'nothing saved: {exc}', error=True)
            self.loaded = self._recorded
            d = self._recorded.duration
            return self._say(f'recorded "{self._rec_name}" ({d:.1f} s)')
        return self._say(reason, error=error)

    def pop_recorded(self) -> Optional[Scenario]:
        recorded, self._recorded = self._recorded, None
        return recorded

    # ------------------------------------------------------------ periodic

    def tick(self, now: float) -> Base:
        """Advance one control period. Returns (linear.x, angular.z) to publish, or None."""
        if self.mode == PLAYING:
            self._elapsed = now - self._play_t0
            if not self.mega_ok(now):
                self.stop(now, 'aborted: lost /mega/status', error=True)
                return self._out(0.0, 0.0)
            if self._elapsed >= self.loaded.duration:
                self.stop(now, f'finished "{self.loaded.name}"')
                return self._out(0.0, 0.0)
            step = self.loaded.at(self._elapsed)
            lin_max, ang_max = self.cfg.max_linear_pwm, self.cfg.max_angular_pwm
            return self._out(_clamp(step.linear, -lin_max, lin_max),
                             _clamp(step.angular, -ang_max, ang_max))

        if now - self._teleop_t <= self.cfg.teleop_timeout_s:
            lin, ang = self._teleop
        else:
            if self._teleop_t != -math.inf:
                # deadman: the browser stopped refreshing a held key / the joystick
                self._teleop_t = -math.inf
                self._zero_until = now + self.cfg.zero_burst_s
            lin, ang = 0.0, 0.0

        if self.mode == RECORDING:
            self._elapsed = now - self._rec_t0
            self._rec_add(now, lin, ang)

        if lin != 0.0 or ang != 0.0 or now < self._zero_until or self.mode != IDLE:
            return self._out(lin, ang)
        self._command = (0.0, 0.0)
        return None

    def snapshot(self, now: float) -> dict:
        return {
            'mode': self.mode,
            'message': self.message,
            'error': self.error,
            'mega_ok': self.mega_ok(now),
            'failsafe': self.failsafe,
            'control_mode': self.control_mode,
            'linear_pwm': self.linear_pwm,
            'angular_pwm': self.angular_pwm,
            'max_linear_pwm': self.cfg.max_linear_pwm,
            'max_angular_pwm': self.cfg.max_angular_pwm,
            'command': {'linear': self._command[0], 'angular': self._command[1]},
            'elapsed': self._elapsed if self.mode != IDLE else None,
            'recording': self._rec_name if self.mode == RECORDING else None,
            'loaded': self.loaded.summary() if self.loaded else None,
        }

    # ------------------------------------------------------------ internals

    def _out(self, lin: float, ang: float) -> Tuple[float, float]:
        self._command = (lin, ang)
        return lin / self.cfg.k_lin, ang / self.cfg.k_ff

    def _rec_add(self, now: float, lin: float, ang: float) -> None:
        if self._rec_steps and (self._rec_steps[-1].linear, self._rec_steps[-1].angular) == (lin, ang):
            return
        self._rec_steps.append(Step(now - self._rec_t0, lin, ang))

    def _finish_recording(self) -> Scenario:
        steps = self._rec_steps
        first = next((i for i, s in enumerate(steps) if s.moving), None)
        if first is None:
            raise ValueError('the robot was never driven')
        t0 = steps[first].t
        steps = [Step(s.t - t0, s.linear, s.angular) for s in steps[first:]]
        # every change is recorded, so the list alternates to a final (0, 0):
        # that step is where the user let go for the last time
        return Scenario(self._rec_name, steps, self._rec_created)

    def _say(self, message: str, error: bool = False) -> Tuple[bool, str]:
        self.message = message
        self.error = error
        return not error, message


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
