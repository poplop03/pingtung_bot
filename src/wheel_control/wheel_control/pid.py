"""PI(D) controller sized for a yaw-rate loop on a noisy MEMS gyro.

Three things here that a textbook PID usually leaves out and that matter on
this robot:

  * derivative on MEASUREMENT, not on error - a step in the setpoint would
    otherwise produce an impulse straight through to the wheels
  * the derivative term is low-pass filtered - raw BNO055 gyro noise
    differentiated by dt is mostly noise amplification
  * conditional integration for anti-windup - with base_pwm=80 the
    differential term saturates at +/-175, and a wound-up integrator is the
    classic cause of "it overshoots the turn then wanders back"
"""

from dataclasses import dataclass, field


@dataclass
class PID:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    out_min: float = -175.0
    out_max: float = 175.0

    # Derivative low-pass time constant, seconds. 0 disables filtering.
    # Rule of thumb: 3-10x the control period. At 50 Hz, 0.05-0.15 s.
    tau: float = 0.08

    # Hard clamp on the integral term's contribution, in output units.
    # Keep it well under the output range so I alone can never saturate.
    i_limit: float = 90.0

    _integral: float = field(default=0.0, init=False)
    _prev_meas: float = field(default=0.0, init=False)
    _d_filt: float = field(default=0.0, init=False)
    _first: bool = field(default=True, init=False)

    # exposed for logging / tuning
    p_term: float = field(default=0.0, init=False)
    i_term: float = field(default=0.0, init=False)
    d_term: float = field(default=0.0, init=False)
    saturated: bool = field(default=False, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_meas = 0.0
        self._d_filt = 0.0
        self._first = True
        self.p_term = self.i_term = self.d_term = 0.0
        self.saturated = False

    def update(self, setpoint: float, measurement: float, dt: float,
               feedforward: float = 0.0) -> float:
        if dt <= 0.0:
            return self._clamp(feedforward + self.p_term + self.i_term + self.d_term)[0]

        error = setpoint - measurement

        self.p_term = self.kp * error

        # --- derivative on measurement, filtered ---
        if self._first:
            self._prev_meas = measurement
            self._first = False
        raw_d = -(measurement - self._prev_meas) / dt   # note the sign
        self._prev_meas = measurement
        if self.tau > 0.0:
            alpha = dt / (self.tau + dt)
            self._d_filt += alpha * (raw_d - self._d_filt)
        else:
            self._d_filt = raw_d
        self.d_term = self.kd * self._d_filt

        # --- tentative output, then decide whether to integrate ---
        candidate = feedforward + self.p_term + self.i_term + self.d_term
        _, would_saturate = self._clamp(candidate)

        # Conditional integration: stop winding up if we are already pinned
        # against a limit AND the error would push us further into it.
        pushing_out = (candidate > self.out_max and error > 0) or \
                      (candidate < self.out_min and error < 0)
        if not pushing_out:
            self._integral += error * dt
            i_raw = self.ki * self._integral
            if i_raw > self.i_limit:
                i_raw = self.i_limit
                self._integral = self.i_limit / self.ki if self.ki else 0.0
            elif i_raw < -self.i_limit:
                i_raw = -self.i_limit
                self._integral = -self.i_limit / self.ki if self.ki else 0.0
            self.i_term = i_raw

        out, self.saturated = self._clamp(
            feedforward + self.p_term + self.i_term + self.d_term)
        return out

    def _clamp(self, value: float):
        if value > self.out_max:
            return self.out_max, True
        if value < self.out_min:
            return self.out_min, True
        return value, False
