from pick_fruit.controller import (
    DRIVE, IDLE, PREPARE, Config, GantryStatus, PickController)


class Sim:
    """Controller plus a fake gantry. A goto takes move_s (0: done by the next status)."""

    def __init__(self, move_s=0.0, **cfg):
        self.ctrl = PickController(Config(**cfg), {'ready': (0, 0), 'hit': (500, 300)})
        self.t = 0.0
        self.pos = (0, 0)
        self.target = None                   # goto in progress
        self.arrive_t = 0.0
        self.move_s = move_s
        self.servo = 90
        self.hit = False
        self.bases = []
        self.commands = []

    def step(self, dt=0.05):
        self.t += dt
        if self.target is not None and self.t >= self.arrive_t:
            self.pos, self.target = self.target, None
        rem = (0, 0) if self.target is None else (1, 1)
        self.ctrl.on_status(GantryStatus(rem, self.servo, False, self.pos), self.t)
        self.ctrl.on_fruit(True, self.hit, self.t)
        base, commands = self.ctrl.tick(self.t)
        self.bases.append(base)
        for kind, arg in commands:
            self.commands.append((kind, arg))
            if kind == 'goto':
                self.target = tuple(arg)
                self.arrive_t = self.t + self.move_s
            elif kind == 'gripper':
                self.servo = int(round(arg))
        return base

    def run(self, seconds):
        for _ in range(int(seconds / 0.05)):
            self.step()


def started(move_s=0.0, **cfg):
    """Running and driving. move_s: how long each gantry move takes from now on."""
    sim = Sim(**cfg)
    sim.step()
    assert sim.ctrl.start(sim.t)[0]
    sim.run(1.0)
    assert sim.ctrl.mode == DRIVE
    sim.move_s = move_s
    return sim


def test_start_requires_hit_position():
    sim = Sim()
    sim.ctrl.positions['hit'] = None
    sim.step()
    ok, message = sim.ctrl.start(sim.t)
    assert not ok and 'hit' in message
    assert sim.ctrl.mode == IDLE


def test_start_requires_status_and_vision():
    ctrl = PickController(Config(), {'hit': (1, 1)})
    assert not ctrl.start(0.0)[0]


def test_legacy_pick_position_loads_as_hit():
    ctrl = PickController(Config(), {'ready': (1, 2), 'pick': (3, 4)})
    assert ctrl.positions == {'ready': (1, 2), 'hit': (3, 4)}


def test_start_prepares_then_drives_forward():
    sim = Sim(forward_pwm=90.0, k_lin=150.0)
    sim.pos = (9, 9)
    sim.step()
    sim.ctrl.start(sim.t)
    assert sim.step() == (0.0, 0.0)
    assert sim.ctrl.mode == PREPARE
    sim.run(1.0)
    assert sim.ctrl.mode == DRIVE
    assert sim.step() == (0.6, 0.0)          # 90 PWM / k_lin 150
    assert sim.commands == [('goto', (0, 0))]


def test_line_hit_slows_down_and_reaches_out():
    sim = started(forward_pwm=100.0, k_lin=100.0, hit_slowdown_pct=20.0)
    sim.commands.clear()
    sim.hit = True
    assert sim.step() == (0.8, 0.0)          # 20% slower, never stops
    assert sim.commands == [('goto', (500, 300))]
    sim.run(1.0)                             # still on the line: no repeat goto
    assert sim.commands == [('goto', (500, 300))]
    assert sim.bases[-1] == (0.8, 0.0)
    assert sim.ctrl.hits == 1


def test_line_clear_retracts_after_delay():
    sim = started(forward_pwm=100.0, k_lin=100.0, ready_delay_s=0.3)
    sim.hit = True
    sim.step()
    sim.commands.clear()
    sim.hit = False
    sim.run(0.2)
    assert sim.commands == []                # a flicker does not retract
    assert sim.bases[-1] == (0.8, 0.0)
    sim.run(0.2)
    assert sim.commands == [('goto', (0, 0))]
    assert sim.bases[-1] == (1.0, 0.0)       # full speed again


def test_flicker_keeps_gantry_out():
    sim = started(ready_delay_s=0.3)
    sim.hit = True
    sim.step()
    sim.commands.clear()
    for _ in range(10):                      # 0 for one frame, then 1 again
        sim.hit = False
        sim.step()
        sim.hit = True
        sim.step()
    assert sim.commands == []
    assert sim.ctrl.hits == 1


def test_each_fruit_counts_once():
    sim = started(ready_delay_s=0.1)
    for _ in range(3):
        sim.hit = True
        sim.run(0.5)
        sim.hit = False
        sim.run(0.5)
    assert sim.ctrl.hits == 3


def test_short_hit_still_reaches_hit_before_retracting():
    sim = started(ready_delay_s=0.3, move_s=1.0)
    sim.commands.clear()
    sim.hit = True
    sim.step()
    sim.hit = False                          # fruit gone long before the gantry arrives
    sim.run(0.8)
    assert sim.commands == [('goto', (500, 300))]
    assert not sim.ctrl.at_hit
    sim.run(0.4)                             # arrived, and clear for > ready_delay_s
    assert sim.ctrl.at_hit
    assert sim.commands == [('goto', (500, 300)), ('goto', (0, 0))]
    assert sim.pos == (500, 300)             # it left from the hit position


def test_retract_waits_for_fresh_status_at_hit():
    # a status from before the goto arrived (still at ready, 0 left) must not count
    sim = started(ready_delay_s=0.0, move_s=0.0)
    sim.commands.clear()
    sim.hit = True
    sim.step()
    sim.hit = False
    sim.step()
    assert sim.commands == [('goto', (500, 300))]
    sim.run(0.2)
    assert sim.commands[-1] == ('goto', (0, 0))


def test_hit_not_reached_aborts_run():
    sim = started(move_timeout_s=1.0, move_s=5.0)
    sim.hit = True
    sim.run(1.2)
    assert sim.ctrl.mode == IDLE and sim.ctrl.error
    assert 'hit' in sim.ctrl.message


def test_stop_while_hitting_halts_everything():
    sim = started()
    sim.hit = True
    sim.step()
    sim.ctrl.stop(sim.t)
    assert sim.ctrl.mode == IDLE and not sim.ctrl.running
    assert sim.step() == (0.0, 0.0)          # zero burst
    sim.run(1.0)
    assert sim.bases[-1] is None             # then silent


def test_prepare_timeout_faults():
    sim = Sim(move_timeout_s=1.0)
    sim.step()
    sim.ctrl.start(sim.t)
    # gantry never arrives: keep reporting another position
    for _ in range(40):
        sim.t += 0.05
        sim.ctrl.on_status(GantryStatus((0, 0), 90, False, (7, 7)), sim.t)
        sim.ctrl.on_fruit(True, False, sim.t)
        sim.ctrl.tick(sim.t)
    assert sim.ctrl.mode == IDLE and sim.ctrl.error


def test_prepare_waits_for_fresh_status():
    """Ready is only reached after FRESH_STATUS statuses, even if pos already matches."""
    sim = Sim()
    sim.step()
    sim.ctrl.start(sim.t)
    sim.ctrl.tick(sim.t)                     # no new status arrives
    sim.ctrl.tick(sim.t)
    assert sim.ctrl.mode == PREPARE


def test_pause_driving_without_vision():
    sim = started()
    sim.hit = True
    sim.step()
    sim.commands.clear()
    sim.t += 2.0
    sim.ctrl.on_status(GantryStatus((0, 0), 90, False, (500, 300)), sim.t)
    base, commands = sim.ctrl.tick(sim.t)
    assert base == (0.0, 0.0)
    assert 'vision' in sim.ctrl.message
    sim.t += 0.5
    sim.ctrl.on_status(GantryStatus((0, 0), 90, False, (500, 300)), sim.t)
    base, commands = sim.ctrl.tick(sim.t)
    assert base == (0.0, 0.0)
    assert commands == [('goto', (0, 0))]    # stale vision retracts the gantry


def test_teleop_deadman():
    sim = Sim(forward_pwm=30.0, k_lin=150.0, teleop_angular=0.5)
    sim.step()
    sim.ctrl.drive(1, -1, sim.t)
    assert sim.step() == (0.2, -0.5)
    sim.run(0.4)                    # no refresh
    assert sim.bases[-1] in ((0.0, 0.0), None)
    sim.run(0.5)
    assert sim.bases[-1] is None


def test_manual_commands_refused_while_running():
    sim = started()
    assert not sim.ctrl.jog(10, 0, sim.t)[0]
    assert not sim.ctrl.drive(1, 0, sim.t)[0]
    assert not sim.ctrl.save_position('hit', sim.t)[0]


def test_save_position_uses_reported_pos():
    sim = Sim()
    sim.pos = (123, -45)
    sim.step()
    ok, _ = sim.ctrl.save_position('hit', sim.t)
    assert ok and sim.ctrl.positions['hit'] == (123, -45)


def test_status_array_needs_positions():
    assert GantryStatus.from_array([1, 2, 90, 0, 5, 6]).pos == (5, 6)
    try:
        GantryStatus.from_array([1, 2, 90, 0])
    except ValueError:
        pass
    else:
        raise AssertionError('short status accepted')


def test_save_waits_for_status_after_jog():
    sim = Sim()
    sim.step()
    sim.ctrl.jog(100, 0, sim.t)
    assert not sim.ctrl.save_position('hit', sim.t)[0]
    sim.step()
    sim.step()
    assert sim.ctrl.save_position('hit', sim.t)[0]


def test_teleop_and_run_share_the_drive_pwm():
    sim = Sim(k_lin=100.0)
    sim.step()
    sim.ctrl.set_forward_pwm(60)
    sim.ctrl.drive(1, 0, sim.t)
    assert sim.step() == (0.6, 0.0)
    sim.ctrl.set_forward_pwm(90)            # browser resends the held key
    sim.ctrl.drive(1, 0, sim.t)
    assert sim.step() == (0.9, 0.0)
    sim.ctrl.drive(0, 0, sim.t)
    sim.ctrl.start(sim.t)
    sim.run(1.0)
    assert sim.step() == (0.9, 0.0)


def test_start_refuses_zero_pwm():
    sim = Sim()
    sim.step()
    sim.ctrl.set_forward_pwm(0)
    ok, message = sim.ctrl.start(sim.t)
    assert not ok and 'PWM' in message
