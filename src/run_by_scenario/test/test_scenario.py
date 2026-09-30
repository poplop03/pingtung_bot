import json

import pytest

from run_by_scenario.scenario import (
    IDLE, JOYSTICK, PLAYING, RECORDING, WASD, Config, Scenario, ScenarioController,
    ScenarioStore, Step)

DT = 0.02


class Sim:
    """Controller with a fake clock and a Mega that always reports status."""

    def __init__(self, **cfg):
        cfg = {'k_lin': 100.0, 'k_ff': 40.0, 'linear_pwm': 50.0, 'angular_pwm': 20.0, **cfg}
        self.ctrl = ScenarioController(Config(**cfg))
        self.t = 0.0
        self.bases = []

    def step(self, drive=None):
        self.t += DT
        self.ctrl.on_status(False, self.t)
        if drive is not None:
            self.ctrl.drive(drive[0], drive[1], self.t)
        base = self.ctrl.tick(self.t)
        self.bases.append((self.t, base))
        return base

    def run(self, seconds, drive=None):
        for _ in range(round(seconds / DT)):
            self.step(drive)


def record(sim, name='test'):
    """1 s idle, 1 s forward, 0.5 s left turn, release, 1 s idle, stop."""
    assert sim.ctrl.record(name, 'now', sim.t)[0]
    sim.run(1.0)
    sim.run(1.0, drive=(1, 0))
    sim.run(0.5, drive=(0, 1))
    sim.ctrl.drive(0, 0, sim.t)
    sim.run(1.0)
    ok, msg = sim.ctrl.stop(sim.t)
    assert ok, msg
    return sim.ctrl.pop_recorded()


def test_record_trims_idle_and_keeps_changes():
    sim = Sim()
    scenario = record(sim)
    assert sim.ctrl.mode == IDLE
    assert [(s.linear, s.angular) for s in scenario.steps] == [(50.0, 0.0), (0.0, 20.0), (0.0, 0.0)]
    assert scenario.steps[0].t == 0.0
    assert scenario.steps[1].t == pytest.approx(1.0, abs=DT)
    assert scenario.duration == pytest.approx(1.5, abs=DT)
    assert sim.ctrl.loaded is scenario            # ready to START right away
    assert sim.ctrl.pop_recorded() is None


def test_pwms_scale_to_twist():
    sim = Sim()
    assert sim.step(drive=(1, 0)) == (0.5, 0.0)      # 50 / k_lin 100
    assert sim.step(drive=(0, -1)) == (0.0, -0.5)    # -20 / k_ff 40, right turn
    assert sim.ctrl.set_pwm(80, 40)[0]
    assert sim.step(drive=(-1, 1)) == (-0.8, 1.0)


def test_joystick_is_proportional_and_rounded():
    sim = Sim(linear_pwm=100.0, angular_pwm=40.0)
    assert sim.ctrl.set_control_mode(JOYSTICK, sim.t)[0]
    sim.step(drive=(0.504, -0.25))
    assert sim.ctrl.snapshot(sim.t)['command'] == {'linear': 50.0, 'angular': -10.0}


def test_joystick_jitter_does_not_bloat_the_recording():
    sim = Sim(linear_pwm=100.0)
    sim.ctrl.record('joy', '', sim.t)
    for i in range(50):                           # +-0.3 PWM of noise around 60
        sim.step(drive=(0.6 + (0.003 if i % 2 else -0.003), 0))
    sim.ctrl.stop(sim.t)
    scenario = sim.ctrl.pop_recorded()
    assert [s.linear for s in scenario.steps] == [60.0, 0.0]


def test_switching_control_mode_stops_a_held_command():
    sim = Sim()
    sim.step(drive=(1, 0))
    assert sim.ctrl.set_control_mode(JOYSTICK, sim.t)[0]
    assert sim.step() == (0.0, 0.0)
    assert sim.ctrl.control_mode == JOYSTICK
    assert not sim.ctrl.set_control_mode('mouse', sim.t)[0]


def test_bad_control_mode_in_config_falls_back_to_wasd():
    assert ScenarioController(Config(control_mode='nope')).control_mode == WASD
    assert ScenarioController(Config(), control_mode=JOYSTICK).control_mode == JOYSTICK


def test_recording_with_no_driving_saves_nothing():
    sim = Sim()
    sim.ctrl.record('empty', '', sim.t)
    sim.run(1.0)
    ok, msg = sim.ctrl.stop(sim.t)
    assert not ok and 'never driven' in msg
    assert sim.ctrl.pop_recorded() is None


def test_stop_while_driving_ends_with_zero():
    sim = Sim()
    sim.ctrl.record('held', '', sim.t)
    sim.run(0.5, drive=(1, 0))
    sim.ctrl.stop(sim.t)
    scenario = sim.ctrl.pop_recorded()
    assert not scenario.steps[-1].moving
    assert scenario.duration == pytest.approx(0.5, abs=DT)


def test_deadman_during_recording_is_recorded_as_stop():
    sim = Sim()
    sim.ctrl.record('deadman', '', sim.t)
    sim.run(0.5, drive=(1, 0))
    sim.run(1.0)                                  # browser stopped refreshing
    sim.run(0.5, drive=(1, 0))
    sim.ctrl.stop(sim.t)
    scenario = sim.ctrl.pop_recorded()
    assert [s.linear for s in scenario.steps] == [50.0, 0.0, 50.0, 0.0]


def test_playback_replays_the_recording():
    sim = Sim()
    scenario = record(sim)
    sim.run(1.0)
    sim.ctrl.set_pwm(200, 100)                    # playback uses the recorded PWM
    assert sim.ctrl.start(sim.t)[0]
    t0 = sim.t
    sim.bases.clear()
    sim.run(scenario.duration + 0.5)
    assert sim.ctrl.mode == IDLE and 'finished' in sim.ctrl.message
    driving = [(t - t0, b) for t, b in sim.bases if b and b != (0.0, 0.0)]
    assert driving[0][1] == (0.5, 0.0)            # 50 PWM / k_lin 100
    assert any(b == (0.0, 0.5) for _, b in driving)   # 20 PWM / k_ff 40
    assert driving[-1][0] == pytest.approx(scenario.duration, abs=2 * DT)
    assert sim.bases[-1][1] is None               # zero burst over, quiet again


def test_teleop_and_pwm_locked_while_playing():
    sim = Sim()
    record(sim)
    sim.ctrl.start(sim.t)
    assert sim.ctrl.mode == PLAYING
    assert not sim.ctrl.drive(-1, 0, sim.t)[0]
    assert not sim.ctrl.set_pwm(10, 10)[0]
    assert not sim.ctrl.set_control_mode(JOYSTICK, sim.t)[0]
    assert not sim.ctrl.record('x', '', sim.t)[0]
    sim.ctrl.stop(sim.t)
    assert sim.step() == (0.0, 0.0)


def test_playback_aborts_without_status():
    sim = Sim()
    record(sim)
    sim.ctrl.start(sim.t)
    sim.t += 2.0                                  # no status for 2 s
    assert sim.ctrl.tick(sim.t) == (0.0, 0.0)
    assert sim.ctrl.mode == IDLE and sim.ctrl.error


def test_start_needs_loaded_and_mega():
    ctrl = ScenarioController(Config())
    assert not ctrl.start(0.0)[0]
    ctrl.load(Scenario('a', [Step(0, 50, 0), Step(1, 0, 0)]))
    assert 'mega' in ctrl.start(10.0)[1]


def test_record_mode_teleop_still_drives():
    sim = Sim()
    sim.ctrl.record('x', '', sim.t)
    assert sim.ctrl.mode == RECORDING
    assert sim.step(drive=(1, 0)) == (0.5, 0.0)


def test_scenario_validation():
    with pytest.raises(ValueError):
        Scenario('../evil', [Step(0, 0, 0)])
    with pytest.raises(ValueError):
        Scenario('a', [Step(0, 50, 0)])           # does not end with 0, 0
    with pytest.raises(ValueError):
        Scenario('a', [Step(1, 50, 0), Step(0, 0, 0)])


def test_store_round_trip(tmp_path):
    store = ScenarioStore(str(tmp_path / 'sc'))
    assert store.names() == []
    original = Scenario('row_1', [Step(0, 60, 0), Step(1.234, -40, 25), Step(2, 0, 0)], 'x')
    store.save(original)
    assert store.names() == ['row_1']
    loaded = store.load('row_1', k_ff=33.0)
    assert loaded.steps == original.steps and loaded.created == 'x'
    assert loaded.at(1.5).linear == -40 and loaded.at(0.5).linear == 60
    store.delete('row_1')
    assert not store.exists('row_1')
    with pytest.raises(ValueError):
        store.path('a/b')


def test_version_1_files_convert_rad_s_to_turn_pwm(tmp_path):
    store = ScenarioStore(str(tmp_path))
    (tmp_path / 'old.json').write_text(json.dumps({'version': 1, 'name': 'old', 'steps': [
        {'t': 0.0, 'pwm': 80.0, 'angular': 0.6}, {'t': 1.0, 'pwm': 0.0, 'angular': 0.0}]}))
    scenario = store.load('old', k_ff=30.0)
    assert scenario.steps[0] == Step(0.0, 80.0, pytest.approx(18.0))
