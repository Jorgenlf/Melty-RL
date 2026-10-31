"""Headless physics checks. No pygame, no display.

Run with `uv run pytest -q` from the repository root.
"""

import numpy as np
import pytest

from src import control, motor
from src import state_space as ss
from src.bot import Bot
from src.rksolver import simulate

DT = 1e-4
FULL_SPIN = np.array([0.0, 1.0, 1.0])


def settle(cmd, seconds=60.0, dt=DT):
    """Runs a command to steady state and returns the bot."""
    bot = Bot(dt)
    chunk = int(10.0/dt)
    for _ in range(int(seconds/10.0)):
        bot.run(control.closed_loop, cmd, chunk)
    return bot


def course_error(cmd, seconds=20.0):
    """Angle between actual drift and commanded direction, in degrees."""
    bot = settle(cmd, seconds)
    start = bot.position.copy()
    bot.run(control.closed_loop, cmd, int(5.0/DT))
    d = bot.position - start
    return float(np.degrees(np.arctan2(d[1], d[0]) - cmd[0]))


def test_terminal_spin_matches_documented_figure():
    bot = settle(FULL_SPIN, 30.0)
    assert bot.omega*30/np.pi == pytest.approx(4002.5, abs=5.0)


def test_terminal_spin_is_below_the_no_load_ceiling():
    """Back-EMF, not b_omega alone, must be what caps spin."""
    kt, R, V, bw = motor.k_t, motor.R_motor, motor.V_bat, motor.b_w
    shaft_free = kt*V/R/(bw + kt*kt/R)
    ceiling_rpm = shaft_free*motor.r_wheel/ss.r_w*30/np.pi

    assert ceiling_rpm == pytest.approx(4413.0, abs=2.0)
    assert settle(FULL_SPIN, 30.0).omega*30/np.pi < ceiling_rpm


def test_translation_tracks_the_commanded_heading():
    """PHASE_LEAD is a calibration; this is what pins it."""
    assert abs(course_error(np.array([0.0, 0.5, 1.0]))) < 5.0


def test_translation_follows_a_rotated_command():
    assert abs(course_error(np.array([np.pi/2, 0.5, 1.0]))) < 5.0


def test_zero_spin_coasts_rather_than_braking():
    """Gating on at zero duty would regen brake and thrust the bot backwards."""
    u = control.wheel_duties(0.0, 400.0, np.array([0.0, 0.5, 0.0]))
    assert np.all(u == 0.0)

    bot = settle(FULL_SPIN, 20.0)
    before = bot.omega
    start = bot.position.copy()
    bot.run(control.closed_loop, np.array([0.0, 0.5, 0.0]), int(0.5/DT))
    travelled = np.linalg.norm(bot.position - start)

    assert 0.0 < bot.omega < before       # coasting down, not braking hard
    assert travelled < 0.05               # and not being thrust anywhere


def test_slow_spin_forces_both_motors_on():
    """Below MIN_TRANSLATION_OMEGA the lead calibration does not hold."""
    slow = control.MIN_TRANSLATION_OMEGA*0.5
    u = control.wheel_duties(0.0, slow, np.array([0.0, 0.2, 1.0]))
    assert u[2] == 1.0 and u[3] == 1.0


def test_portion_one_produces_no_translation():
    bot = settle(FULL_SPIN, 30.0)
    assert np.linalg.norm(bot.velocity) < 1e-3


def test_lower_portion_trades_spin_for_speed():
    full = settle(FULL_SPIN, 60.0)
    half = settle(np.array([0.0, 0.5, 1.0]), 60.0)

    assert half.omega < full.omega
    assert np.linalg.norm(half.velocity) > np.linalg.norm(full.velocity)


def test_motor_current_is_clamped_at_the_esc_limit():
    """Stall current is 247 A, so the clamp is what binds, not resistance."""
    w_dot, force = motor.wheel_dot(0.0, 1.0, 1.0, 0.0)
    torque = w_dot*motor.J_w + motor.r_wheel*force
    assert torque == pytest.approx(motor.k_t*motor.i_max, rel=1e-6)


def test_coasting_wheel_carries_no_current():
    w_dot, _ = motor.wheel_dot(100.0, 1.0, 0.0, 100.0*motor.r_wheel)
    assert w_dot == pytest.approx(-motor.b_w*100.0/motor.J_w, rel=1e-9)


def test_gate_fractions_average_to_the_commanded_portion():
    """The HUD reads this, so it has to mean what it says."""
    bot = settle(np.array([0.0, 0.5, 1.0]), 30.0)
    cmd = np.array([0.0, 0.5, 1.0])
    traj = bot.run(control.closed_loop, cmd, 2000)
    gates = control.gate_fractions(traj[:, 2], traj[:, 5], cmd)
    assert gates[0] == pytest.approx(0.5, abs=0.05)
    assert gates[1] == pytest.approx(0.5, abs=0.05)


def test_timestep_is_converged():
    """1e-4 must agree with half that, or the drive square wave is aliased."""
    x0 = np.zeros(8)
    x0[5] = 400.0
    cmd = np.array([0.5, 0.5, 1.0])

    coarse = simulate(control.closed_loop, x0, cmd, DT, 10000)[-1]
    fine = simulate(control.closed_loop, x0, cmd, DT/2, 20000)[-1]

    assert np.max(np.abs(coarse[:2] - fine[:2])) < 1e-3
    assert coarse[5] == pytest.approx(fine[5], rel=1e-4)
