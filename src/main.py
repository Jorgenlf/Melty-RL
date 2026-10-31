import numpy as np
import pygame

import src.control as control
import src.geomutils as geom
from src.bot import Bot
from src.viz import View

DT = 1e-4                  # 150 substeps/rev at 4000 RPM, 300 at 2000 RPM
RENDER_HZ = 60
FRAME_SUBSTEPS = 167       # fixed count for the M key, so stepping is reproducible
MAX_SUBSTEPS = 4000        # ceiling so a stalled frame cannot spiral
STEER_RATE = np.pi         # rad/s that left/right precess the virtual front
DRIVE_PORTION = 0.5        # motor-on fraction while driving; 0.5 is max translation
MIN_PORTION = 0.05
PORTION_RATE = 0.5         # per second that [ and ] trim the driving portion


def read_command(keys, psi_ref, portion, frame_dt):
    """Steers the virtual front, then builds cmd = [psi_d, portion, spin].

    The front is a reference heading, not a physical feature: the chassis spins
    continuously and only the LEDs mark where the front points.
    """
    turn = float(keys[pygame.K_RIGHT]) - float(keys[pygame.K_LEFT])
    psi_ref = geom.ssa(psi_ref - turn*STEER_RATE*frame_dt)

    trim = float(keys[pygame.K_RIGHTBRACKET]) - float(keys[pygame.K_LEFTBRACKET])
    portion = min(1.0, max(MIN_PORTION, portion + trim*PORTION_RATE*frame_dt))

    drive = float(keys[pygame.K_UP]) - float(keys[pygame.K_DOWN])
    psi_d = psi_ref if drive >= 0.0 else geom.ssa(psi_ref + np.pi)

    spin = 0.0
    if keys[pygame.K_SPACE]:
        spin = 1.0
    elif keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
        spin = -1.0

    # Powering the whole revolution is what suppresses translation, so not
    # driving means portion 1 rather than a narrowed arc pointed nowhere.
    cmd = np.array([psi_d, portion if drive != 0.0 else 1.0, spin])
    return cmd, psi_ref, portion


def main():
    pygame.init()
    view = View()
    clock = pygame.time.Clock()

    bot = Bot(DT)
    psi_ref = 0.0
    portion = DRIVE_PORTION
    paused = False
    pending = 0
    accumulator = 0.0
    running = True

    while running:
        # Clamped to the physics the frame can actually run: a stalled frame
        # advances MAX_SUBSTEPS of sim time, so steering must not precess by
        # the full wall-clock gap or the front jumps on recovery.
        frame_dt = min(clock.tick(RENDER_HZ)/1000.0, MAX_SUBSTEPS*DT)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_c:
                    view.follow = not view.follow
                elif event.key == pygame.K_p:
                    paused = not paused
                    accumulator = 0.0
                elif event.key == pygame.K_n and paused:
                    pending += 1
                elif event.key == pygame.K_m and paused:
                    pending += FRAME_SUBSTEPS
                elif event.key == pygame.K_r:
                    bot = Bot(DT)
                    psi_ref = 0.0
                    portion = DRIVE_PORTION
                    accumulator = 0.0
                    pending = 0
                    view.reset()

        keys = pygame.key.get_pressed()
        cmd, psi_ref, portion = read_command(keys, psi_ref, portion, frame_dt)

        if paused:
            substeps, pending = pending, 0
        else:
            accumulator += frame_dt
            substeps = int(accumulator/DT)
            if substeps > MAX_SUBSTEPS:
                # Drop the backlog rather than fast-forwarding for the next
                # several frames; a stall should cost sim time, not replay it.
                substeps = MAX_SUBSTEPS
                accumulator = 0.0
            else:
                accumulator -= substeps*DT

        if substeps > 0:
            traj = bot.run(control.closed_loop, cmd, substeps)
            view.push_trail(traj)
            view.update_leds(traj[:, 2], psi_ref)
            view.update_gates(control.gate_fractions(traj[:, 2], traj[:, 5], cmd))

        view.draw(bot, cmd, psi_ref, portion, paused, substeps,
                  control.translating(bot.omega, cmd))

    pygame.quit()


if __name__ == "__main__":
    main()
