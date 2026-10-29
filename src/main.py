import numpy as np
import pygame

import src.control as control
import src.geomutils as geom
import src.state_space as ss
from src.bot import Bot
from src.viz import View

DT = 1e-4                  # 150 substeps/rev at 4000 RPM, 300 at 2000 RPM
RENDER_HZ = 60
FRAME_SUBSTEPS = 167       # fixed count for the M key, so stepping is reproducible
MAX_SUBSTEPS = 4000        # ceiling so a stalled frame cannot spiral
STEER_RATE = np.pi         # rad/s that left/right precess the virtual front


def read_command(keys, psi_ref, frame_dt):
    """Steers the virtual front, then builds cmd = [psi_d, throttle, tau_cmd].

    The front is a reference heading, not a physical feature: the chassis spins
    continuously and only the LEDs mark where the front points.
    """
    turn = float(keys[pygame.K_RIGHT]) - float(keys[pygame.K_LEFT])
    psi_ref = geom.ssa(psi_ref - turn*STEER_RATE*frame_dt)

    drive = float(keys[pygame.K_UP]) - float(keys[pygame.K_DOWN])
    throttle = 1.0 if drive != 0.0 else 0.0
    psi_d = psi_ref if drive >= 0.0 else psi_ref + np.pi

    tau_cmd = 0.0
    if keys[pygame.K_SPACE]:
        tau_cmd = float(ss.tau_max)
    elif keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
        tau_cmd = -float(ss.tau_max)

    return np.array([psi_d, throttle, tau_cmd]), psi_ref


def main():
    pygame.init()
    pygame.key.set_repeat(250, 30)
    view = View()
    clock = pygame.time.Clock()

    bot = Bot(DT)
    psi_ref = 0.0
    paused = False
    pending = 0
    accumulator = 0.0
    running = True

    while running:
        frame_dt = clock.tick(RENDER_HZ)/1000.0

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
                    accumulator = 0.0
                    pending = 0
                    view.clear_trail()

        keys = pygame.key.get_pressed()
        cmd, psi_ref = read_command(keys, psi_ref, frame_dt)

        if paused:
            substeps, pending = pending, 0
        else:
            accumulator += frame_dt
            substeps = min(int(accumulator/DT), MAX_SUBSTEPS)
            accumulator -= substeps*DT

        if substeps > 0:
            traj = bot.run(control.closed_loop, cmd, substeps)
            view.push_trail(traj)
            view.update_leds(traj[:, 2], psi_ref)

        wheels = control.wheel_forces(bot.theta, cmd)
        view.draw(bot, cmd, psi_ref, wheels, paused, substeps)

    pygame.quit()


if __name__ == "__main__":
    main()
