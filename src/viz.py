import numpy as np
import pygame

import src.geomutils as geom
import src.state_space as ss

BACKGROUND = (18, 20, 24)
GRID = (34, 38, 46)
GRID_AXIS = (58, 64, 78)
CHASSIS = (82, 148, 226)
CHASSIS_EDGE = (150, 200, 255)
BLUR = (60, 100, 160)
WHEEL = (235, 235, 240)
LED_FRONT = (60, 240, 110)
LED_REAR = (240, 70, 70)
LED_CORE = (255, 255, 255)
ARROW = (120, 230, 150)
TRAIL = (70, 110, 150)
TEXT = (226, 230, 236)
TEXT_DIM = (130, 140, 155)
WARN = (255, 120, 120)

PIXELS_PER_M = 250.0
GRID_SPACING_M = 0.5
BLUR_THRESHOLD_REV_S = 5.0
TRAIL_MAX = 600
LED_ARC = np.radians(14.0)   # half-width of the arc over which the LEDs fire


def _flash_angle(thetas, psi_ref):
    """Body angle at which the LEDs fired this frame, or None if they did not.

    Scans every substep rather than the frame endpoint: at operating RPM the
    chassis turns about a full revolution per frame, so sampling once would
    catch the flash at random.
    """
    hits = np.flatnonzero(np.abs(geom.ssa(thetas - psi_ref)) < LED_ARC)
    return float(thetas[hits[-1]]) if hits.size else None


class View:
    """Pygame rendering of the arena and the bot."""

    def __init__(self, width=1180, height=760):
        self.surface = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Melty-RL")
        self.font = pygame.font.SysFont("consolas", 16)
        self.font_small = pygame.font.SysFont("consolas", 14)
        self.follow = True
        self.trail = []
        self.flash_angle = None

    @property
    def center(self):
        w, h = self.surface.get_size()
        return np.array([w*0.5, h*0.5])

    def _camera(self, bot):
        return bot.position if self.follow else np.zeros(2)

    def to_screen(self, world_xy, camera):
        """Arena metres (+Y up) to screen pixels (+Y down)."""
        rel = (np.asarray(world_xy, dtype=np.float64) - camera)*PIXELS_PER_M
        return self.center + np.array([rel[0], -rel[1]])

    def push_trail(self, traj):
        # Subsample: a frame holds ~167 substeps but only needs a few points.
        self.trail.extend(traj[::16, 0:2])
        if len(self.trail) > TRAIL_MAX:
            del self.trail[:len(self.trail) - TRAIL_MAX]

    def clear_trail(self):
        self.trail.clear()

    def update_leds(self, thetas, psi_ref):
        """Records where on the rim the LEDs fired this frame, if at all."""
        self.flash_angle = _flash_angle(thetas, psi_ref)

    def draw(self, bot, cmd, psi_ref, wheels, paused, substeps):
        camera = self._camera(bot)
        self.surface.fill(BACKGROUND)
        self._draw_grid(camera)
        self._draw_trail(camera)
        self._draw_bot(bot, cmd, psi_ref, camera)
        self._draw_hud(bot, cmd, psi_ref, wheels, paused, substeps)
        pygame.display.flip()

    def _draw_grid(self, camera):
        w, h = self.surface.get_size()
        step = GRID_SPACING_M*PIXELS_PER_M
        origin = self.to_screen((0.0, 0.0), camera)

        first_x = origin[0] % step
        for i in range(int(w/step) + 2):
            x = first_x + i*step
            colour = GRID_AXIS if abs(x - origin[0]) < 0.5 else GRID
            pygame.draw.line(self.surface, colour, (x, 0), (x, h))

        first_y = origin[1] % step
        for i in range(int(h/step) + 2):
            y = first_y + i*step
            colour = GRID_AXIS if abs(y - origin[1]) < 0.5 else GRID
            pygame.draw.line(self.surface, colour, (0, y), (w, y))

    def _draw_trail(self, camera):
        if len(self.trail) < 2:
            return
        points = [self.to_screen(p, camera) for p in self.trail]
        pygame.draw.aalines(self.surface, TRAIL, False, points)

    def _draw_bot(self, bot, cmd, psi_ref, camera):
        pos = self.to_screen(bot.position, camera)
        radius = float(ss.R_body*PIXELS_PER_M)
        rev_s = abs(bot.spin_rate)

        pygame.draw.circle(self.surface, CHASSIS, pos, radius)
        pygame.draw.circle(self.surface, CHASSIS_EDGE, pos, radius, 2)

        if rev_s > BLUR_THRESHOLD_REV_S:
            pygame.draw.circle(self.surface, BLUR, pos,
                               float(ss.r_w*PIXELS_PER_M), 7)
        else:
            self._draw_wheels(bot, pos)

        # Rim tick at the true heading. Above ~17 rev/s this aliases against the
        # 60 Hz frame rate and jitters, which is the honest depiction.
        tick = pos + radius*np.array([np.cos(bot.theta), -np.sin(bot.theta)])
        pygame.draw.line(self.surface, WHEEL, pos, tick, 2)

        # LEDs are bolted to the chassis at body angles 0 and pi, so they ride
        # round with the stripe and only light as they sweep past psi_ref.
        lit = self.flash_angle is not None
        led_angle = self.flash_angle if lit else bot.theta
        self._draw_led(pos, radius, led_angle, LED_FRONT, lit)
        self._draw_led(pos, radius, led_angle + np.pi, LED_REAR, lit)

        if cmd[1] > 0.0:
            psi_d = cmd[0]
            direction = np.array([np.cos(psi_d), -np.sin(psi_d)])
            self._draw_arrow(pos, direction, radius + 70.0)

    def _draw_led(self, pos, radius, angle, colour, lit):
        unit = np.array([np.cos(angle), -np.sin(angle)])
        centre = pos + (radius - 7.0)*unit
        if lit:
            pygame.draw.circle(self.surface, colour, centre, 10)
            pygame.draw.circle(self.surface, LED_CORE, centre, 4)
        else:
            pygame.draw.circle(self.surface, [c//4 for c in colour], centre, 10, 2)

    def _draw_wheels(self, bot, pos):
        for side in (-1.0, 1.0):
            offset = side*ss.r_w*PIXELS_PER_M
            # Wheels sit on the body y-axis, which is theta + 90 degrees.
            normal = np.array([np.cos(bot.theta + np.pi/2),
                               -np.sin(bot.theta + np.pi/2)])
            axis = np.array([np.cos(bot.theta), -np.sin(bot.theta)])
            hub = pos + offset*normal
            pygame.draw.line(self.surface, WHEEL,
                             hub - 11.0*axis, hub + 11.0*axis, 6)

    def _draw_arrow(self, origin, direction, length):
        tip = origin + length*direction
        pygame.draw.line(self.surface, ARROW, origin, tip, 3)
        perp = np.array([-direction[1], direction[0]])
        pygame.draw.polygon(self.surface, ARROW, [
            tip + 12.0*direction,
            tip + 7.0*perp,
            tip - 7.0*perp,
        ])

    def _draw_hud(self, bot, cmd, psi_ref, wheels, paused, substeps):
        rpm = bot.spin_rate*60.0
        leds = "FIRE" if self.flash_angle is not None else "----"
        lines = [
            f"{rpm:8.0f} RPM   {bot.spin_rate:6.2f} rev/s",
            f"{bot.speed:8.2f} m/s   heading {np.degrees(bot.heading):7.1f} deg",
            f"front    {np.degrees(geom.ssa(psi_ref)):7.1f} deg   led {leds}",
            f"psi_d    {np.degrees(cmd[0]):7.1f} deg   throttle {cmd[1]:.2f}",
            f"tau_cmd  {cmd[2]:7.3f} N m",
            f"wheels   {wheels[0]:7.2f} {wheels[1]:7.2f} N",
            f"sat s    {wheels[2]:7.3f}",
            f"substeps {substeps:7d}",
        ]
        if paused:
            lines += [
                "",
                f"X   {bot.position[0]: .6f}",
                f"Y   {bot.position[1]: .6f}",
                f"th  {bot.theta: .6f}",
                f"vx  {bot.velocity[0]: .6f}",
                f"vy  {bot.velocity[1]: .6f}",
                f"w   {bot.omega: .6f}",
            ]

        for i, line in enumerate(lines):
            colour = WARN if (line.startswith("sat") and wheels[2] < 0.999) else TEXT
            self.surface.blit(self.font.render(line, True, colour), (16, 14 + 20*i))

        footer = ("PAUSED  N substep  M frame  P resume"
                  if paused else
                  "up/down drive   left/right steer front   space spin   shift brake   P pause   C camera   R reset")
        mode = f"camera: {'follow' if self.follow else 'fixed'}"
        h = self.surface.get_height()
        self.surface.blit(self.font_small.render(footer, True, TEXT_DIM), (16, h - 44))
        self.surface.blit(self.font_small.render(mode, True, TEXT_DIM), (16, h - 24))
