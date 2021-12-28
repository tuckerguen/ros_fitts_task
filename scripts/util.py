import pygame
import numpy as np
import matplotlib.pyplot as plt


def all_pts_close(pts, tol=0.0000001):
    if not pts:
        return True
    pts = np.array(pts)
    in_tol = [np.all(np.abs(pts[:, i] - np.mean(pts[:, i])) < tol) for i in range(2)]
    return np.all(in_tol)


def is_pygame_running():
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    return running


def pygame_get_screenres():
    pygame.display.set_mode((0, 0), pygame.WINDOWMINIMIZED)
    pygame.init()
    w, h = pygame.display.get_surface().get_size()
    pygame.quit()
    return w, h


def plt_config_for_screen():
    w, h = pygame_get_screenres()
    plt.ylim((0, h))
    plt.gca().invert_yaxis()
    plt.xlim((0, w))


def render_lines_of_text(screen, text_lines, font='Arial', text_height=30, line_gap=5, color=(255,255,255)):
    font = pygame.font.SysFont(font, text_height)
    for i, text_line in enumerate(text_lines):
        textsurface = font.render(text_line, False, color)
        screen.blit(textsurface, (0, (text_height+line_gap) * i))


class Timepoint:
    """
    A single timepoint. Given by timestamp, x pixel, y pixel, 3d point
    """
    def __init__(self, t, x, y, pt_3d):
        self.t = t
        self.x = x
        self.y = y
        self.pt_3d = pt_3d

    def print(self):
        print(f"{self.t:.2f}: ({self.x}, {self.y}), "
              f"({self.pt_3d[0]:0.3f}, {self.pt_3d[1]:0.3f}, {self.pt_3d[2]:0.3f})")


class Trajectory:
    """
    A single fitts task trajectory. Defined by a target position, target size,
    and list of timepoints specifying the trajectory
    """
    def __init__(self, cpos, crad):
        self.cpos = cpos
        self.crad = crad
        self.timepoints = []

    def __getitem__(self, item):
        return self.timepoints[item]

    def add_timepoint(self, tp):
        self.timepoints.append(tp)

    def print(self):
        print(f"Trajectory: {self.cpos}, {self.crad}, {len(self.timepoints)}")

    def to_numpy(self):
        coords = np.array([(tp.x, tp.y) for tp in self.timepoints])
        pts_3d = np.array([tp.pt_3d for tp in self.timepoints])
        times = np.array([tp.t for tp in self.timepoints])
        return coords[:, 0], coords[:, 1], times, pts_3d

    def plot2d(self):
        x, y, _, _ = self.to_numpy()
        plt.scatter(x, y)
        plt_config_for_screen()
        plt.show()

    def replay(self):
        pygame.init()
        screen = pygame.display.set_mode()
        screen.fill((0, 0, 0))
        clock = pygame.time.Clock()
        dt = self.timepoints[1].t - self.timepoints[0].t
        self.pygame_draw(screen, clock, dt)
        pygame.quit()

    def pygame_draw(self, screen, clock, dt, speed=1):
        for tp in self.timepoints:
            pygame.draw.circle(screen, (0, 0, 255), self.cpos, self.crad, 0)
            pygame.draw.circle(screen, (255, 255, 255), (tp.x, tp.y), 5, 0)
            pygame.display.flip()
            pygame.display.update()
            clock.tick(int(1 / dt) * speed)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
        return False


class Trial:
    """
    A collection of old_trajectories
    """
    def __init__(self):
        self.trajectories = []

    def __len__(self):
        return len(self.trajectories)

    def __getitem__(self, item):
        return self.trajectories[item]

    def add_trajectory(self, t):
        self.trajectories.append(t)

    def plot(self):
        for traj in self.trajectories:
            x, y, _, _ = traj.to_numpy()
            plt.plot(x, y)
        plt_config_for_screen()
        plt.show()

    def plot_ith(self, i):
        self.trajectories[i].plot2d()

    def replay_ith(self, i):
        self.trajectories[i].replay()

    def replay(self, speed=1):
        pygame.init()
        screen = pygame.display.set_mode((0, 0), pygame.WINDOWMINIMIZED)
        screen.fill((0, 0, 0))
        clock = pygame.time.Clock()
        dt = self.trajectories[0].timepoints[1].t - self.trajectories[0].timepoints[0].t
        for traj in self.trajectories:
            screen.fill((0, 0, 0))
            done = traj.pygame_draw(screen, clock, dt, speed=speed)
            if done:
                pygame.quit()
                return
        pygame.quit()

