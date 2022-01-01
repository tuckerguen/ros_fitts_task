import dill
import matplotlib.pyplot as plt
import numpy as np
import pygame


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


def plt_config_lims(xlim, ylim):
    # w, h = pygame_get_screenres()
    plt.ylim((0, ylim))
    plt.gca().invert_yaxis()
    plt.xlim((0, xlim))


def render_lines_of_text(screen, text_lines, font='Arial', text_height=30, line_gap=5, color=(255, 255, 255)):
    font = pygame.font.SysFont(font, text_height)
    for i, text_line in enumerate(text_lines):
        textsurface = font.render(text_line, False, color)
        screen.blit(textsurface, (0, (text_height + line_gap) * i))


class Timepoint:
    """
    A single timepoint. Given by timestamp, x pixel, y pixel, 3d point
    """

    def __init__(self, t, pt):
        self.t = t
        self.pt = pt

    def print(self):
        print(f"{self.t:.2f}: ({self.pt})")


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
        pts = np.array([tp.pt for tp in self.timepoints])
        times = np.array([tp.t for tp in self.timepoints])
        return times, pts

    def plot2d(self):
        _, pts = self.to_numpy()
        plt.scatter(pts[:, 0], pts[:, 1])
        WS_WIDTH = 0.5300869565118
        WS_HEIGHT = 0.298173902
        plt_config_lims(WS_WIDTH, WS_HEIGHT)
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
        fw = 1920 / 0.5300869565118
        fh = 1080 / 0.298173902

        for tp in self.timepoints:
            size_scl = 1080 / 0.298173902
            pygame.draw.circle(screen, (0, 0, 255), (self.cpos[0] * fw, self.cpos[1] * fh), self.crad * size_scl, 0)
            pygame.draw.circle(screen, (255, 255, 255), (tp.pt[0] * fw, tp.pt[1] * fh), 5, 0)
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

    def __init__(self, framerate=60, delay_secs=0.5, n_trials=5):
        self.trajectories = []
        self.framerate = framerate
        self.delay_secs = delay_secs
        self.n_trials = n_trials

    def __len__(self):
        return len(self.trajectories)

    def __getitem__(self, item):
        return self.trajectories[item]

    def run(self, task, pointer_func):
        clock = pygame.time.Clock()
        traj = None

        # Main game loop
        n = 0
        t = 0
        while n < self.n_trials:
            p_pointer = pointer_func()
            is_home, success, target_pos, target_size = task.step(p_pointer)

            # Track the trial points at the correct times
            if not is_home:
                if not traj:
                    traj = Trajectory(target_pos, target_size)
                    t = 0
                tp = Timepoint(t, p_pointer)
                traj.add_timepoint(tp)

            if success:
                n += 1
                self.add_trajectory(traj)
                traj = None

            t += 1 / self.framerate
            clock.tick(self.framerate)

        task.exit()

    def save(self, fp):
        with open(fp, "wb") as f:
            dill.dump(self, f)

    def add_trajectory(self, t):
        self.trajectories.append(t)

    def plot(self):
        for traj in self.trajectories:
            _, pts = traj.to_numpy()
            try:
                plt.plot(pts[:, 0], pts[:, 1])
            except IndexError as e:
                print(e)
        WS_WIDTH = 0.5300869565118
        WS_HEIGHT = 0.298173902
        plt_config_lims(WS_WIDTH, WS_HEIGHT)
        plt.show()

    def plot_ith(self, i):
        self.trajectories[i].plot2d()

    def replay_ith(self, i):
        self.trajectories[i].replay()

    def replay(self, speed=1):
        pygame.init()
        screen = pygame.display.set_mode((1920, 1080))
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
