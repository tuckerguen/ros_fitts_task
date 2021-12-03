import pygame


def render_lines_of_text(screen, text_lines, font='Arial', text_height=30, line_gap=5, color=(255,255,255)):
    font = pygame.font.SysFont(font, text_height)
    for i, text_line in enumerate(text_lines):
        textsurface = font.render(text_line, False, color)
        screen.blit(textsurface, (0, (text_height+line_gap) * i))


class Timepoint:
    def __init__(self, t, x, y, pt_3d):
        self.t = t
        self.x = x
        self.y = y
        self.pt_3d = pt_3d

    def print(self):
        print(f"{self.t:.2f}: ({self.x}, {self.y}), ({self.pt_3d[0]:0.3f}, {self.pt_3d[1]:0.3f}, {self.pt_3d[2]:0.3f})")


class Trajectory:
    def __init__(self, cpos, crad):
        self.cpos = cpos
        self.crad = crad
        self.data = []

    def add_timepoint(self, tp):
        self.data.append(tp)

    def print(self):
        print(f"Trajectory: {self.cpos}, {self.crad}, {len(self.data)}")

