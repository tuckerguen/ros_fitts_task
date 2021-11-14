import pygame


def render_lines_of_text(screen, text_lines, font='Arial', text_height=30, line_gap=5, color=(255,255,255)):
    font = pygame.font.SysFont(font, text_height)
    for i, text_line in enumerate(text_lines):
        textsurface = font.render(text_line, False, color)
        screen.blit(textsurface, (0, (text_height+line_gap) * i))
