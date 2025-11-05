import pygame
import random

# Initialize Pygame
pygame.init()

# Define constants for the screen size
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Two Bouncing Squares")

# --- Square Properties ---
SQUARE_SIZE = 50

# Define colors
WHITE = (255, 255, 255)

# Create two squares with different positions and speeds
square1 = pygame.Rect(
    SCREEN_WIDTH // 2 - SQUARE_SIZE // 2,
    SCREEN_HEIGHT // 2 - SQUARE_SIZE // 2,
    SQUARE_SIZE,
    SQUARE_SIZE,
)
square2 = pygame.Rect(
    random.randint(0, SCREEN_WIDTH - SQUARE_SIZE),
    random.randint(0, SCREEN_HEIGHT - SQUARE_SIZE),
    SQUARE_SIZE,
    SQUARE_SIZE,
)

# Initial speeds for both squares
speed1_x, speed1_y = 1, 1
speed2_x, speed2_y = 1, 1

# Game loop control
running = True

# --- Game Loop ---
while running:
    # Event Handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Game Logic (Updates)
    # Move squares
    square1.x += speed1_x
    square1.y += speed1_y

    square2.x += speed2_x
    square2.y += speed2_y

    # Bounce square 1
    if square1.right >= SCREEN_WIDTH or square1.left <= 0:
        speed1_x *= -1
    if square1.bottom >= SCREEN_HEIGHT or square1.top <= 0:
        speed1_y *= -1

    # Bounce square 2
    if square2.right >= SCREEN_WIDTH or square2.left <= 0:
        speed2_x *= -1
    if square2.bottom >= SCREEN_HEIGHT or square2.top <= 0:
        speed2_y *= -1

    # Bounce off each other
    if square1.colliderect(square2):
        speed1_x *= -1
        speed1_y *= -1
        speed2_x *= -1
        speed2_y *= -1

    # Drawing
    screen.fill((0, 0, 0))  # Clear screen with black
    pygame.draw.rect(screen, WHITE, square1)
    pygame.draw.rect(screen, WHITE, square2)

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
