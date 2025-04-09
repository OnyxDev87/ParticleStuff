import pygame
import math

pygame.init()

width = 1080
height = 720

screen = pygame.display.set_mode((width, height))

running = True
clock = pygame.time.Clock()

delta_time = 0.1
G = 1000
gravity_force = 500
dampening = 0.99

class Particle:
    def __init__(self, surface, pos, color, radius, mass):
        self.surface = surface
        self.pos = pygame.math.Vector2(pos)
        self.prev_pos = self.pos
        self.color = color
        self.radius = radius
        self.mass = mass
        self.acc = pygame.math.Vector2(0, 0)
    
    def draw(self):
        pygame.draw.circle(self.surface, self.color, (int(self.pos.x), int(self.pos.y)), self.radius)

    def wallCollisions(self, width, height):
        if self.pos.x > width - self.radius:
            # right wall
            self.pos.x = width - self.radius
            self.prev_pos.x = self.pos.x + (self.pos.x - self.prev_pos.x)
        elif self.pos.x < self.radius:
            # left wall
            self.pos.x = self.radius
            self.prev_pos.x = self.pos.x + (self.pos.x - self.prev_pos.x)
        if self.pos.y > height - self.radius:
            # ground
            self.pos.y = height - self.radius
            self.prev_pos.y = self.pos.y + (self.pos.y - self.prev_pos.y)
        elif self.pos.y < self.radius:
            # top
            self.pos.y = self.radius
            self.prev_pos.y = self.pos.y + (self.pos.y - self.prev_pos.y)

    def update(self, width, height):
        new_pos = (2 * self.pos - self.prev_pos + (self.acc* (delta_time**2)))
        self.prev_pos = self.pos
        self.pos = new_pos

        # print((self.pos - self.prev_pos) / delta_time)

        self.wallCollisions(width, height)

        self.draw()


def applyGravity(particleList):
    for i, particle1 in enumerate(particleList):
        for j, particle2 in enumerate(particleList):
            if i != j:
                direction = particle2.pos - particle1.pos
                distance = direction.length()

                if distance > particle1.radius + particle2.radius:
                    force = G * (particle1.mass * particle2.mass) / (distance ** 2)
                    acceleration = (direction.normalize() * force) / particle1.mass
                    particle1.acc += acceleration

def applyDownGravity(particleList):
    for particle in particleList:
        particle.acc.y += gravity_force

def spring(p1, p2, dist):
    direction = p1.pos - p2.pos
    midpoint = (p1.pos + p2.pos) / 2

    distance = direction.length()
    if distance > 0:
        direction = direction.normalize()
        direction *= (dist / 2)
        goalP1 = midpoint + direction
        goalP2 = midpoint - direction

        p1.pos = goalP1
        p2.pos = goalP2

        pygame.draw.line(screen, (255, 255, 255), p1.pos, p2.pos, 3)

circle_center = pygame.math.Vector2(540, 360)  # Center of the screen
circle_radius = 100  # Radius of the circle

# Create 8 particles in a circle
particles = []
num_particles = 8
for i in range(num_particles):
    angle = (2 * math.pi / num_particles) * i  # Angle for each particle
    x = circle_center.x + circle_radius * math.cos(angle)
    y = circle_center.y + circle_radius * math.sin(angle)
    particle = Particle(screen, [x, y], (255, 255, 255), 10, 10)
    particles.append(particle)

# Example: Apply gravity and springs between adjacent particles
for i in range(len(particles)):
    particles[i].acc = pygame.math.Vector2(0, 100)  # Example acceleration

while running:

    screen.fill((30, 30, 30))
    applyDownGravity(particles)
    for particle in particles:
        if i < len(particles) - 1:
            spring(particles[i], particles[i + 1], 100)

    for p in particles:
        p.update(width, height)

    for particle in particles:
        particle.acc = pygame.math.Vector2(0, 0)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    delta_time = clock.tick(60) / 1000
    delta_time = max(0.001, min(0.1, delta_time))

pygame.quit()
