import pygame
import math
import numpy as np
import random

pygame.init()

width = 1080
height = 720

screen = pygame.display.set_mode((width, height))

running = True
clock = pygame.time.Clock()

delta_time = 0.1
G = 1000
gravity_force = 1000
dampening = 0.98

circle_center = pygame.math.Vector2(540, 360)
num_particles = 25
particle_dist = 15
particles1 = []
particles2 = []
scale  = 1
circle_radius = math.floor(((num_particles*particle_dist)/2)/math.pi)
expected_area = math.pi * (circle_radius ** 2)

mousePressed = pygame.mouse.get_pressed()[0]
grabbed_particle = None
grab_radius = 50

collision_circle_radius = 30


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
        velocity = (self.pos - self.prev_pos) * dampening
        new_pos = self.pos + velocity + self.acc * (delta_time ** 2)
        self.prev_pos = self.pos
        self.pos = new_pos
        self.wallCollisions(width, height)
        # self.draw()

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

def applySprings(particles, rest_length, iterations=5):
    for _ in range(iterations):
        for i in range(len(particles)):
            p1 = particles[i]
            p2 = particles[(i + 1) % len(particles)]
            spring(p1, p2, rest_length)

def computePolygonArea(particles):
    area = 0
    n = len(particles)
    for i in range(n):
        j = (i + 1) % n
        area += (particles[i].pos.x * particles[j].pos.y) - (particles[j].pos.x * particles[i].pos.y)
    return abs(area) / 2

def applyInflation(particles, expected_area, inflation_strength=0.0001):
    current_area = computePolygonArea(particles)
    area_diff = expected_area - current_area
    if abs(area_diff) < 1:
        return
    centroid = sum((p.pos for p in particles), pygame.math.Vector2(0, 0)) / len(particles)
    for p in particles:
        direction = (p.pos - centroid).normalize()
        p.pos += direction * area_diff * inflation_strength

def createSoftBody(particles, circle_center, particle_dist, num_particles):
    for i in range(num_particles):
        angle = (2 * math.pi / num_particles) * i
        x = circle_center.x + particle_dist * math.cos(angle)
        y = circle_center.y + particle_dist * math.sin(angle)
        particle = Particle(screen, [x, y], (255, 255, 255), 10, 10)
        particles.append(particle)

def cubicSpline(pts):
    N = len(pts)
    z = np.array([[p.pos.x, p.pos.y] for p in pts])
    n = N

    # Create a periodic tridiagonal system
    A = np.zeros((n, n))
    np.fill_diagonal(A, 4)
    np.fill_diagonal(A[1:], 1)
    np.fill_diagonal(A[:, 1:], 1)
    A[0, -1] = 1
    A[-1, 0] = 1

    # Compute r
    r = np.zeros((n, 2))
    for i in range(n):
        r[i] = 3 * (z[(i + 1) % n] - z[(i - 1) % n])

    # Solve for derivatives
    w = np.linalg.solve(A, r)

    # Draw the spline
    for i in range(n):
        a = z[i]
        b = w[i]
        c = 3 * (z[(i + 1) % n] - z[i]) - 2 * w[i] - w[(i + 1) % n]
        d = 2 * (z[i] - z[(i + 1) % n]) + w[i] + w[(i + 1) % n]

        for t in np.linspace(0, 1, 20):
            point = a + b * t + c * (t ** 2) + d * (t ** 3)
            pygame.draw.circle(screen, (255, 255, 255), point.astype(int), 2)

def collide_with_circle(particles, circle_center, circle_radius):
    for p in particles:
        to_particle = p.pos - circle_center
        distance = to_particle.length()
        if distance < circle_radius + p.radius:
            if distance != 0:
                normal = to_particle.normalize()
            else:
                normal = pygame.math.Vector2(1, 0)

            overlap = (circle_radius + p.radius) - distance
            p.pos += normal * overlap

createSoftBody(particles1, pygame.math.Vector2(300, 360), 15, 25)
createSoftBody(particles2, pygame.math.Vector2(300, 360), 15, 25)

for p in particles1:
    p.acc = pygame.math.Vector2(1000, 1000)

while running:

    screen.fill((30, 30, 30))

    mousePressed = pygame.mouse.get_pressed()[0]
    mouse_pos = pygame.mouse.get_pos()
    mouse_vector = pygame.math.Vector2(mouse_pos)

    mousePressedNow = pygame.mouse.get_pressed()[0]
    mouse_pos = pygame.mouse.get_pos()
    mouse_vector = pygame.math.Vector2(mouse_pos)

    if mousePressedNow and grabbed_particle is None:
        for p in particles1:
            if p.pos.distance_to(mouse_vector) < grab_radius:
                grabbed_particle = p
                break

    if mousePressedNow and grabbed_particle:
        grabbed_particle.pos = mouse_vector
        grabbed_particle.prev_pos = mouse_vector

    if not mousePressedNow:
        grabbed_particle = None

    keys = pygame.key.get_pressed()
    space_pressed = pygame.mouse.get_pressed()[2]

    if space_pressed:
        pygame.draw.circle(screen, (200, 100, 100), (int(mouse_vector.x), int(mouse_vector.y)), collision_circle_radius)
        collide_with_circle(particles1, mouse_vector, collision_circle_radius)

    applyDownGravity(particles1)

    applySprings(particles1, particle_dist, iterations=8)
    for i in range(15):
        applyInflation(particles1, expected_area)

    for i in range(len(particles1)):
        particles1[i].update(width, height)
        particles1[i].acc = pygame.math.Vector2(0, 0)
        # pygame.draw.line(screen, (255, 255, 255), particles1[i].pos, particles1[(i + 1) % len(particles1)].pos, 3)
        
    cubicSpline(particles1)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    delta_time = clock.tick(60) / 1000
    delta_time = max(0.001, min(0.1, delta_time))

pygame.quit()
