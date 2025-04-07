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
gravity_force = 50000

class Particle:
    def __init__(self, surface, pos, color, radius, mass):
        self.surface = surface
        self.pos = pygame.math.Vector2(pos)
        self.color = color
        self.radius = radius
        self.mass = mass
        self.velocity = pygame.math.Vector2(0, 0)
        self.acc = pygame.math.Vector2(0, 0)
    
    def draw(self):
        pygame.draw.circle(self.surface, self.color, (int(self.pos.x), int(self.pos.y)), self.radius)

    def wallCollisions(self, width, height):
        if self.pos.x > width - self.radius:
            self.pos.x = width - self.radius
            self.velocity.x *= -1 * 0.5
        elif self.pos.x < 0 + self.radius:
            self.pos.x = 0 + self.radius
            self.velocity.x *= -1 * 0.5
        if self.pos.y > height - self.radius:
            self.pos.y = height - self.radius
            self.velocity.y *= -1 * 0.5
        elif self.pos.y < 0 + self.radius:
            self.pos.y = 0 + self.radius
            self.velocity.y *= -1 * 0.5

    def update(self, width, height):
        self.velocity += self.acc * delta_time
        self.pos += self.velocity * delta_time
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
        particle.acc.y += gravity_force / particle.mass

def elasticCollisions(particleList):
    for i, particle1 in enumerate(particleList):
        for j, particle2 in enumerate(particleList):
            if i < j:
                direction = particle2.pos - particle1.pos
                distance = direction.length()

                if distance < particle1.radius + particle2.radius:
                    direction.normalize_ip()

                    velocity_diff = particle1.velocity - particle2.velocity
                    speed = velocity_diff.dot(direction)

                    if speed > 0:
                        m1 = particle1.mass
                        m2 = particle2.mass

                        particle1.velocity -= (2 * m2 * speed / (m1 + m2)) * direction
                        particle2.velocity += (2 * m1 * speed / (m1 + m2)) * direction

def spring(p1, p2, dist):
    direction =  p1.pos - p2.pos
    midpoint = (p1.pos + p2.pos) / 2
    direction = direction.normalize()
    direction *= (dist / 2)
    goalP1 = midpoint + direction
    goalP2 = midpoint - direction

    p1.pos = goalP1
    p2.pos = goalP2

    pygame.draw.line(screen, (255, 255, 255), p1.pos, p2.pos, 3)

p1 = Particle(screen, [25, 100], (0, 0, 255), 10, 1000)
p2 = Particle(screen, [1000, 70], (255, 0, 0), 10, 1000)
p3 = Particle(screen, [25, 700], (0, 255, 0), 10, 1000)
p4 = Particle(screen, [250, 700], (255, 255, 0), 10, 1000)
particles = [p1, p2, p3, p4]
p1.velocity = pygame.math.Vector2(90, 100)
p1.velocity = pygame.math.Vector2(271, 100)

while running:

    for particle in particles:
        particle.acc = pygame.math.Vector2(0, 0)

    screen.fill((30, 30, 30))
    # applyGravity(particles)
    applyDownGravity(particles)
    spring(p1, p2, 100)
    spring(p2, p3, 100)
    spring(p3, p4, 100)
    spring(p4, p1, 100)
    spring(p2, p4, 150)
    spring(p1, p3, 150)
    elasticCollisions(particles)
    for p in particles:
        p.update(width, height)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    delta_time = clock.tick(60) / 1000
    delta_time = max(0.001, min(0.1, delta_time))

pygame.quit()
