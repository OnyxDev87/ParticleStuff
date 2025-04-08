import pygame
import math

pygame.init()

w, h = 1080, 720
scr = pygame.display.set_mode((w, h))
r, c, s = True, pygame.time.Clock(), 0.1
G, gf = 1000, 50000

class P:
    def __init__(self, scr, p, cl, r, m):
        self.s, self.p, self.cl, self.r, self.m = scr, pygame.math.Vector2(p), cl, r, m
        self.v, self.a = pygame.math.Vector2(0, 0), pygame.math.Vector2(0, 0)
    
    def d(self): pygame.draw.circle(self.s, self.cl, (int(self.p.x), int(self.p.y)), self.r)

    def wc(self):
        if self.p.x > w - self.r: self.p.x, self.v.x = w - self.r, -self.v.x * 0.5
        elif self.p.x < 0 + self.r: self.p.x, self.v.x = 0 + self.r, -self.v.x * 0.5
        if self.p.y > h - self.r: self.p.y, self.v.y = h - self.r, -self.v.y * 0.5
        elif self.p.y < 0 + self.r: self.p.y, self.v.y = 0 + self.r, -self.v.y * 0.5

    def u(self):
        self.v += self.a * s
        self.p += self.v * s
        self.wc()
        self.d()

def ag(pL):
    for i, p1 in enumerate(pL):
        for j, p2 in enumerate(pL):
            if i != j:
                d = p2.p - p1.p
                di = d.length()
                if di > p1.r + p2.r:
                    f = G * (p1.m * p2.m) / (di ** 2)
                    a = (d.normalize() * f) / p1.m
                    p1.a += a

def adg(pL): 
    for p in pL: p.a.y += gf / p.m

def ec(pL):
    for i, p1 in enumerate(pL):
        for j, p2 in enumerate(pL):
            if i < j:
                d = p2.p - p1.p
                di = d.length()
                if di < p1.r + p2.r:
                    d.normalize_ip()
                    v = p1.v - p2.v
                    sp = v.dot(d)
                    if sp > 0:
                        m1, m2 = p1.m, p2.m
                        p1.v -= (2 * m2 * sp / (m1 + m2)) * d
                        p2.v += (2 * m1 * sp / (m1 + m2)) * d

def sp(p1, p2, dist):
    d = p1.p - p2.p
    mp = (p1.p + p2.p) / 2
    d = d.normalize() * (dist / 2)
    p1.p, p2.p = mp + d, mp - d
    pygame.draw.line(scr, (255, 255, 255), p1.p, p2.p, 3)

p1, p2, p3, p4 = P(scr, [25, 100], (0, 0, 255), 10, 1000), P(scr, [1000, 70], (255, 0, 0), 10, 1000), P(scr, [25, 700], (0, 255, 0), 10, 1000), P(scr, [250, 700], (255, 255, 0), 10, 1000)
p1.v, p2.v = pygame.math.Vector2(90, 100), pygame.math.Vector2(271, 100)
pl = [p1, p2, p3, p4]

while r:
    for p in pl: p.a = pygame.math.Vector2(0, 0)
    scr.fill((30, 30, 30))
    adg(pl)
    sp(p1, p2, 100)
    sp(p2, p3, 100)
    sp(p3, p4, 100)
    sp(p4, p1, 100)
    sp(p2, p4, 150)
    sp(p1, p3, 150)
    ec(pl)
    for p in pl: p.u()

    for e in pygame.event.get():
        if e.type == pygame.QUIT: r = False

    pygame.display.flip()
    s = c.tick(60) / 1000
    s = max(0.001, min(0.1, s))

pygame.quit()

