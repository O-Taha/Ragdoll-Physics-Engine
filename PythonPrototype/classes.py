from typing import List, Tuple, Callable
import pygame
from pygame.math import Vector2, Vector3

running: bool = False

class Point:
	def __init__(self, pos: Vector3, vel: Vector3, w: float = 1.0):
		self._pos:			Vector3	= pos		# DO NOT REMOVE '_' OR ELSE : RecursionError: maximum recursion depth exceeded
		self.vel:			Vector3	= vel
		self.acc:			Vector3	= 0
		self.w:				float	= w			# inverse of weight
		self.recordPos:		Array[Vector3]	= [pos]
	
	@property
	def pos(self):
		return self._pos
	@pos.setter
	def pos(self, newPos: Vector3):
		self._pos = newPos
		self.recordPos.append(newPos)
		

class Edge:
	def __init__(self, p1: Point, p2: Point, s: float):
		self.points: Tuple[Point, Point] = (p1, p2)
		self.s: float = s

class Body:
	def __init__(self, pSet: List[Point], eSet: List[Edge], wireframe: bool, freeze: bool, forces: List[Callable]):
		self.forces: List[Callable]
		self.points: List[Point] = pSet
		self.edges: List[Edge] = eSet
		self.wireframe: bool = wireframe
		self.freeze: bool = freeze


def gravity(world, body, point):
    """
    world.g : Vector3
    point.w : masse
    """
    return point.w * world.g

def wind(world, body, point):
    """
    Force quadratique opposée à la vitesse relative à l'air
	Hypothèses :
		world.air_density
		world.wind_velocity
		body.area
		body.drag_coeff
    """
    v_rel = point.vel - world.wind_velocity

    speed = v_rel.length()
    if speed == 0:
        return Vector3(0,0,0)

    direction = -v_rel.normalize()

    return 0.5 * world.air_density * body.drag_coeff * body.area * speed**2 * direction

def solid_friction(world, body, point):
    """
    Frottement cinétique simple
	On suppose :
		body.mu_s
		body.mu_k
		body.normal (Vector3)
		collision déjà détectée
    """
    if not hasattr(body, "normal"):
        return Vector3(0,0,0)

    N = body.normal
    normal_force = N * point.w * abs(world.g.dot(N))

    v_tangent = point.vel - point.vel.dot(N) * N
    speed = v_tangent.length()

    if speed == 0:
        return Vector3(0,0,0)

    direction = -v_tangent.normalize()

    return body.mu_k * normal_force.length() * direction

def viscous_drag(world, body, point):
    """
    body.viscous_coeff = alpha
    """
    return -body.viscous_coeff * point.vel

class World:
	def __init__(self, forces: List[Callable], bodies: List[Body], T: float = 0, h: float = 1.0):
		self.global_forces: List[Callable] = forces
		self.bodies: List[Body] = bodies
		self.t: float = 0
		self.T: float = T
		self.h: float = h
		self.recordTime: Array[float] = []

	def run_step(self, h: float = None):
		h = h or self.h
		for body in self.bodies:
			for p in body.points:
				self.verlet(p)
			for p in body.points:
				self.collision(p)
			for e in body.edges:
				self.constraint(e)
		self.t += h
		self.recordTime.append(self.t)
			
	def verlet(self, p:Point):
		acc_n = self.computeAccel(p)
		p.acc = acc_n
		p.pos = p.pos + self.h*p.vel + ((self.h**2)/2)*p.acc
		p.acc = self.computeAccel(p)
		p.vel = p.vel + (self.h/2)*(acc_n + p.acc)

	def collision(self, p:Point):
		pass

	def constraint(self, edge: Edge):
		pass

	def computeAccel(self, p: Point) -> Vector3:
		total_force = Vector3(0,0,0)

		for f in self.global_forces:
			total_force += f(self, body, point)
		for f in body.forces:
			total_force += f(self, body, point)

		return total_force * point.w

if __name__ == "__main__":
	pygame.init()
	screen_width, screen_height = 600, 400
	screen = pygame.display.set_mode((screen_width, screen_height))
	pygame.display.set_caption("Point Simulation")
	clock = pygame.time.Clock()

	# Gravité vers le bas (axe Y négatif)
	g = Vector3(0, -9.81, 0)

	p = Point(
		pos=Vector3(screen_width/2, screen_height/2, 0),
		vel=Vector3(10, 50, 0),
		w=1.0
	)
	body = Body([p], [], wireframe=False, freeze=False)
	world = World(forces=[g], bodies=[body], T = 0.0, h=0.1)

	running = True
	last_time = pygame.time.get_ticks() / 1000 # in seconds
	while running:
		now = pygame.time.get_ticks() / 1000
		h = now - last_time
		last_time = now

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

		world.run_step()

		screen.fill((0,0,0))
		for body in world.bodies:
			for p in body.points:
				pygame.draw.circle(screen, (255,0,0), (int(p.pos.x), int(screen_height - p.pos.y)), 5)
				font = pygame.font.SysFont(None, 20)
				text = font.render(f"t={world.t:.2f} y={p.pos.y:.2f}", True, (255,255,255))
				screen.blit(text, (10, 10))

		pygame.display.flip()
		clock.tick(60)

		if world.T and world.t >= world.T:
			running = False

	pygame.quit()
	print(p.recordPos)
	# ajouter requirements.txt
	# créer un setter/getter pour point.w et corriger les fonctions de forces en adéquation
	# stocker temporairement en dur les coeff des forces
	# tester les forces en callables
	# transformer les forces en classes avec des sous-classes singleton qui ont chacun leurs paramètres spécifiques
	# corriger t et T (T = 10 correspond a 3 sec...)
