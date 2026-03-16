from typing import List, Tuple, Callable
import pygame
from pygame.math import Vector2, Vector3

running: bool = False

class Point:
	def __init__(self, pos: Vector3, vel: Vector3, w: float = 1.0):
		self._pos:			Vector3	= pos		# DO NOT REMOVE '_' OR ELSE : RecursionError: maximum recursion depth exceeded
		self._old_pos:		Vector3 = pos
		self.vel:			Vector3	= vel
		self.acc:			Vector3	= 0
		self.w:				float	= w			# inverse of weight
		self.old_w:			float
		self.recordPos:		Array[Vector3]	= [pos]
	
	@property
	def pos(self):
		return self._pos
	@pos.setter
	def pos(self, newPos: Vector3):
		self._pos = newPos
		#self.recordPos.append(newPos)
		

class Edge:
	def __init__(self, p1: Point, p2: Point, l: float ,s: float):
		self.points: Tuple[Point, Point] = (p1, p2)
		self.l: float = l
		self.s: float = s

class Body:
	def __init__(self, points: List[Point], edges: List[Edge], forces: List[Callable], wireframe: bool, freeze: bool,):
		self.points: List[Point] = points
		self.edges: List[Edge] = edges
		self.forces: List[Callable] = forces
		self.wireframe: bool = wireframe
		self.freeze: bool = freeze


def gravity(world, body, point) -> Vector3:
	"""
	world.g : Vector3
	point.w : masse
	"""
	if point.w == 0:
		return Vector3(0, 0, 0)
	return Vector3(0, (-9.81 * 100) / point.w, 0)

def wind(world, body, point) -> Vector3:
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

def solid_friction(world, body, point) -> Vector3:
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

def viscous_drag(world, body, point) -> Vector3:
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
		self.h = h
		for body in self.bodies:
			if not body.wireframe: return

			if not body.freeze:
				for p in body.points:
					# sauvegarde de la position pour recalculer la vitesse apres les contraintes et collisions
					p._old_pos = Vector3(p.pos)
					self.verlet(body, p)
			for _ in range(10):
				for p in body.points:
					for volume in self.bodies:
						if volume is not body and not volume.wireframe:
							self.collision(p, volume)
				for e in body.edges:
					self.constraint(e)
		
			# recalcul de la vitesse
			for p in body.points:
				p.vel = ((p.pos - p._old_pos) / h)

		self.t += h
		self.recordTime.append(self.t)
			
	def verlet(self, body,  p:Point):
		acc_n = self.computeAccel(body, p)
		p.acc = acc_n
		p.pos = p.pos + self.h*p.vel + ((self.h**2)/2)*p.acc
		p.acc = self.computeAccel(body, p)
		p.vel = p.vel + (self.h/2)*(acc_n + p.acc)

	def collision(self, p:Point, volume:Body):

		# On suppose que l'obstacle a au moins 3 points : 
		# A (bas-gauche), B (bas-droite), D (haut-gauche)
		A = volume.points[0].pos
		B = volume.points[1].pos
		D = volume.points[3].pos

		# Vecteurs directeurs
		u = B - A
		v = D - A
		ap = p.pos - A

		# Carrés des longueurs (pour éviter sqrt)
		u_len_sq = u.length_squared()
		v_len_sq = v.length_squared()

		if u_len_sq == 0 or v_len_sq == 0: return

		# Projections normalisées (0.0 à 1.0)
		mu = ap.dot(u) / u_len_sq
		mv = ap.dot(v) / v_len_sq

		# Test d'inclusion
		if 0 <= mu <= 1 and 0 <= mv <= 1:
			# On stocke la position AVANT la répulsion
			old_pos_before_correction = Vector3(p.pos)
			# On calcule les distances aux bords (en unités normalisées)
			# Mais pour repousser, on repasse en pixels
			dist_left   = mu * u.length()
			dist_right  = (1 - mu) * u.length()
			dist_bottom = mv * v.length()
			dist_top    = (1 - mv) * v.length()

			min_d = min(dist_left, dist_right, dist_bottom, dist_top)

			# Repousse le point selon l'axe local
			if min_d == dist_left:
				p.pos -= u.normalize() * dist_left
			elif min_d == dist_right:
				p.pos += u.normalize() * dist_right
			elif min_d == dist_bottom:
				p.pos -= v.normalize() * dist_bottom
			else:
				p.pos += v.normalize() * dist_top
				
			# Friction
			velocity_vector = p.pos - p._old_pos
			p._old_pos += velocity_vector * (1 - 0.1)

	def constraint(self, edge: Edge):
		
		if (edge.points[0].w + edge.points[1].w) == 0: 
			return

		d = edge.points[1].pos - edge.points[0].pos

		if d.length() < 0.0001: 
			return

		err = (d.length() - edge.l)/d.length()
		edge.points[0].pos = edge.points[0].pos + edge.s * (edge.points[0].w / (edge.points[0].w + edge.points[1].w)) * err * d
		edge.points[1].pos = edge.points[1].pos - edge.s * (edge.points[1].w / (edge.points[0].w + edge.points[1].w)) * err * d
		

	def computeAccel(self, body: Body, point: Point) -> Vector3:
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

	"""
	#pendule
	p1 = Point(
		pos=Vector3(300, 390, 0),
		vel=Vector3(0, 0, 0),
		w=0.0
	)
	p2 = Point(
		pos=Vector3(250, 390, 0),
		vel=Vector3(0, 0, 0),
		w=0.8
	)

	p3 = Point(
		pos=Vector3(200, 390, 0),
		vel=Vector3(0, 0, 0),
		w=0.8
	)

	p4 = Point(
		pos=Vector3(150, 390, 0),
		vel=Vector3(0, 0, 0),
		w=0.4
	)

	p5 = Point(
		pos=Vector3(175, 440, 0),
		vel=Vector3(0, 0, 0),
		w=0.4
	)

	p6 = Point(
		pos=Vector3(175, 340, 0),
		vel=Vector3(0, 0, 0),
		w=0.4
	)

	e1 = Edge(
		p1=p1, 
		p2=p2, 
		l=100.0, 
		s=0.2
	)

	e2 = Edge(
		p1=p2, 
		p2=p3, 
		l=100.0, 
		s=0.2
	)

	e3 = Edge(
		p1=p3, 
		p2=p5, 
		l=50.0, 
		s=1
	)

	e4 = Edge(
		p1=p5, 
		p2=p4, 
		l=50.0, 
		s=1
	)

	e5 = Edge(
		p1=p4, 
		p2=p6, 
		l=50.0, 
		s=1
	)

	e6 = Edge(
		p1=p6, 
		p2=p3, 
		l=50.0, 
		s=1
	)

	e7 = Edge(
		p1=p5, 
		p2=p6, 
		l=70.0, 
		s=1
	)"""

	"""
	#ressort
	p1 = Point(
        pos=Vector3(300, 300, 0), 
        vel=Vector3(0, 0, 0), 
        w=0.0
    )

	p2 = Point(
        pos=Vector3(300, 100, 0), 
        vel=Vector3(0, 0, 0), 
        w=0.01 # Masse de 100 (1 / 0.01)
    )

	e1 = Edge(p1, p2, l=100.0, s=0.1)"""

	# Positions de départ (x, y)
	x, y = 300, 380

	# --- Tête et Tronc ---
	top_head = Point(Vector3(x, y, 0), Vector3(0,0,0), w=1.0)      # FIXE
	base_head = Point(Vector3(x, y-30, 0), Vector3(0,0,0), w=1.0)
	base_neck = Point(Vector3(x, y-45, 0), Vector3(0,0,0), w=1.0)
	mid_torso = Point(Vector3(x, y-60, 0), Vector3(0,0,0), w=1.0)
	mid_pelvis = Point(Vector3(x, y-90, 0), Vector3(0,0,0), w=1.0)

	# --- Bras ---
	l_shoulder = Point(Vector3(x-10, y-50, 0), Vector3(0,0,0), w=1.0)
	r_shoulder = Point(Vector3(x+10, y-50, 0), Vector3(0,0,0), w=1.0)
	l_elbow    = Point(Vector3(x-55, y-50, 0), Vector3(0,0,0), w=1.0)
	r_elbow    = Point(Vector3(x+55, y-50, 0), Vector3(0,0,0), w=1.0)
	l_hand     = Point(Vector3(x-100, y-50, 0), Vector3(0,0,0), w=1.0)
	r_hand     = Point(Vector3(x+100, y-50, 0), Vector3(0,0,0), w=1.0)

	# --- Jambes ---
	l_hip    = Point(Vector3(x-10, y-105, 0), Vector3(0,0,0), w=1.0)
	r_hip    = Point(Vector3(x+10, y-105, 0), Vector3(0,0,0), w=1.0)
	l_knee   = Point(Vector3(x-10, y-150, 0), Vector3(0,0,0), w=1.0)
	r_knee   = Point(Vector3(x+10, y-150, 0), Vector3(0,0,0), w=1.0)
	l_ankle  = Point(Vector3(x-10, y-200, 0), Vector3(0,0,0), w=1.0)
	r_ankle  = Point(Vector3(x+10, y-200, 0), Vector3(0,0,0), w=1.0)

	all_points = [
	top_head, base_head, base_neck, mid_torso, mid_pelvis,
	l_shoulder, r_shoulder, l_elbow, r_elbow, l_hand, r_hand,
	l_hip, r_hip, l_knee, r_knee, l_ankle, r_ankle
	]

	def create_bone(p1, p2, s=1.0):
		distance = (p1.pos - p2.pos).length()
		return Edge(p1, p2, l=distance, s=s)

	bones = [
	create_bone(top_head, base_head),
	create_bone(base_head, base_neck),
	# Épaules et Torse
	create_bone(base_neck, l_shoulder),
	create_bone(base_neck, r_shoulder),
	create_bone(l_shoulder, mid_torso),
	create_bone(r_shoulder, mid_torso),
	
	# Bras
	create_bone(l_shoulder, l_elbow),
	create_bone(l_elbow, l_hand),
	create_bone(r_shoulder, r_elbow),
	create_bone(r_elbow, r_hand),
	
	# Colonne et Bassin
	create_bone(mid_torso, mid_pelvis),
	create_bone(mid_pelvis, l_hip),
	create_bone(mid_pelvis, r_hip),
	
	# Jambes
	create_bone(l_hip, l_knee),
	create_bone(l_knee, l_ankle),
	create_bone(r_hip, r_knee),
	create_bone(r_knee, r_ankle),

	# Contraintes
	# Torso
	create_bone(l_shoulder, r_shoulder),
	create_bone(base_neck, mid_torso),

	# Pelvis
	create_bone(l_hip, r_hip),

	# Body
	create_bone(l_shoulder, mid_pelvis, 0.5),
	create_bone(r_shoulder, mid_pelvis, 0.5),
	create_bone(l_hip, mid_torso, 0.5),
	create_bone(r_hip, mid_torso, 0.5)
	]


	body = Body(all_points, bones, [], wireframe=True, freeze=True)
	
	p1 = Point(Vector3(0, 0, 0), Vector3(0,0,0), w=1.0)
	p2 = Point(Vector3(600, 0, 0), Vector3(0,0,0), w=1.0)
	p3 = Point(Vector3(600, 50, 0), Vector3(0,0,0), w=1.0)
	p4 = Point(Vector3(0, 50, 0), Vector3(0,0,0), w=1.0)

	e1 = Edge(p1, p2, 50, 1)
	e2 = Edge(p2, p3, 50, 1)
	e3 = Edge(p3, p4, 50, 1)
	e4 = Edge(p4, p1, 50, 1)

	volume = Body([p1, p2, p3, p4], [e1, e2, e3, e4], [], wireframe=False, freeze=True)

	world = World(forces=[gravity], bodies=[body, volume], T = 0.0, h=0.016)

	running = True
	last_time = pygame.time.get_ticks() / 1000 # in seconds
	
	selected_point = None
	is_dragging = False

	while running:

		# CORRECTION DU DELTA TIME : on récupère les secondes
		dt = clock.tick(60) / 1000.0 
		if dt > 0.05: dt = 0.016 # Sécurité si la fenêtre est déplacée

		##now = pygame.time.get_ticks() / 1000
		##h = now - last_time
		##last_time = now

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

			
			if event.type == pygame.MOUSEBUTTONDOWN:
				mx, my = event.pos # Position actuelle
				mouse_v = Vector3(mx, screen_height - my, 0)
				
				for p in all_points:
					if (p.pos - mouse_v).length() < 15: # Rayon de 15 pixels
						selected_point = p
						is_dragging = True
						# On mémorise si le point était fixe ou non pour le restaurer après
						p.old_w = p.w 
						p.w = 0.0 # On le rend "fixe" temporairement pour qu'il suive la souris parfaitement
						break
			
			if event.type == pygame.MOUSEBUTTONUP:
				if is_dragging and selected_point:
					selected_point.w = selected_point.old_w # On lui rend sa masse initiale
					is_dragging = False
					selected_point = None
			
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE:
					for body in world.bodies:
						body.freeze = not(body.freeze)
				
		if is_dragging and selected_point:
			mx, my = pygame.mouse.get_pos()
			world_y = screen_height - my

			# On déplace le point
			selected_point.pos = Vector3(mx, world_y, 0)
			# Important : on aligne old_pos pour ne pas accumuler d'énergie cinétique folle
			selected_point._old_pos = Vector3(mx, world_y, 0)
		

		##dt_ms = clock.tick(60) 
		##dt_sec = dt_ms / 1000.0  # Conversion en secondes (ex: 0.0166)

		world.run_step(dt)

		#world.run_step(pygame.time.Clock.get_time(clock))

		screen.fill((0,0,0))
		for body in world.bodies:
			for p in body.points:
				pygame.draw.circle(screen, (255,0,0), (int(p.pos.x), int(screen_height - p.pos.y)), 5)
				font = pygame.font.SysFont(None, 20)
				text = font.render(f"t={world.t:.2f} y={p.pos.y:.2f}", True, (255,255,255))
				screen.blit(text, (10, 10))
			for e in body.edges:
				start_point = (int(e.points[0].pos.x), int(screen_height - int(e.points[0].pos.y)))
				end_point   = (int(e.points[1].pos.x), int(screen_height - int(e.points[1].pos.y)))
				pygame.draw.line(screen, (0, 255, 0), start_point, end_point, 2)

		pygame.display.flip()
		#clock.tick(60)

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
