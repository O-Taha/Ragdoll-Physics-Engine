from typing import List, Tuple
import pygame
from pygame.math import Vector2, Vector3
import matplotlib.pyplot as plt

class Point:
	def __init__(self, pos: Vector3, vel: Vector3, acc: Vector3, w: float = 1.0):
		self.pos: Vector3 = pos
		self.vel: Vector3 = vel
		self.acc: Vector3 = acc
		self.w: float = w

class Edge:
	def __init__(self, p1: Point, p2: Point, s: float):
		self.points: Tuple[Point, Point] = (p1, p2)
		self.s: float = s

class Body:
	def __init__(self, pSet: List[Point], eSet: List[Edge], wireframe: bool, freeze: bool):
		self.points: List[Point] = pSet
		self.edges: List[Edge] = eSet
		self.wireframe: bool = wireframe
		self.freeze: bool = freeze

class World:
	def __init__(self, forces: List[Vector3], bodies: List[Body], h: float = 1.0):
		self.forces = forces
		self.bodies = bodies
		self.h = h

	def run(self, T: float = 0):
		t: float = 0
		times = []
		positions_y = []
		while((t < T) if T else 1):
			for body in self.bodies:
				for p in body.points:
					self.verlet(p)
					print(f"t={t:.2f} pos={p.pos} vel={p.vel}")
					times.append(t)
					positions_y.append(p.pos.y)
				for p in body.points:
					self.collision(p)
				for e in body.edges:
					self.constraint(e)
			t += self.h
		plt.figure()
		plt.plot(times, positions_y)
		plt.xlabel("Time (s)")
		plt.ylabel("Height (y)")
		plt.title("Point falling under gravity")
		plt.grid(True)
		plt.show()

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
		return sum(self.forces, Vector3(0, 0, 0)) * p.w

if __name__ == "__main__":
	# Gravité vers le bas (axe Y négatif)
	g = Vector3(0, -9.81, 0)

	# Point initial à hauteur 10
	p = Point(
		pos=Vector3(0, 10, 0),
		vel=Vector3(0, 5, 0),
		acc=Vector3(0, 0, 0),
		w=1.0
	)

	body = Body([p], [], wireframe=False, freeze=False)

	world = World(
		forces=[g],
		bodies=[body],
		h=0.1
	)

	world.run(T=2.0)
	
	# affichage pygame
	# corriger les forces (pour chaque point)