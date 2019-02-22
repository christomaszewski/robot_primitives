import shapely.geometry
import shapely.ops
import numpy as np
import collections
import operator 

from .base import Area, AreaType


class Domain(Area):
	""" Domain represents the entire area over which planning occurs. 
		 Its id is defined as 0 and its type is free by definition.
	"""

	def __init__(self, bounding_polygon, ingress_point=None, egress_point=None):
		self._id = 0
		self._type = AreaType.FREE
		self._polygon = bounding_polygon
		self._vertices = list(self._polygon.exterior.coords)

		self._obstacles = {}

		self._ingress_point = ingress_point
		self._egress_point = egress_point

	@classmethod
	def from_box_corners(cls, corner1, corner2, ingress_point=None, egress_point=None):
		bounding_box = shapely.geometry.box(*corner1, *corner2)

		if ingress_point is None and egress_point is None:
			ingress_point = corner1
			egress_point = corner1
		elif egress_point is None:
			egress_point = ingress_point
		elif ingress_point is None:
			ingress_point = egress_point

		return cls(bounding_box, ingress_point, egress_point)

	def add_obstacle(self, obstacle):
		self._obstacles[obstacle.id] = obstacle
		self._vertices.extend(obstacle.vertices)

	def add_obstacles(self, *obstacles):
		for o in obstacles:
			self._obstacles[o.id] = o
			self._vertices.extend(o.vertices)

	def compute_intersection(self, obj):
		# If obj does not intersect domain boundary,
		# it wont intersect anything in the domain
		if not self._polygon.intersects(obj):
			return []

		intersection = self._polygon.intersection(obj)
		intersection_points = [IntersectionPoint(c, self._id) for c in intersection.coords]


		for o in self._obstacles.values():
			if o.polygon.intersects(obj):
				intersection = o.polygon.intersection(obj)

				# TODO: Do this some other way - handle multilinestring, linestring, and point
				if isinstance(intersection, collections.Iterable):
					intersection = shapely.ops.linemerge(intersection)	
				
				if not isinstance(intersection, collections.Iterable):
					intersection = (intersection,)
				
				intersection_list = list(intersection)

				new_points = [IntersectionPoint(c, o.id) for intersect in intersection_list for c in intersect.coords]

				# If intersects at a single point, add this point to the list twice
				if len(new_points) == 1:
					new_points *= 2

				intersection_points.extend(new_points)

		return intersection_points

	def get_configuration_space(self, vehicle_radius):
		offset_boundary = self._polygon.buffer(-vehicle_radius, join_style=2)
		offset_obstacles = [o.polygon.buffer(vehicle_radius, join_style=1) for o in self._obstacles.values()]

		return (offset_boundary, offset_obstacles)

	@property
	def polygon(self):
		return shapely.geometry.Polygon(self._polygon.exterior.coords, holes=[o.polygon.exterior.coords for o in self._obstacles.values()])

	@property
	def vertices(self):
		return self._vertices
	
	@property
	def bounds(self):
		return self._polygon.bounds
	
	@property
	def obstacles(self):
		return self._obstacles

	@property
	def ingress_point(self):
		return self._ingress_point
	
	@property
	def egress_point(self):
		return self._egress_point
	

class Obstacle(Area):

	id_num = 1

	def __init__(self, polygon):
		self._id = Obstacle.id_num
		Obstacle.id_num += 1
		self._type = AreaType.OBSTACLE
		self._polygon = polygon
		self._vertices = list(polygon.exterior.coords)

	@classmethod
	def from_vertex_list(cls, vertices):
		polygon = shapely.geometry.Polygon(vertices)

		return cls(polygon)

	@property
	def vertices(self):
		return self._vertices