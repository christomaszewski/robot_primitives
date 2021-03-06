import shapely.geometry
import shapely.ops
import numpy as np
import collections
import operator
import os
import json

from .base import Area, AreaType

class AreaJSONEncoder(json.JSONEncoder):

	def default(self, obj):
		if issubclass(obj, Area) and hasattr(obj, 'json_repr'):
			return obj.json_repr()
		else:
			return json.JSONEncoder.default(self, obj)

	@classmethod
	def decode(cls, json_dict):
		bounding_polygon = shapely.geometry.Polygon(json_dict['vertices'])
		return cls(bounding_polygon, area_type=json_dict['type'], identifier=json_dict['id'])

class Region(Area):
	""" A basic type of Area that implements a variety of useful methods 
		 other Area classes can inherit
	"""

	def _init__(self, bounding_polygon, area_type=AreaType.UNDEFINED, identifier=0):
		self._polygon = bounding_polygon
		self._type = area_type
		self._id = identifier
		self._vertices = list(self._polygon.exterior.coords)[:-1]

	def get_side(self, side_index):
		if side_index >= self.num_sides:
			raise IndexError(f"Error: Requested side index {side_index} > number of defined sides {self.num_sides}.")
		
		return self._polygon.exterior.coords[side_index:side_index+2] 

	def contains_point(self, point):
		return self._polygon.intersects(shapely.geometry.Point(*point))

	@property
	def vertices(self):
		return self._vertices

	@property
	def num_sides(self):
		return len(self._vertices)

	@property
	def interior_angles(self):
		""" Return interior angles of polygon that defines the area """
		coord_list = list(self._polygon.exterior.coords)[:-1]

		interior_angles = {}

		for i, pt in enumerate(coord_list):
			curr_pt = np.array(pt)
			prev_pt = np.array(coord_list[i-1])
			next_pt = np.array(coord_list[(i+1)%len(coord_list)])

			a = prev_pt - curr_pt
			b = next_pt - curr_pt
			cos_angle = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
			angle = np.degrees(np.arccos(cos_angle))

			interior_angles[pt] = angle

		return interior_angles

	@property
	def bounds(self):
		return self._polygon.bounds

	@property
	def area(self):
		return self._polygon.area

	@property
	def diameter(self):
		# returns maximum diameter of polygon?
		min_x, min_y, max_x, max_y = self.bounds
		return np.linalg.norm((max_x-min_x, max_y-min_y))

class Domain(Region):
	""" Domain represents the entire area over which planning occurs. 
		 Its id is defined as 0 and its type is free by definition.
	"""

	json_encoder = AreaJSONEncoder

	def __init__(self, bounding_polygon, ingress_point=None, egress_point=None):
		self._id = 0
		self._type = AreaType.FREE
		self._polygon = bounding_polygon
		self._vertices = list(self._polygon.exterior.coords)[:-1] # Dropping the last repeated point, needs testing, may break stuff

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

	@classmethod
	def from_vertex_list(cls, vertices, ingress_point=None, egress_point=None):
		bounding_polygon = shapely.geometry.Polygon(vertices)

		return cls(bounding_polygon, ingress_point, egress_point)

	@classmethod
	def from_json_dict(cls, json_dict):
		bounding_polygon = shapely.geometry.Polygon(json_dict['vertices'])

		return cls(bounding_polygon, ingress_point=json_dict['ingress'], egress_point=json_dict['egress'])

	@classmethod
	def from_file(cls, filename):
		extension = os.path.splitext(filename)[1][1:]
		with open(filename, mode='r') as f:
			if extension == 'json':
				domain = json.load(f, object_hook=Domain.from_json_dict)
			else:
				print(f"Error: Unrecognized extension {extension}, supported extension is json")
				domain = None

		return domain

	def json_repr(self):
		return dict(id=self._id, vertices=self._vertices, ingress=self._ingress_point, egress=self._egress_point)

	def save(self, filename):
		extension = os.path.splitext(filename)[1][1:]
		with open(filename, 'w') as f:
			if extension == 'json':
				json.dump(self, f, skipkeys=True, cls=Domain.json_encoder, indent=2)
			else:
				print(f"Error: Unrecognized extension {extension}, supported extension is json")

	def add_obstacle(self, obstacle):
		self._obstacles[obstacle.id] = obstacle
		self._vertices.extend(obstacle.vertices)

	def add_obstacles(self, *obstacles):
		for o in obstacles:
			self._obstacles[o.id] = o
			self._vertices.extend(o.vertices)

	def compute_intersection(self, obj):
		if not self._polygon.intersects(obj):
			return []

		intersection  = self._polygon.intersection(obj)

		return [np.array(c) for c in intersection.coords]
		# # If obj does not intersect domain boundary,
		# # it wont intersect anything in the domain
		# if not self._polygon.intersects(obj):
		# 	return []

		# intersection = self._polygon.intersection(obj)
		# intersection_points = [IntersectionPoint(c, self._id) for c in intersection.coords]


		# for o in self._obstacles.values():
		# 	if o.polygon.intersects(obj):
		# 		intersection = o.polygon.intersection(obj)

		# 		# TODO: Do this some other way - handle multilinestring, linestring, and point
		# 		if isinstance(intersection, collections.Iterable):
		# 			intersection = shapely.ops.linemerge(intersection)	
				
		# 		if not isinstance(intersection, collections.Iterable):
		# 			intersection = (intersection,)
				
		# 		intersection_list = list(intersection)

		# 		new_points = [IntersectionPoint(c, o.id) for intersect in intersection_list for c in intersect.coords]

		# 		# If intersects at a single point, add this point to the list twice
		# 		if len(new_points) == 1:
		# 			new_points *= 2

		# 		intersection_points.extend(new_points)

		# return intersection_points

	def get_configuration_space(self, vehicle_radius):
		offset_boundary = self._polygon.buffer(-vehicle_radius, join_style=2)
		offset_obstacles = [o.polygon.buffer(vehicle_radius, join_style=1) for o in self._obstacles.values()]

		return (offset_boundary, offset_obstacles)

	def line_of_sight(self, p1, p2):
		line = shapely.geometry.LineString([p1, p2])

		if not self._polygon.contains(line):
			return False

		for o in self._obstacles.values():
			if line.intersects(o.polygon):
				return False

		return True

	def offset_domain(self, offset):
		offset_boundary = self._polygon.buffer(-offset, join_style=2)
		offset_obstacles = [o.polygon.buffer(offset, join_style=1) for o in self._obstacles.values()]

		d = Domain(offset_boundary, self._ingress_point, self._egress_point)

		for o in offset_obstacles:
			d.add_obstacle(Obstacle(o))

		return d

	@property
	def polygon(self):
		return shapely.geometry.Polygon(self._polygon.exterior.coords, holes=[o.polygon.exterior.coords for o in self._obstacles.values()])
	
	@property
	def obstacles(self):
		return self._obstacles

	@property
	def ingress_point(self):
		return self._ingress_point
	
	@property
	def egress_point(self):
		return self._egress_point
	

class Obstacle(Region):

	id_num = 1

	def __init__(self, polygon):
		self._id = Obstacle.id_num
		Obstacle.id_num += 1
		self._type = AreaType.OBSTACLE
		self._polygon = polygon
		self._vertices = list(polygon.exterior.coords)[:-1] # Dropping last repeated point, needs testing

	@classmethod
	def from_vertex_list(cls, vertices):
		polygon = shapely.geometry.Polygon(vertices)

		return cls(polygon)