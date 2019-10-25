from abc import ABC, abstractmethod
from enum import Enum

class AreaType(Enum):
	FREE = 0
	OBSTACLE = 1
	UNDEFINED = 2


class Area(ABC):

	@property
	def id(self):
		""" All areas must have valid id """
		return self._id

	@property
	def type(self):
		""" All areas must have valid type """
		return self._type

	@property
	def polygon(self):
		""" All areas must have defining polygon """
		return self._polygon

	@property
	def interior_angles(self):
		""" Return interior angles of polygon that defines the area """
		coord_list = list(self.polygon.exterior.coords)[:-1]

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
	@abstractmethod
	def vertices(self):
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is Area:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented
	

class Path(ABC):

	@property
	def coord_list(self):
		return self._coord_list

	@property
	@abstractmethod
	def length(self):
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is Path:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented
	

class Field(ABC):

	@abstractmethod
	def __getitem__(self, index):
		"""Method to sample field at given point

		Args:
			index (2-tuple): point at which to sample the field

		Returns:
			value (tuple): value sampled from field at index

		"""
		raise NotImplementedError()


	@property
	def boundary(self):
		return None
	

class Heuristic(ABC):

	@abstractmethod
	def compute_cost(self, start_point, end_point):
		""" Approximate the cost of movement between two points """
		raise NotImplementedError()