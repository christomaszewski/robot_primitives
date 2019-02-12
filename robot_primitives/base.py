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
	@abstractmethod
	def vertices(self):
		raise NotImplementedError()
	

class Path(ABC):

	@property
	def coord_list(self):
		return self._coord_list

	@property
	@abstractmethod
	def length(self):
		raise NotImplementedError()
	

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


class Heuristic(ABC):

	@abstractmethod
	def compute_cost(self, start_point, end_point):
		""" Approximate the cost of movement between two points """
		raise NotImplementedError()