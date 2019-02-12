import numpy as np

from .base import Path

class ConstrainedPath(Path):

	def __init__(self, coord_list, **constraints):
		self._coord_list = coord_list
		self._constraints = constraints

		# Compute euclidean path length
		self._length = 0.
		for u, v in zip(self._coord_list, self._coord_list[1:]):
			self._length += np.linalg.norm(np.asarray(v) - np.asarray(u))

		for param in self._constraints.keys():
			setattr(ConstrainedPath, param, self._property_factory(param))

	def __getitem__(self, index):
		if index < len(self._coord_list):
			return self._coord_list[index]
		else:
			return None

	def __iadd__(self, other):
		other_constraints = set(other.constrained_parameters)
		other_constraints.difference_update(self._constraints.keys())

		# Update existing constrained params
		for param in self._constraints.keys():
			if other.is_constrained(param):
				self._constraints[param].extend(getattr(other, param))
			else:
				self._constraints[param].extend([None]*other.size)

		# Add newly constrained params, us None for path up to now
		for new_param in other_constraints:
			self._constraints[new_param] = [None]*self.size
			self._constraints[new_param].extend(getattr(other, new_param))

		# Add coords of other path to this path and update length
		self._coord_list.extend(other.coord_list)
		self._length += other.length

		return self

	def _property_factory(self, parameter):
		return property(lambda obj:obj._constraints[parameter], 
							lambda obj, val: obj._constraints.update({parameter:value}), 
							lambda obj:obj._constraints.pop(parameter))

	def is_constrained(self, parameter):
		return parameter in self._constraints

	def add_point(self, point, **constraints):
		self._length += np.linalg.norm(np.asarray(point) - np.asarray(self._coord_list[-1]))
		self._coord_list.append(tuple(point))

		new_keys = set(constraints.keys())
		existing_keys = set(self._constraints.keys())

		undefined_keys = existing_keys.difference(new_keys)

		for k in new_keys:
			self._constraints[k].append(constraints[k])

		for k in undefined_keys:
			self._constraints[k].append(None)

	@property
	def size(self):
		return len(self._coord_list)
	
	@property
	def length(self):
		return self._length
	
	@property
	def constrained_parameters(self):
		return self._constraints.keys()
	