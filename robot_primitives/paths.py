import numpy as np
import json
import os

from .base import Path


class PathJSONEncoder(json.JSONEncoder):

	def default(self, data):
		dict_rep = {'coord_list':data.coord_list, **data.constraints}
		return dict_rep

	@classmethod
	def decode(cls, json_dict):
		coord_list = json_dict.pop('coord_list')

		path = ConstrainedPath(coord_list, **json_dict)
		return path

class ConstrainedPath(Path):

	json_encoder = PathJSONEncoder

	def __init__(self, coord_list, **constraints):
		self._coord_list = coord_list
		self._constraints = constraints

		# Compute euclidean path length
		self._compute_length()

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
		self._compute_length()

		return self

	@classmethod
	def from_file(cls, filename):
		extension = os.path.splitext(filename)[1][1:]
		with open(filename, mode='r') as f:
			if extension == 'json':
				path = json.load(f, object_hook=PathJSONEncoder.decode)
			else:
				print(f"Error: Unrecognized extension {extension}, supported extension is json")
				path = None

		return path

	def save(self, filename):
		extension = os.path.splitext(filename)[1][1:]
		with open(filename, 'w') as f:
			if extension == 'json':
				json.dump(self, f, cls=ConstrainedPath.json_encoder, indent=2)
			else:
				print(f"Error: Unrecognized extension {extension}, supported extension is json")


	def _property_factory(self, parameter):
		return property(lambda obj:obj._constraints[parameter], 
							lambda obj, val: obj._constraints.update({parameter:value}), 
							lambda obj:obj._constraints.pop(parameter))

	def _compute_length(self):
		self._length = 0.
		for u, v in zip(self._coord_list, self._coord_list[1:]):
			self._length += np.linalg.norm(np.asarray(v) - np.asarray(u))

	def transform(self, transform_func):
		self._coord_list = [transform_func(pt) for pt in self._coord_list]
		self._compute_length()

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
	
	@property
	def constraints(self):
		return self._constraints
	