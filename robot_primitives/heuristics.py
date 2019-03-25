import numpy as np

from .base import Heuristic

class EuclideanDistance(Heuristic):

	@staticmethod
	def compute_cost(start_point, end_point):
		return np.linalg.norm(np.asarray(end_point)-np.asarray(start_point))


class OpposingFlowEnergy(Heuristic):

	def __init__(self, flow_field, delta=0.01):
		self._flow_field = flow_field
		self._delta = delta

	def compute_cost(self, start_point, end_point, nominal_speed=0.5):
		start = np.array(start_point)
		end = np.array(end_point)

		diff = end - start
		length = np.linalg.norm(diff)

		nominal_vel = (diff / length) * nominal_speed

		step = nominal_vel * self._delta

		segment_start = start.copy()
		segment_end = start + step
		start_vel = np.array(self._flow_field[segment_start])
		end_vel = np.array(self._flow_field[segment_end])

		total_cost = 0.
		while np.linalg.norm(segment_end - start) < length:
			avg_vel = (start_vel + end_vel)/2.
			boat_vel = nominal_vel - avg_vel
			segment_cost = np.linalg.norm(boat_vel)*self._delta
			total_cost += segment_cost

			segment_start += step
			segment_end += step
			start_vel = end_vel
			end_vel = np.array(self._flow_field[segment_end])

		return total_cost

	"""
	def compute_cost(self, start_point, end_point):
		start = np.array(start_point)
		end = np.array(end_point)

		diff = end - start
		length = np.linalg.norm(diff)
		step = self._delta * (diff / length)

		segment_start = start.copy()
		segment_end = start + step
		start_vel = np.array(self._flow_field[segment_start])
		end_vel = np.array(self._flow_field[segment_end])
		total_cost = 0.
		while np.linalg.norm(segment_end - start) < length:
			total_cost += np.sum((segment_end - segment_start)*((end_vel + start_vel)/2.))

			segment_start += step
			segment_end += step
			start_vel = end_vel
			end_vel = np.array(self._flow_field[segment_end])

		return total_cost
		"""
