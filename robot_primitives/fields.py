import numpy as np
import shapely.geometry

from .base import Field

class VectorField(Field):

	def __init__(self, field_func):
		self._field_func = field_func

	@classmethod
	def from_uniform_vector(cls, flow_vector):
		field_func = lambda x,y: flow_vector
		return cls(field_func)

	@classmethod
	def from_channel_flow_model(cls, channel_width, max_velocity, offset=(0,0)):
		x0,y0 = offset
		field_func = lambda x,y: (0, ((4 * (x - x0) / channel_width - 4 * (x - x0)**2 / channel_width**2) * max_velocity))

		return cls(field_func)

	@classmethod
	def from_channel_flow_with_pylon(cls, channel_width, max_velocity, pylon_bounds):
		channel_flow = lambda x: (4 * x / channel_width - 4 * x**2 / channel_width**2) * max_velocity

		pylon_width = pylon_bounds[1] - pylon_bounds[0]

		pylon_boundary_flow = [channel_flow(x) for x in pylon_bounds]

		pylon_flow = lambda x: pylon_boundary_flow[0] - 2*(x-pylon_bounds[0])*pylon_boundary_flow[0]/pylon_width if x < pylon_bounds[0] + pylon_width/2 else 2*(x-pylon_bounds[0]-pylon_width/2)*pylon_boundary_flow[1]/pylon_width

		compound_flow = lambda x,y: (0, channel_flow(x)) if x < pylon_bounds[0] or x > pylon_bounds[1] else (0, pylon_flow(x))

		return cls(compound_flow)

	def __getitem__(self, index):
		return self._field_func(*index)

class BoundedVectorField(VectorField):

	def __init__(self, field_func, bounding_region, undefined_value=(0.,0.)):
		self._field_func = field_func
		self._bounding_region = bounding_region
		self._undefined_value = undefined_value

	@classmethod
	def channel_flow_model(cls, bounding_region, center_axis, max_velocity, channel_width=None, **other_args):
		print(f"center_axis: {center_axis}")
		center_axis_vector = np.array([center_axis[1][0]-center_axis[0][0], center_axis[1][1] - center_axis[0][1]])
		center_axis_length = np.linalg.norm(center_axis_vector)
		center_axis_direction = center_axis_vector/center_axis_length
		print(f"center_axis_vector: {center_axis_vector}")

		if not channel_width:
			# Compute channel width from bounding_region
			perpendicular_vector = np.array([-center_axis_vector[1], center_axis_vector[0]])

			perpendicular_direction = perpendicular_vector/np.linalg.norm(perpendicular_vector)
			bounding_verts_scalar_proj = [np.dot(np.array(vert), perpendicular_direction) for vert in bounding_region.vertices]
			channel_width = abs(max(bounding_verts_scalar_proj) - min(bounding_verts_scalar_proj))

			print(f"channel_width: {channel_width}")

		dist = lambda x,y: np.cross(np.array([x-center_axis[0][0], y - center_axis[0][1]]), center_axis_vector)/center_axis_length

		#field_magnitude = lambda x,y: (4 * (dist(x,y)) / channel_width - 4 * (dist(x,y))**2 / channel_width**2) * max_velocity
		field_magnitude = lambda x,y: (4 * (dist(x,y)+channel_width/2) / channel_width - 4 * (dist(x,y)+channel_width/2)**2 / channel_width**2) * max_velocity

		field_func = lambda x,y: tuple(field_magnitude(x,y)*center_axis_direction)

		return cls(field_func, bounding_region, **other_args)

	@classmethod
	def extended_channel_flow_model(cls, bounding_region, center_axis, max_velocity, min_velocity=0., channel_width=None, **other_args):
		print(f"center_axis: {center_axis}")
		center_axis_vector = np.array([center_axis[1][0]-center_axis[0][0], center_axis[1][1] - center_axis[0][1]])
		center_axis_length = np.linalg.norm(center_axis_vector)
		center_axis_direction = center_axis_vector/center_axis_length
		print(f"center_axis_vector: {center_axis_vector}")

		perpendicular_vector = np.array([-center_axis_vector[1], center_axis_vector[0]])

		perpendicular_direction = perpendicular_vector/np.linalg.norm(perpendicular_vector)
		bounding_verts_scalar_proj = [np.dot(np.array(vert), perpendicular_direction) for vert in bounding_region.vertices]

		if not channel_width:
			# Compute channel width from bounding_region
			channel_width = abs(max(bounding_verts_scalar_proj) - min(bounding_verts_scalar_proj))

			print(f"channel_width: {channel_width}")

		# Distance of a point (x,y) to the center axis line using cross product
		dist = lambda x,y: np.linalg.norm(np.cross(np.array([x-center_axis[0][0], y - center_axis[0][1]]), center_axis_vector))/center_axis_length

		w = channel_width / 2.
		a = (min_velocity - max_velocity)/(w**2)
		b = (min_velocity - max_velocity)/w - a*w
		print(f"a: {a}, b: {b}")
		field_magnitude = lambda x,y: a*dist(x,y)**2 + b*dist(x,y) + max_velocity
		#field_magnitude = lambda x,y: (4 * (dist(x,y)) / channel_width - 4 * (dist(x,y))**2 / channel_width**2) * max_velocity
		#field_magnitude = lambda x,y: (4 * (dist(x,y)+channel_width/2) / channel_width - 4 * (dist(x,y)+channel_width/2)**2 / channel_width**2) * max_velocity

		field_func = lambda x,y: tuple(field_magnitude(x,y)*center_axis_direction)

		return cls(field_func, bounding_region, **other_args)

	"""
	@classmethod
	def half_channel_flow_model(cls, bounding_region, flow_axis, max_velocity, **other_args):
		flow_axis_vector = np.array([flow_axis[1][0]-flow_axis[0][0], flow_axis[1][1] - flow_axis[0][1]])
		flow_axis_length = np.linalg.norm(flow_axis_vector)
		flow_axis_direction = flow_axis_vector/flow_axis_length

		# Compute perpendicular vector
		perpendicular_vector = np.array([-center_axis_vector[1], center_axis_vector[0]])
		perpendicular_direction = perpendicular_vector/np.linalg.norm(perpendicular_vector)

		bounding_verts_scalar_proj = [np.dot(np.array(vert), perpendicular_direction) for vert in bounding_region.vertices]
		half_channel_width = abs(max(bounding_verts_scalar_proj) - min(bounding_verts_scalar_proj))

	"""

	@classmethod
	def linear_flow_model(cls, bounding_region, flow_axis, v1, v2, **other_args):
		"""
		x_coords = [vert[0] for vert in bounding_region.vertices]
		y_coords = [vert[1] for vert in bounding_region.vertices]
		origin = np.array([min(x_coords), min(y_coords)])

		offset_flow_axis = [np.array(flow_axis[0]) - origin, np.array(flow_axis[1]) - origin]
		"""


		flow_axis_vector = np.array([flow_axis[1][0] - flow_axis[0][0], flow_axis[1][1] - flow_axis[0][1]])
		flow_axis_length = np.linalg.norm(flow_axis_vector)
		flow_axis_direction = flow_axis_vector / flow_axis_length

		# Compute perpendicular vector
		perpendicular_vector = np.array([-flow_axis_vector[1], flow_axis_vector[0]])
		#perpendicular_vector = np.array([flow_axis_vector[1], -flow_axis_vector[0]])
		# perpendicular_vector = np.array([-center_axis_vector[1], center_axis_vector[0]])
		perpendicular_direction = perpendicular_vector/np.linalg.norm(perpendicular_vector)

		# Take absolute value of project here so that it shouldn't matter which orientation of perpendicular vector we use
		bounding_verts_scalar_proj = [abs(np.dot(np.array(vert), perpendicular_direction)) for vert in bounding_region.vertices]
		x1 = min(bounding_verts_scalar_proj)
		x2 = max(bounding_verts_scalar_proj)
		projected_region_width = x2 - x1

		slope = (v2 - v1) / projected_region_width

		projected_dist = lambda x,y: abs(np.dot(np.array([x,y]), perpendicular_direction)) - x1

		field_magnitude = lambda x,y: v1 + slope * projected_dist(x,y)

		field_func = lambda x,y: tuple(field_magnitude(x,y)*flow_axis_direction)

		return cls(field_func, bounding_region, **other_args)

	@classmethod
	def unidirectional_poly_flow_model(cls, bounding_region, flow_dir, measurement_pts, flow_speeds, poly_deg, **other_args):
		#flow_axis_vector = np.array([flow_axis[1][0] - flow_axis[0][0], flow_axis[1][1] - flow_axis[0][1]])
		#flow_axis_length = np.linalg.norm(flow_axis_vector)
		flow_dir /= np.linalg.norm(flow_dir)
		perp_flow_dir = np.array([-flow_dir[1], flow_dir[0]])

		proj_pts = [np.dot(pt, perp_flow_dir) for pt in measurement_pts]
		field_magnitude = np.polynomial.Polynomial.fit(proj_pts, flow_speeds, poly_deg)

		projected_dist = lambda x,y: np.dot(np.array([x,y], perp_flow_dir))

		field_func = lambda x,y: tuple(field_magnitude(projected_dist(x,y))*flow_dir)

		return cls(field_func, bounding_region, **other_args)

	def __getitem__(self, index):
		#Todo: remove this once it is verified new contains point method works
		#if not self._bounding_region.polygon.contains(shapely.geometry.Point(*index)):	
		if not self._bounding_region.contains_point(index):
			#print('Error: Specified point lies out of valid boundary for this field')
			return self._undefined_value
		else:
			return self._field_func(*index)

	@property
	def undefined_value(self):
		return self._undefined_value

	@undefined_value.setter
	def undefined_value(self, new_val):
		self._undefined_value = new_val


class RadialChannelField(BoundedVectorField):

	def __init__(self, bounding_region, origin, min_vel=0., max_vel=0.5, undefined_value=(0.,0.)):
		self._bounding_region = bounding_region
		self._origin = np.array(origin)
		self._undefined_value = undefined_value

		# Precompute sweep line distance that will cross entire domain regardless of origin
		self._region_diameter = self._bounding_region.diameter + 1.0

		self._min_vel = min_vel
		self._max_vel = max_vel

	def _field_magnitude(self, width, dist):
		a = (self._min_vel - self._max_vel)/(width**2)
		b = (self._min_vel - self._max_vel)/width - a*width
		return a*dist**2 + b*dist + self._max_vel

	def _field_func(self, x, y):
		pt = np.array((x,y))
		pt_vec = pt - self._origin
		sweep_vec = pt_vec/np.linalg.norm(pt_vec) * self._region_diameter + self._origin
		cross_section = self._bounding_region.compute_intersection(shapely.geometry.LineString([self._origin, sweep_vec]))

		cross_vec = cross_section[1] - cross_section[0]
		cross_len = np.linalg.norm(cross_vec)
		midpoint = 0.5 * cross_vec + cross_section[0]

		dist = np.linalg.norm(pt - midpoint)
		flow_magnitude = self._field_magnitude(cross_len/2., dist)
		flow_direction = np.array([pt_vec[1], -pt_vec[0]])/np.linalg.norm(pt_vec)

		return tuple(flow_magnitude*flow_direction)

class AsymmetricRadialChannelField(BoundedVectorField):

	def __init__(self, bounding_region, origin, min_vel=0., max_vel=0.5, center_ratios=(0.5, 0.75), undefined_value=(0.,0.)):
		self._bounding_region = bounding_region
		self._origin = np.array(origin)
		self._undefined_value = undefined_value
		self._center_ratios = center_ratios

		# Precompute sweep line distance that will cross entire domain regardless of origin
		self._region_diameter = self._bounding_region.diameter + 1.0

		self._min_vel = min_vel
		self._max_vel = max_vel

	def _field_magnitude(self, width, dist):
		a = (self._min_vel - self._max_vel)/(width**2)
		b = (self._min_vel - self._max_vel)/width - a*width
		return a*dist**2 + b*dist + self._max_vel

	def _field_func(self, x, y):
		pt = np.array((x,y))
		pt_vec = pt - self._origin
		pt_len = np.linalg.norm(pt_vec)

		angle = np.arctan2(pt_vec[1], pt_vec[0])
		center_ratio = np.linalg.norm((self._center_ratios[0]*np.cos(angle), self._center_ratios[1]*np.sin(angle)))

		sweep_vec = pt_vec/pt_len * self._region_diameter + self._origin
		cross_section = self._bounding_region.compute_intersection(shapely.geometry.LineString([self._origin, sweep_vec]))

		cross_vec = cross_section[1] - cross_section[0]
		cross_len = np.linalg.norm(cross_vec)
		center_point = center_ratio * cross_vec + cross_section[0]
		center_point_vec = center_point - self._origin
		center_point_len = np.linalg.norm(center_point_vec)

		flow_magnitude = 0.
		if pt_len > center_point_len:
			# pt on outer bank side of flow center
			dist = pt_len - center_point_len
			width = (1. - center_ratio) * cross_len
			flow_magnitude = self._field_magnitude(width, dist)
		else:
			# pt on inner bank side of flow center
			dist = center_point_len - pt_len
			width = center_ratio * cross_len
			flow_magnitude = self._field_magnitude(width, dist)

		flow_direction = np.array([pt_vec[1], -pt_vec[0]])/pt_len

		return tuple(flow_magnitude*flow_direction)