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

		pylon_flow = lambda x: pylon_boundary_flow[0] - 2*x*pylon_boundary_flow[0]/pylon_width if x < pylon_bounds[0] + pylon_width/2 else 2*x*pylon_boundary_flow[1]/pylon_width

		compound_flow = lambda x,y: (0, channel_flow(x)) if x < pylon_bounds[0] or x > pylon_bounds[1] else (0, pylon_flow(x))

		return cls(compound_flow)

	def __getitem__(self, index):
		return self._field_func(*index)