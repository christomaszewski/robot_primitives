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

	def __getitem__(self, index):
		return self._field_func(*index)