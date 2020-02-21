import numpy as np
from context import robot_primitives as rp

domain  = rp.areas.Domain.from_vertex_list([(4.,2.), (3.,11.), (10.,12.), (11.,3.)])
flow_axis = [(9.,2.), (8.,13.)]
v1 = 0.5
v2 = 1.5
flow_field = rp.fields.BoundedVectorField.linear_flow_model(domain, flow_axis, v1, v2)

pt = (3.0001,11.)

print(f"{pt} vec: {flow_field[pt]}, mag: {np.linalg.norm(np.array(flow_field[pt]))}")

pt = (10.99999,3.)

print(f"{pt} vec: {flow_field[pt]}, mag: {np.linalg.norm(np.array(flow_field[pt]))}")
