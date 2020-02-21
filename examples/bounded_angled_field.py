from context import robot_primitives as rp

domain  = rp.areas.Domain.from_vertex_list([(5.,0.), (10.,5.), (5.,10.), (0.,5.)])
center_axis = [(2.5,2.5), (7.5,7.5)]
max_velocity = 2.0
flow_field = rp.fields.BoundedVectorField.channel_flow_model(domain, center_axis, max_velocity)

pt = (5.,5)

print(f"{pt} vec: {flow_field[pt]}")
