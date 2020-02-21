from context import robot_primitives as rp

flow_field = rp.fields.VectorField.from_channel_flow_with_pylon(channel_width=20, max_velocity=-2, pylon_bounds=(8,12))

print(flow_field[(0,0)], flow_field[(6,0)], flow_field[(8,0)], flow_field[(10,0)], flow_field[(12,0)])