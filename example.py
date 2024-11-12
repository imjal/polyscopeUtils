import numpy as np
import polyscope as ps

from polyscopeUtils.geometry import GeometryPrimitives


ps.init()

basis = GeometryPrimitives.createCoordinateBasis(np.eye(3), color=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], radius=0.025, scale=1)
name = "coordBasis"
ps_coord = ps.register_surface_mesh(f"{name}", np.asarray(basis.vertices), np.asarray(basis.triangles), transparency=1) 
ps_coord.add_color_quantity(f"{name}_colors", np.asarray(basis.vertex_colors), defined_on='vertices', enabled=True)
ps_coord.set_smooth_shade(True)

ps.show()