import numpy as np

import open3d as o3d
import glm


def getCylinderTransform(endpoints):
    a = endpoints[1]-endpoints[0]
    a = glm.vec3(a[0], a[1], a[2])
    b = glm.vec3(0, 0, 1)
    
    mat = glm.mat4()
    # translate
    mat = glm.translate(mat, glm.vec3(endpoints[0][0], endpoints[0][1], endpoints[0][2]))
    # rotate
    v = glm.cross(b, a)
    if v!= glm.vec3(0, 0, 0):
        angle = glm.acos(glm.dot(b, a) / (glm.length(b) * glm.length(a)))
        mat = glm.rotate(mat, angle, v)

    # scale
    scale_factor = glm.length(a)
    mat = glm.scale(mat, glm.vec3(scale_factor, scale_factor, scale_factor))

    return mat

class GeometryPrimitives:

    def __init__(self) -> None:
        self.objects = []

    def add_obj(self, obj:o3d.geometry.TriangleMesh)->None:
        self.objects.append(obj)
    
    @staticmethod
    def collapseMeshObjects(objects):
        mesh = o3d.geometry.TriangleMesh()
        for obj in objects:
            mesh += obj
        return mesh

    @staticmethod
    def createSphere(radius=0.025, center=[0, 0, 0], resolution=20, color=[0, 0, 0])->o3d.geometry.TriangleMesh:
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=resolution)
        mesh.translate(center)
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.array([color]*len(mesh.vertices)))
        mesh.compute_vertex_normals()
        return mesh
    
    @staticmethod
    def createCylinder(endpoints, radius=0.025/2, resolution=20, color=[0, 0, 0])->o3d.geometry.TriangleMesh:
        #canonical cylinder is along z axis with height 1 and centered
        mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=1, resolution=resolution)
        mesh.translate([0, 0, 1/2])

        matrix = getCylinderTransform(endpoints)
        mesh.transform(matrix)
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.array([color]*len(mesh.vertices)))
        mesh.compute_vertex_normals()
        return mesh
    
    @staticmethod
    def calculate_zy_rotation_for_arrow(vec):
        gamma = np.arctan2(vec[1], vec[0])
        Rz = np.array([
                        [np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]
                    ])

        vec = Rz.T @ vec

        beta = np.arctan2(vec[0], vec[2])
        Ry = np.array([
                        [np.cos(beta), 0, np.sin(beta)],
                        [0, 1, 0],
                        [-np.sin(beta), 0, np.cos(beta)]
                    ])
        return Rz, Ry
    
    @staticmethod
    def get_arrow(end, origin=np.array([0, 0, 0]), scale=1):
        assert(not np.all(end == origin))
        vec = end - origin
        size = np.sqrt(np.sum(vec**2))
        ratio_cone_cylinder = 0.15
        radius = 60
        ratio_cone_bottom_to_cylinder = 2

        Rz, Ry = GeometryPrimitives.calculate_zy_rotation_for_arrow(vec)
        mesh = o3d.geometry.TriangleMesh.create_arrow(cone_radius=1/radius * ratio_cone_bottom_to_cylinder * scale,
            cone_height= size * ratio_cone_cylinder* scale,
            cylinder_radius=1/radius* scale,
            cylinder_height=size * (1 - ratio_cone_cylinder *scale))
        mesh.rotate(Ry, center=np.array([0, 0, 0]))
        mesh.rotate(Rz, center=np.array([0, 0, 0]))
        mesh.translate(origin)
        return(mesh)
    
    @staticmethod
    def createArrow(endpoints, radius=0.025/2, resolution=20, scale=1, color=[0, 0, 0])->o3d.geometry.TriangleMesh:
        # mesh = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=radius, cone_radius= radius * 2, cylinder_height=0.95, cone_height=0.05, resolution=resolution)
        # matrix = getCylinderTransform(endpoints)
        # mesh.transform(matrix)
        mesh = GeometryPrimitives.get_arrow(endpoints[1], endpoints[0], scale=scale)
        mesh.compute_vertex_normals()
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.array([color]*len(mesh.vertices)))
        return mesh
    
    @staticmethod
    def createCoordinateBasis(basis, radius=0.025/2, resolution=20,scale=1, color=[0, 0, 0])->o3d.geometry.TriangleMesh:
        meshes = []
        for i, b in enumerate(basis):
            mesh = GeometryPrimitives.createArrow(endpoints=[[0, 0, 0], b], radius=radius, resolution=resolution, color=color[i], scale=scale)
            meshes.append(mesh)
        mesh = GeometryPrimitives.collapseMeshObjects(meshes)
        return mesh
    
    def createMaxBasis(self, points, rgbs, lines, ball_radius=0.025): 
        for rgb, point in zip(rgbs, points): 
            self.add_obj(GeometryPrimitives.createSphere(center=point, color=rgb, radius=ball_radius))
        for line in lines:
            self.add_obj(GeometryPrimitives.createCylinder(endpoints=[points[line[0]], points[line[1]]], color=[0, 0, 0]))

    def render(self):
        # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
        o3d.visualization.draw_geometries(self.objects)
        return