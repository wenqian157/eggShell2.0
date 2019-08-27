import Rhino.Geometry as rg
import math

class ImportObj():

    def __init__(self, path, rotate_bool, scale_factor):

        self.path = path
        self.rotate_bool = rotate_bool
        self. scale_factor = scale_factor

    def import_obj(self):

        file = open(self.path)
        lines = file.readlines()

        self.mesh = rg.Mesh()

        for line in lines:

            if line.find("v")==0 and line.find("n")==-1 and line.find("t")==-1:

                self.mesh.Vertices.Add(rg.Point3d(float((line.split(' '))[1]),
                    float((line.split(' '))[2]),float((line.split(' '))[3])))

            if line.find("f")==0:

                if line.split(' ').Count==4:

                    self.mesh.Faces.AddFace(rg.MeshFace(int(line.split(' ')[1].split('/')[0])-1,
                        int(line.split(' ')[2].split('/')[0])-1,int(line.split(' ')[3].split('/')[0])-1))

                if line.split(' ').Count==5:

                    self.mesh.Faces.AddFace(rg.MeshFace(int(line.split(' ')[1].split('/')[0])-1,
                        int(line.split(' ')[2].split('/')[0])-1,int(line.split(' ')[3].split('/')[0])-1,
                        int(line.split(' ')[4].split('/')[0])-1))

    def rotate_mesh(self, angle = math.pi/2):

        """rotate meshes
        """
        self.mesh.Rotate(angle, rg.Vector3d(0,1,0), rg.Point3d(0,0,0))

    def scale_mesh(self):

        """scale mesh
        """
        self.mesh.Scale(self.scale_factor)

    def get_bounding_box(self):

        """get bounding bounding
        """
        self.bounding_box = self.mesh.GetBoundingBox(True)
        corners = self.bounding_box.GetCorners()

        avgx = 0
        avgy = 0

        min_z = 99999

        for p in corners:

            avgx += p.X
            avgy += p.Y

            if p.Z < min_z:
                min_z = p.Z

        x = avgx / len(corners)
        y = avgy / len(corners)

        self.base_center = rg.Point3d(x, y, min_z)

        self.base_radius = self.base_center.DistanceTo(corners[0])

    def move_mesh(self):

        """move to origin
        """
        self.max_z = 0

        self.target_vector = rg.Vector3d(-self.base_center.X, -self.base_center.Y, -self.base_center.Z)
        self.move_transform = rg.Transform.Translation(self.target_vector)

        self.bounding_box.Transform(self.move_transform)

        corners = self.bounding_box.GetCorners()

        for p in corners:

            if p.Z > self.max_z:
                self.max_z = p.Z

        self.base_center.Transform(self.move_transform)

        self.mesh.Transform(self.move_transform)

    def calc_normals(self):

        """calculate normals
        """
        self.mesh.Normals.ComputeNormals()
        normals = self.mesh.FaceNormals
        self.z_values = [n.Z for n in normals]

        self.face_centers = []

        for i in range(len(normals)):
            face_center = self.mesh.Faces.GetFaceCenter(i)
            self.face_centers.append(face_center)
