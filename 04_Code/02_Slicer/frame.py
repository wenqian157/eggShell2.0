
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections
from slice import Slice

class Frame():

    """This class manipulates a frame
    """

    def __init__(self, curve_list, point_list, layer_height):

        self.curve_list = curve_list
        self.point_list = point_list
        self.layer_height = layer_height

        self.normals_list = self.tcp_normals()

        self.tcp_frames = self.construct_tcp_frames(self.point_list,
            self.normals_list)

        self.tcp_frames = self.spiral(self.tcp_frames, self.layer_height)

        return self.tcp_frames


    def spiral(self, frames, layer_height):

        frames_per_layer = 0

        for f in frames:

            if self.is_almost_equal(f.OriginZ, 0):

                frames_per_layer += 1

            else:

                break

        count = 0

        move_dist = layer_height/frames_per_layer

        for i, f in enumerate(frames):

            move_trans = rg.Transform.Translation(0,0,move_dist*(i%frames_per_layer))
            f.Transform(move_trans)

        return frames


    def cross_product(self, vector_a, vector_b):

        """Return the perpendicular vector
        from two given vectors
        """
        a_x, a_y, a_z = vector_a[0], vector_a[1], vector_a[2]
        b_x, b_y, b_z = vector_b[0], vector_b[1], vector_b[2]

        c_x, c_y, c_z = (a_y*b_z - a_z*b_y), (a_z*b_x - a_x*b_z), (a_x*b_y - a_y*b_x)

        vector_c = rg.Vector3d(c_x, c_y, c_z)

        return vector_c

    def construct_tcp_frames(self, point_list, normals_list):

        vector_x0 = [0,1,0]

        tcp_frame_list = []

        print len(normals_list)

        for point, normal in zip(point_list, normals_list):

            vector_y = rg.Vector3d(self.cross_product(vector_x0, normal))

            vector_x = rg.Vector3d(self.cross_product(normal, vector_y))

            tcp_frame = rg.Plane(point, vector_y, vector_x)

            tcp_frame_list.append(tcp_frame)

        return tcp_frame_list


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon

    def tcp_normals(self):

        normals_list = []

        start_shift = 0

        for i, c in enumerate(self.curve_list[1::]):

            c_height = c.PointAtStart.Z

            for j, p in enumerate(self.point_list[start_shift::]):

                p_height = p.Z

                if self.is_almost_equal(c_height, p_height):

                    _, c_double = self.curve_list[i].ClosestPoint(p)
                    c_point = self.curve_list[i].PointAt(c_double)

                    vector01 = rg.Vector3d(c_point)
                    vector02 = rg.Vector3d(p)
                    p_normal = vector02 - vector01
                    p_normal.Unitize()

                    # perp_vector = rg.Vector3d(0,0,1)
                    # p_normal = p_normal + perp_vector
                    # p_normal = p_normal + perp_vector

                    normals_list.append(p_normal)

                    start_shift += 1

        return normals_list
