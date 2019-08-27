
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections
import sliceCurve
reload(sliceCurve)
from sliceCurve import SliceCurve

class GenerateWCS():
    """This class generates WCS frames for UR
    """
    def __init__(self, layer_height, line_definition,
        normal_speed, fast_speed, tcp_bool=True):

        self.layer_height = layer_height
        self.line_definition = line_definition
        self.normal_speed = normal_speed
        self.fast_speed = fast_speed
        self.tcp_bool = tcp_bool


    def generate_WCS_frames(self, slice_curves):
        """generates WCS frames
        """
        tcp_frames = self.construct_tcp_frames(slice_curves)
        # self.normals_list = self.tcp_normals(nested_curve_list, self.line_definition)
        # self.tcp_frames = self.spiral(self.tcp_frames, self.layer_height)

        WCS_list = []
        extrude = 1

        for i, f in enumerate(tcp_frames):

            speed = self.normal_speed
            vector_x = f.XAxis
            vector_y = f.YAxis
            p = f.Origin
            safety_point = False

            if i==0 or i==len(tcp_frames)-1: radius = 0
            else:
                radius, max_length = self.calc_blend_radius(p,
                tcp_frames[i-1].Origin, tcp_frames[i+1].Origin)

                if max_length > self.line_definition*2:
                    radius = 0
                    safety_point = True
                    safety_point01 = [p.X, p.Y, p.Z+self.line_definition, vector_x.X, vector_x.Y,
                        vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
                        self.normal_speed, 0, 0]
                    safety_point02 = [tcp_frames[i+1].Origin.X, tcp_frames[i+1].Origin.Y,
                        p.Z+1, vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y,
                        vector_y.Z, self.normal_speed, 0, 0]

                # dist = p.Z - tcp_frames[i+1].Z
                #
                # if dist > self.line_definition*2: safety_point = [point_list[i+1].X, point_list[i+1].Y,
                #     p.Z+1, vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y,
                #     vector_y.Z, self.normal_speed, 0, 0]

            temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            WCS_list.append(temp_list)

            if safety_point: WCS_list.extend([safety_point01,safety_point02])

        point_list = [rg.Point3d(WCS[0], WCS[1], WCS[2]) for WCS in WCS_list]

        return WCS_list, point_list


    def calc_blend_radius(self, current_point, prev_point,
        next_point, dfillet=20, buffer=0.7):
        """calculates blend radius of path
        """
        radius = min((prev_point - current_point).Length/2 * buffer,
            (next_point - current_point).Length/2 * buffer, dfillet)
        max_length = (next_point - current_point).Length/2

        return radius, max_length


    def spiral(self, frames, layer_height):
        """returns frames in a spiral
        """
        frames_per_layer = 0

        for f in frames:
            # if SliceCurve.is_almost_equal(f.OriginZ, 0):
            if 0 < f.OriginZ < 0.1: frames_per_layer += 1
            else: break

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


    def construct_tcp_frames(self, slice_curves, normals_list=[]):
        """constructs tcp frames
        """
        vector_x0 = [-1,0,0]
        normal = rg.Vector3d(0,0,1)
        tcp_frame_list = []

        point_list = [s.resample_points_by_count() for s in slice_curves]
        point_list = [item for sublist in point_list for item in sublist]

        if len(normals_list) > 1:
            for point, normal in zip(point_list, normals_list):
                vector_y = rg.Vector3d(self.cross_product(vector_x0, normal))
                vector_x = rg.Vector3d(self.cross_product(normal, vector_y))
                tcp_frame = rg.Plane(point, vector_y, vector_x)
                tcp_frame_list.append(tcp_frame)

        else:
            for point in point_list:
                tcp_frame = rg.Plane(point, normal)
                tcp_frame_list.append(tcp_frame)

        return tcp_frame_list


    def length_of_vector(self, vector):
        """calculates length of vector
        """
        return math.sqrt(vector.X**2 + vector.Y**2 + vector.Z**2)


    def dot_product(self, vector01, vector02):
        """calculates dot product
        """
        return vector01.X*vector02.X + vector01.Y*vector02.Y + vector01.Z*vector02.Z


    def angle_two_vectors(self, vector01, vector02):
        """calculates angle between two vectors
        """
        dot_pro = self.dot_product(vector01, vector02)
        len_01 = self.length_of_vector(vector01)
        len_02 = self.length_of_vector(vector02)

        return math.acos(dot_pro/(len_01*len_02))


    def tcp_normals(self, nested_curve_list, line_definition):
        """calculates the normals of frames based on
            cantiliver
        """

        curve_point_normal_list = []
        start_shift = 0

        for i,curve_list in enumerate(nested_curve_list):
            for j,curve in enumerate(curve_list):
                points = self.resample_points_by_count(curve, line_definition)
                point_normal_list = []

                for k,point in enumerate(points):
                    if i < self.layer_height:
                        point_normal = (point, rg.Vector3d(0,0,1))
                        point_normal_list.append(point_normal)

                curve_point_normal_list.append([curve, point_normal_list])

        for i, c in enumerate(self.curve_list[1:]):
            c_height = c.PointAtStart.Z

            for j, p in enumerate(self.point_list[start_shift:]):
                p_height = p.Z

                if SliceCurve.is_almost_equal(c_height, p_height):

                    _, c_double = self.curve_list[i].ClosestPoint(p)
                    c_point = self.curve_list[i].PointAt(c_double)

                    vector01 = rg.Vector3d(c_point)
                    vector02 = rg.Vector3d(p)
                    p_normal = vector02 - vector01
                    p_normal.Unitize()

                    z_vector = rg.Vector3d(0,0,1)
                    angle = self.angle_two_vectors(z_vector, p_normal)

                    if angle < 0.45: p_normal = z_vector

                    perp_vector = rg.Vector3d(0,0,1)
                    p_normal = p_normal + perp_vector
                    p_normal = p_normal + perp_vector
                    normals_list.append(p_normal)
                    start_shift += 1

                elif p_height > c_height: break

        return normals_list
