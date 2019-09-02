
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections

"""import SliceCurve
"""
import sliceCurve
reload(sliceCurve)
from sliceCurve import SliceCurve

"""import SlicePoint
"""
import sliceFrame
reload(sliceFrame)
from sliceFrame import SliceFrame

class GenerateWCS():
    """This class generates WCS frames for UR
    """
    def __init__(self, layer_height, line_definition,
        normal_speed, fast_speed):

        self.layer_height = layer_height
        self.line_definition = line_definition
        self.normal_speed = normal_speed
        self.fast_speed = fast_speed


    def generate_WCS_frames(self, slice_curves):
        """generates WCS frames
        """
        tcp_frames = self.construct_tcp_frames(slice_curves)
        self.del_very_close_frames(tcp_frames,1)
        # self.normals_list = self.tcp_normals(nested_curve_list, self.line_definition)
        # self.tcp_frames = self.spiral(self.tcp_frames, self.layer_height)

        WCS_list = []
        # extrude = 1

        for i, f in enumerate(tcp_frames):
            extrude = 1
            speed = int(self.normal_speed-(f.cantiliver*15))
            if f.frame.Origin.Z<0.1: speed-=20
            vector_x = f.frame.XAxis
            vector_y = f.frame.YAxis
            p = f.frame.Origin
            safety_point = False

            if i==0 or i==len(tcp_frames)-1: radius = 0
            else:
                radius, max_length = self.calc_blend_radius(p,
                tcp_frames[i-1].frame.Origin, tcp_frames[i+1].frame.Origin)

                if max_length > self.line_definition:
                    extrude = 0
                    radius = 0
                    safety_point = True
                    safety_point01 = [p.X, p.Y, p.Z+self.line_definition, vector_x.X, vector_x.Y,
                        vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
                        self.normal_speed, 0, 0]
                    safety_point02 = [tcp_frames[i+1].frame.Origin.X, tcp_frames[i+1].frame.Origin.Y,
                        p.Z+10, vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y,
                        vector_y.Z, self.fast_speed, 0, 0]

            temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            # print temp_list
            WCS_list.append(temp_list)

            if safety_point: WCS_list.extend([safety_point01,safety_point02])

        point_list = [rg.Point3d(WCS[0], WCS[1], WCS[2]) for WCS in WCS_list]

        return WCS_list, point_list


    def del_very_close_frames(self, tcp_frames, buffer=0.1):
        """deletes very close points
        """
        # while True:
            # pop_list = False
        # for i,frame in enumerate(tcp_frames):
        #     if (i+1) < len(tcp_frames):
        #         dist = self.two_d_distance([frame.point[0], frame.point[1]],
        #             [tcp_frames[i+1].point[0], tcp_frames[i+1].point[1]])
        #         if dist<buffer:
                    # print dist
                    # pop_list = True
                    # tcp_frames.pop(i+1)
            # if not pop_list:
            #     break

        # return tcp_frames


    def two_d_distance(self, point01, point02):
        """gets 2D distance between two points
        """
        dist = math.sqrt((point01[0] - point02[0])**2 + (point01[1] - point02[1])**2)
        return dist


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


    def construct_tcp_frames(self, slice_curves):
        """constructs tcp frames
        """
        tcp_vector = [-1,0,0]
        base_normal = False
        tcp_frame_list = []

        for i,s in enumerate(slice_curves):
            dist = s.seam.DistanceTo(slice_curves[i-1].seam)
            if dist>self.line_definition: base_normal=[0,0,1]

            for f in s.frames:
                if base_normal: normal=base_normal
                else: normal=f.calc_cantiliver_normal(f.point, slice_curves[i-1].curve)
                # print self.angle_two_vectors(normal,[0,0,1])
                # if (math.pi/4) > abs(self.angle_two_vectors(normal,[0,0,1])) > (math.pi/8):
                #     print self.angle_two_vectors(normal,[0,0,1])
                # else: normal=[0,0,1]

                f.calc_frame(f.point, tcp_vector, [0,0,1])
                f.cantiliver = abs(self.angle_two_vectors(normal,[0,0,1]))
                tcp_frame_list.append(f)

            base_normal = False

        return tcp_frame_list


    def length_of_vector(self, vector):
        """calculates length of vector
        """
        return math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)


    def dot_product(self, vector01, vector02):
        """calculates dot product
        """
        return vector01[0]*vector02[0] + vector01[1]*vector02[1] + vector01[2]*vector02[2]


    def angle_two_vectors(self, vector01, vector02):
        """calculates angle between two vectors
        """
        dot_pro = self.dot_product(vector01, vector02)
        len_01 = self.length_of_vector(vector01)
        len_02 = self.length_of_vector(vector02)
        try:
            angle = math.acos(dot_pro/(len_01*len_02))
        except:
            angle = 0.0

        return angle


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
