
import Rhino.Geometry as rg
import math
import sliceMath
reload (sliceMath)

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
        volume = 0
        for s in slice_curves:
            if volume:
                volume += s.area

        print "volume", volume

        tcp_frames = self.construct_tcp_frames(slice_curves)
        # tcp_frames = self.delete_very_close_frames(tcp_frames, 1)
        # self.normals_list = self.tcp_normals(nested_curve_list, self.line_definition)
        # self.tcp_frames = self.spiral(self.tcp_frames, self.layer_height)

        WCS_list = []
        # extrude = 1

        for i, f in enumerate(tcp_frames):
            extrude = 1
            if f.cantiliver:speed = int(self.normal_speed-(f.cantiliver*15))
            else:speed = self.normal_speed
            if f.frame.Origin.Z<0.1: speed=20
            vector_x = f.frame.XAxis
            vector_y = f.frame.YAxis
            p = f.frame.Origin
            safety_point = False

            if i==0 or i==len(tcp_frames)-1: radius = 0
            elif i<len(tcp_frames)-2:
                radius, max_length = self.calc_blend_radius(p,
                tcp_frames[i-1].frame.Origin, tcp_frames[i+1].frame.Origin)

                # if max_length > self.line_definition:
                if f.transition:
                    extrude = 0
                    radius = 0
                    speed = self.fast_speed
                    # safety_point = True
                    # safety_point01 = [p.X, p.Y, p.Z+self.line_definition, vector_x.X, vector_x.Y,
                    #     vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
                    #     self.normal_speed, 0, 0]
                    # safety_point02 = [tcp_frames[i+1].frame.Origin.X, tcp_frames[i+1].frame.Origin.Y,
                    #     p.Z+10, vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y,
                    #     vector_y.Z, self.fast_speed, 0, 0]

            temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            # print temp_list
            WCS_list.append(temp_list)

            # if safety_point: WCS_list.extend([safety_point01,safety_point02])

        point_list = [rg.Point3d(WCS[0], WCS[1], WCS[2]) for WCS in WCS_list]

        return WCS_list, point_list


    def delete_very_close_frames(self, frames, buffer=0.01):
        """deletes very close points
        """
        while True:
            pop_list = False
            for i,f in enumerate(frames):
                if (i+1)<len(frames) and i!=0:
                    dist = sliceMath.two_d_distance(f.frame.Origin,
                        frames[i+1].frame.Origin)
                    if dist<buffer:
                        pop_list = True
                        frames.pop(i)
            if not pop_list:
                break

        return frames


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


    def reorganize_by_height(self, slice_curves):
        """organize curves by height
        """

        temp_list = []
        nested_list = []

        for i, s in enumerate(slice_curves[:-2]):
            c_height_01 = s.seam.Z
            c_height_02 = slice_curves[i+1].seam.Z

            if sliceMath.is_almost_equal(c_height_01, c_height_02):
                temp_list.append(s)

            else:
                temp_list.append(s)
                nested_list.append(temp_list)
                temp_list = []

        return nested_list


    def closest_curve_to_point(self, point, curve_list):

        min_dist = 999999

        for i,c in enumerate(curve_list):
            dist = sliceMath.three_d_distance(point,c.seam)
            if min_dist>dist: min_dist,min_i = dist,i

        return i


    def construct_tcp_frames(self, slice_curves_base):
        """constructs tcp frames
        """

        # nested_slice_curves = self.reorganize_by_height(slice_curves_base)
        # tcp_vector = [-1,0,0]
        tcp_frame_list = []

        len_points = 0
        len_frames = 0

        for s in slice_curves_base:
            len_points+=len(s.points)
            len_frames+=len(s.frames)

        # print len_points, len_frames

        for s in slice_curves_base:
            for f in s.frames:
                # print type(f.frame)
                tcp_frame_list.append(f)

        # for i,slice_curves in enumerate(nested_slice_curves):
        #     for s in slice_curves:
        #         min_i = self.closest_curve_to_point(s.seam, nested_slice_curves[i-1])
        #         closest_curve = nested_slice_curves[i-1][min_i]
        #
        #         for f in s.frames:
        #             normal=f.calc_cantiliver_normal(f.point, closest_curve.curve)
        #             f.calc_frame(f.point, tcp_vector, [0,0,1])
        #             f.cantiliver = abs(sliceMath.angle_two_vectors(normal,[0,0,1]))
        #             if s.transition_curve:
        #                 print len(s.frames)
        #                 f.transition = True
        #             tcp_frame_list.append(f)

        return tcp_frame_list


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

                if sliceMath.is_almost_equal(c_height, p_height):

                    _, c_double = self.curve_list[i].ClosestPoint(p)
                    c_point = self.curve_list[i].PointAt(c_double)

                    vector01 = rg.Vector3d(c_point)
                    vector02 = rg.Vector3d(p)
                    p_normal = vector02 - vector01
                    p_normal.Unitize()

                    z_vector = rg.Vector3d(0,0,1)
                    angle = sliceMath.angle_two_vectors(z_vector, p_normal)

                    if angle < 0.45: p_normal = z_vector

                    perp_vector = rg.Vector3d(0,0,1)
                    p_normal = p_normal + perp_vector
                    p_normal = p_normal + perp_vector
                    normals_list.append(p_normal)
                    start_shift += 1

                elif p_height > c_height: break

        return normals_list
