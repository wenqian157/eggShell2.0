
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections

class GenerateWCS():

    """This class generates WCS frames for UR
    """

    def __init__(self, line_definition, normal_speed,
        fast_speed):

        self.line_definition = line_definition
        self.normal_speed = normal_speed
        self.fast_speed = fast_speed


    def generate_WCS_list(self, curve_list):

        vector_x = rg.Vector3d(1,0,0)
        vector_y = rg.Vector3d(0,1,0)
        WCS_list = []
        point_list = []
        extrude = 1

        for i, curve in enumerate(curve_list):

            if i != len(curve_list)-1:

                dist_start = self.two_d_distance(curve.PointAtStart, curve_list[i+1].PointAtStart)

                if dist_start > self.line_definition:

                    append_point_0 = True

                else:

                    append_point_0 = False

            else:

                append_point_0 = True

            points = self.resample_points_by_count(curve,
                self.line_definition, append_point_0)

            for p in points:

                point_list.append(p)

        for i, p in enumerate(point_list):

            if i==0 or i==len(point_list)-1:

                radius = 0

            else:

                radius, max_length = self.calc_blend_radius(p,
                    point_list[i-1], point_list[i+1])

                if max_length > self.line_definition*2:

                    radius = 0

            temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                vector_y.X, vector_y.Y, vector_y.Z, self.normal_speed, radius, extrude]
            WCS_list.append(temp_list)



        # nested_curve_list = self.reorganize_by_height(curve_list)
        #
        # for i, curve_list in enumerate(nested_curve_list):
        #
        #     if len(curve_list) < 2:
        #
        #         append_point_0 = False
        #
        #     else:
        #
        #         append_point_0 = True
        #
        #     for j, curve in enumerate(curve_list):
        #
        #         points = self.resample_points_by_count(curve, self.line_definition, append_point_0)
        #
        #         for k, p in enumerate(points):
        #
        #             if len(curve_list)>2 and k==0:
        #
        #                 speed = self.fast_speed
        #                 radius = 0
        #                 extrude = 0
        #
        #             elif len(curve_list)>2 and k==len(points)-1:
        #
        #                 speed = self.normal_speed
        #                 radius = 0
        #                 extrude = 0
        #
        #             elif k!=0 and k!=len(points)-1:
        #
        #                 speed = self.normal_speed
        #                 radius = self.calc_blend_radius(p, points[k-1], points[k+1])
        #                 extrude = 1
        #
        #             else:
        #
        #                 speed = self.normal_speed
        #                 radius = 0
        #                 extrude = 1
                    #
                    # temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                    #     vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
                    # point_list.append(p)
                    # WCS_list.append(temp_list)

        return WCS_list, point_list


    def two_d_distance(self, point01, point02):

        dist = math.sqrt((point01.X - point02.X)**2 + (point01.Y - point02.Y)**2)

        return dist


    def calc_blend_radius(self, current_point, prev_point, next_point, dfillet=20, buffer=0.7):

        radius = min((prev_point - current_point).Length/2 * buffer,
            (next_point - current_point).Length/2 * buffer, dfillet)

        max_length = max((prev_point - current_point).Length/2,
            (next_point - current_point).Length/2)

        return radius, max_length


    def reorganize_by_height(self, curve_list):

        temp_list = []
        nested_list = []

        for i, c in enumerate(curve_list[:-2]):

            c_height_01 = c.PointAtStart.Z
            c_height_02 = curve_list[i+1].PointAtStart.Z

            if self.is_almost_equal(c_height_01, c_height_02):

                temp_list.append(c)

            else:

                temp_list.append(c)
                nested_list.append(temp_list)
                temp_list = []

        return nested_list


    def resample_points_by_count(self, curve, line_definition, append_point_0=True):

        """resamples the curve by count of points
        """

        length = curve.GetLength()
        points_per_curve = int(round(length/line_definition))

        divisions = curve.DivideByCount(points_per_curve, True)
        points = [curve.PointAt(d) for d in divisions]

        if append_point_0:

            points.append(points[0])

        return points


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon
