
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


    def get_center_of_points(self, points):

        sum_x = 0
        sum_y = 0

        for p in points:

            sum_x += p.X
            sum_y += p.Y

        avrg_x = sum_x/len(points)
        avrg_y = sum_y/len(points)

        avrg_p = rg.Point3d(avrg_x, avrg_y, points[0].Z)

        return avrg_p


    def move_to_origin(self, points):

        """moves a list of points to 0,0,0
        """
        avrg_p = self.get_center_of_points(points)
        print avrg_p

        # min_z = point_sample[0].Z

        # self.target = rg.Point3d(0, 0, min_z)

        target_transform = rg.Transform.Translation(-avrg_p.X,
            -avrg_p.Y, -avrg_p.Z)

        # self.target.Transform(target_transform)

        for p in points:

            p.Transform(target_transform)


    def generate_WCS_list(self, curve_list):

        vector_x = rg.Vector3d(1.0,0.0,0.0)
        vector_y = rg.Vector3d(0.0,1.0,0.0)
        WCS_list = []
        point_list = []
        extrude = 1
        speed = self.normal_speed
        append_point_0 = False


        for i, curve in enumerate(curve_list):

            points = self.resample_points_by_count(curve,
                    self.line_definition, append_point_0)

            for j,p in enumerate(points):

                point_list.append(p)

                if j==0 or j==len(points)-1:
                    radius = 0

                else:
                    radius, _ = self.calc_blend_radius(p,
                        points[j-1], points[j+1])

                temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                    vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
                WCS_list.append(temp_list)

        return WCS_list, point_list

            # if i < 151:
            #
            #     speed = self.normal_speed
            #     append_point_0 = False
            #     points = self.resample_points_by_count(curve,
            #         self.line_definition, append_point_0)
            #
            #     for j, p in enumerate(points[:-2]):
            #         point_list.append(p)
            #
            #         if j==0 or j==len(points)-1:
            #             radius = 2
            #
            #         else:
            #             radius, _ = self.calc_blend_radius(p,
            #                 points[j-1], points[j+1])
            #
            #         temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
            #             vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            #         WCS_list.append(temp_list)
            #
            # if 150 < i < 264 or 1164 < i:
            #     append_point_0 = False
            #     points = self.resample_points_by_count(curve,
            #         self.line_definition, append_point_0)
            #
            #     for j, p in enumerate(points[:-2]):
            #         point_list.append(p)
            #
            #         if j==0 or j==len(points)-1:
            #             radius = 2
            #             speed = 20
            #
            #         elif j==1 or j==len(points)-2:
            #             radius, _ = self.calc_blend_radius(p,
            #                 points[j-1], points[j+1])
            #             speed = 30
            #
            #         else:
            #             radius, _ = self.calc_blend_radius(p,
            #                 points[j-1], points[j+1])
            #             speed = self.normal_speed
            #
            #         temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
            #             vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            #         WCS_list.append(temp_list)
            #
            #
            # elif 263 < i < 1165:
            #     append_point_0 = True
            #     points = self.resample_points_by_count(curve,
            #         self.line_definition, append_point_0)
            #
            #     for j, p in enumerate(points):
            #         point_list.append(p)
            #         speed = self.normal_speed
            #
            #         if j==0 or j==len(points)-1:
            #             radius = 0
            #
            #         else:
            #             radius, _ = self.calc_blend_radius(p,
            #                 points[j-1], points[j+1])
            #
            #         temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
            #             vector_y.X, vector_y.Y, vector_y.Z, speed, radius, extrude]
            #         WCS_list.append(temp_list)

        # insert_index = []
        #
        # for i, WCS in enumerate(WCS_list[:-2]):
        #     if WCS[2]-WCS_list[i+1][2] > 1:
        #         insert_index.append(i+1)
        #
        # for i, index in enumerate(insert_index):
        #     speed = self.normal_speed
        #     radius = 0

            # WCS_list.insert(index+i, [WCS_list[index+i][0], WCS_list[index+i][1], WCS_list[index+i-1][2]+1,
            #     vector_x.X, vector_x.Y, vector_x.Z, vector_y.X,
            #     vector_y.Y, vector_y.Z, speed, radius, extrude])
            # point_list.insert(index+i, rg.Point3d(WCS_list[index+i][0], WCS_list[index+i][1], WCS_list[index+i-1][2]+1)

        # return WCS_list, point_list

                # WCS_list.insert(i+1, [WCS_list[i+1][0], WCS_list[i+1][1], WCS[2]+1,
                #     vector_x.X, vector_x.Y, vector_x.Z, vector_y.X,
                #     vector_y.Y, vector_y.Z, speed, radius, extrude])
                # point_list.insert(i+1, rg.Point3d(WCS_list[i+1][0],
                #     WCS_list[i+1][1], WCS[2]+1))




            # if i != len(curve_list)-1:
            #
            #     dist_start = self.two_d_distance(curve.PointAtStart, curve_list[i+1].PointAtStart)
            #
            #     if dist_start > self.line_definition/10:
            #
            #         append_point_0 = True
            #
            #     else:
            #
            #         append_point_0 = True
            #
            # else:
            #
            #     append_point_0 = True
            #
            # points = self.resample_points_by_count(curve,
            #     self.line_definition, append_point_0)

            # for p in points:
            #
            #     if i==0 or i==len(point_list)-1:
            #
            #         radius = 0
            #
            #     else:
            #         radius, max_length = self.calc_blend_radius(p,
            #             point_list[i-1], point_list[i+1])
            #
            #         if max_length > self.line_definition*2:
            #             radius = 0
            #
            #         dist = p.DistanceTo(point_list[i+1])
            #
            #         if dist > self.line_definition*2:
            #             safety_point = [point_list[i+1].X, point_list[i+1].Y, p.Z+1,
            #                 vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
            #                 self.normal_speed, 0, 0]
            #
            #     point_list.append(p)

        # for i, p in enumerate(point_list):
        #
        #     if i != len(point_list)-1:
        #
        #         dist = p.DistanceTo(point_list[i+1])

        # safety_point = False
        #
        # for i, p in enumerate(point_list):
        #
        #     safety_point = False
        #
        #     if i==0 or i==len(point_list)-1:
        #         radius = 0
        #
        #     else:
        #         radius, max_length = self.calc_blend_radius(p,
        #             point_list[i-1], point_list[i+1])
        #
        #         if max_length > self.line_definition*2:
        #             radius = 0
        #
        #         dist = p.DistanceTo(point_list[i+1])
        #
        #         if dist > self.line_definition*2:
        #             safety_point = [point_list[i+1].X, point_list[i+1].Y, p.Z+1,
        #                 vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
        #                 self.normal_speed, 0, 0]
        #
        #     temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
        #         vector_y.X, vector_y.Y, vector_y.Z, self.normal_speed, radius, extrude]
        #     WCS_list.append(temp_list)
        #
        #     if safety_point:
        #         WCS_list.append(safety_point)


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

        # point_list = [rg.Point3d(WCS[0], WCS[1], WCS[2]) for WCS in WCS_list]


    def generate_WCS_list_points(self, point_list):

        vector_x = rg.Vector3d(1.0,0.0,0.0)
        vector_y = rg.Vector3d(0.0,1.0,0.0)
        WCS_list = []
        extrude = 1

        for i, p in enumerate(point_list):

            if i != len(point_list)-1:

                print type(p)

                dist = p.DistanceTo(point_list[i+1])

        safety_point = False

        for i, p in enumerate(point_list):

            safety_point = False

            if i==0 or i==len(point_list)-1:
                radius = 0

            else:
                radius, max_length = self.calc_blend_radius(p,
                    point_list[i-1], point_list[i+1])

                if max_length > self.line_definition*2:
                    radius = 0

                dist = p.Z - point_list[i+1].Z

                if dist > 2:
                    safety_point = [point_list[i+1].X, point_list[i+1].Y, p.Z+1,
                        vector_x.X, vector_x.Y, vector_x.Z, vector_y.X, vector_y.Y, vector_y.Z,
                        self.normal_speed, 0, 0]

            temp_list = [p.X, p.Y, p.Z, vector_x.X, vector_x.Y, vector_x.Z,
                vector_y.X, vector_y.Y, vector_y.Z, self.normal_speed, radius, extrude]
            WCS_list.append(temp_list)

            if safety_point:
                print safety_point
                WCS_list.append(safety_point)

        point_list = [rg.Point3d(WCS[0], WCS[1], WCS[2]) for WCS in WCS_list]

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

        # print type(curve)

        divisions = curve.DivideByCount(points_per_curve, True)
        # print divisions

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
