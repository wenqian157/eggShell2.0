
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections


class Slice():

    """This class slices a mesh
    """

    def __init__(self, layer_height,
        fix_self_intersections, smooth_curves):

        self.layer_height = layer_height
        self.line_definition = layer_height*10
        self.intersection_buffer = 0.04
        self.fix_self_intersections_bool = fix_self_intersections
        self.smooth_curves_bool = smooth_curves


    def contour_mesh(self, mesh):

        """creates curves from a mesh by contouring the mesh
        """

        self.min_z, self.max_z = self.get_min_max_z(mesh)

        self.start_contour, self.end_contour = self.define_start_end(self.max_z, self.min_z, True)

        self.contour_curves = rg.Mesh.CreateContourCurves(mesh,
            rg.Point3d(0,0,self.start_contour),
            rg.Point3d(0,0,self.end_contour), self.layer_height)

        self.move_to_origin(self.contour_curves)

        self.contour_curves = self.fix_errors(self.contour_curves)

        if self.fix_self_intersections_bool:

            self.contour_curves = self.fix_self_intersections(self.contour_curves, self.intersection_buffer, self.layer_height)

        elif self.smooth_curves_bool:

            self.contour_curves = self.smooth_curves(self.contour_curves, self.layer_height, 0.6)

        self.contour_curves, self.seaming_points, self.nested_c_list = self.auto_align_seams(self.contour_curves)

        self.contour_curves = self.shortest_path(self.contour_curves, self.nested_c_list)

        # self.resampled_points = self.resample_points_by_count(self.contour_curves, self.line_definition)

        # self.resampled_points = self.del_very_close_points(self.resampled_points, self.layer_height)

        return self.contour_curves, self.seaming_points


    def shortest_path(self, curves, nested_curve_list, max_branch_diff=25):

        shortest_path_list = []
        loop_start = 0
        loop_end = max_branch_diff
        max_branch_count = 0

        for curves in nested_curve_list:

            num_curves = len(curves)

            if max_branch_count<num_curves:

                # max_branch_count=num_curves
                max_branch_count = 1

        # print max_branch_count

        while True:

            # if self.contour_curves == len(shortest_path_list):
            #
            #     break

            if loop_end >= len(nested_curve_list)*self.layer_height:

                break

            for loop in range(max_branch_count):

                for i, curves in enumerate(nested_curve_list[loop_start:loop_end]):

                    if len(curves) > loop:

                        # print loop
                        # print len(curves)
                        # print loop+i+loop_start
                        # print curves[loop].PointAtStart.Z
                        shortest_path_list.append(curves[loop])

            loop_start += max_branch_diff
            loop_end += max_branch_diff
            # print loop_end
            # print len(nested_curve_list)

        # print len(shortest_path_list)
        # print len(self.contour_curves)
        return shortest_path_list

        # for branch in range(max_branch_count):
        #
        #     if len(remaining_curves)
        #
        #     for i, curves in enumerate(nested_curve_list[loop_start:loop_end]):
        #
        #         num_curves = len(curves)
        #
        #         temp_list = []
        #
        #         for j, curve in enumerate(curves):
        #
        #             if j == 0:
        #
        #                 shortest_path_list.append(curve)
        #
        #             else:
        #
        #                 temp_list.append(curve)
        #
        #         remaining_curves.append(temp_list)
        #
        #     if i < max_branch_count-1:
        #
        #         break
        #
        #     if loop_end > len(nested_curve_list)*self.layer_height:
        #
        #         break
        #
        #     for i, curves in enumerate(nested_curve_list[loop_start:loop_end]):
        #
        #         num_curves = len(curves)
        #
        #         for j, curve in enumerate(curves):
        #
        #             shortest_path_list.append(curve)
        #             curves.pop(j)
        #             break
        #
        #     loop_start += max_branch_diff
        #     loop_end += max_branch_diff
        #
        # return shortest_path_list

            # if len(shortest_path_list) == len_curve_list:
            #
            #     return shortest_path_list
            #     break
            #
            # count = 0
            #
            # for i, c in enumerate(curves[1:]):
            #
            #     if count > max_branch_diff:
            #
            #         print "break"
            #         break
            #
            #     else:
            #
            #         dist_start_pt = self.two_d_distance(shortest_path_list[-1].PointAtStart, c.PointAtStart)
            #
            #         if dist_start_pt < layer_height*2:
            #
            #             shortest_path_list.append(c)
            #             count += 1
            #
            #         else:
            #
            #             remaining_curves.append(c)
            #
            # curves = remaining_curves



            # elif len(remaining_curves) > 0:
            #
            #     shortest_path_list.append(remaining_curves[0])
            #
            # remaining_curves = []
            # count = 0
            #
            # print "else"
            #
            # for i, c in enumerate(curves[1:]):
            #
            #     if count > max_branch_diff:
            #
            #         print "break"
            #         break
            #
            #     else:
            #
            #         dist_start_pt = self.two_d_distance(shortest_path_list[-1].PointAtStart, c.PointAtStart)
            #
            #         if dist_start_pt < layer_height*2:
            #
            #             shortest_path_list.append(c)
            #             current_curves.pop(i)
            #             count += 1
            #
            #         else:
            #
            #             remaining_curves.append(c)


    def fix_errors(self, curves):

        fixed_curves = []

        avg_length = 0

        for c in curves:

            c_length = c.GetLength()

            avg_length += c_length

        avg_length = avg_length/len(curves)

        for c in curves:

            c_length = c.GetLength()

            if c_length > avg_length/3:

                fixed_curves.append(c)

        return fixed_curves


    def get_min_max_z(self, mesh):

        """get bounding bounding
        """
        bounding_box = mesh.GetBoundingBox(True)
        corners = bounding_box.GetCorners()

        avgx = 0
        avgy = 0

        min_z = 99999
        max_z = 0

        for p in corners:

            if p.Z < min_z:

                min_z = p.Z

            elif p.Z > max_z:

                max_z = p.Z

        return min_z, max_z


    def del_very_close_points(self, points, layer_height):

        """deletes very close points that could cause an abnormal robot behaviour
        """

        for i in range(0, len(points) - 1):

            if (i+1) < len(points):

                dist_bool = False

                x01 = points[i].X
                x02 = points[i + 1].X

                y01 = points[i].Y
                y02 = points[i + 1].Y

                dist = math.sqrt((x01-x02)**2 + (y01-y02)**2)

                if abs(dist) <= layer_height/2:

                    print "pop"

                    points.pop(i + 1)

        return points


    # def resample_points_by_count(self, curves, line_definition):
    #
    #     """resamples the curve by count of points
    #     """
    #
    #     resampled_points = []
    #
    #     for c in curves:
    #
    #         temp_list = []
    #
    #         length = c.GetLength()
    #         points_per_curve = int(round(length/line_definition))
    #
    #         divisions = c.DivideByCount(points_per_curve, True)
    #         points = [c.PointAt(d) for d in divisions]
    #
    #         points.append(points[0])
    #
    #         for p in points:
    #
    #             temp_list.append(p)
    #
    #         resampled_points.append(temp_list)
    #
    #     return resampled_points


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


    def arrange_curves(self, nested_list):

        fixed_list = []

        for i, list in enumerate(nested_list):

            if len(list) > 1 and i != 0:

                min_length = 9999999999

                for j, curve in enumerate(list):

                    dist = curve.PointAtStart.DistanceTo(fixed_list[i-1][0].PointAtStart)

                    if dist < min_length:

                        min_length = dist
                        min_index = j

                list = list[min_index:]+list[:min_index]
                fixed_list.append(list)

            else:

                fixed_list.append(list)

        return fixed_list


    def auto_align_seams(self, curve_list):

        """aligns all seams of all curves
        """

        nested_curve_list = self.reorganize_by_height(curve_list)

        seaming_points = []
        aligned_curves = []

        for i, curve_list in enumerate(nested_curve_list):

            center_point = self.get_center_of_curves(curve_list)

            for j, c in enumerate(curve_list):

                c.Domain = rg.Interval(0, 1)

                if len(curve_list) == 1 and len(seaming_points) > 0:

                    closest_seam_point = seaming_points[0]
                    _, v = c.ClosestPoint(closest_seam_point)

                else:

                    _, v = c.ClosestPoint(center_point)

                c.ChangeClosedCurveSeam(v)
                seaming_points.append(c.PointAt(v))
                aligned_curves.append(c)

        nested_curve_list = self.arrange_curves(nested_curve_list)

        return aligned_curves, seaming_points, nested_curve_list


    def two_d_distance(self, point01, point02):

        dist = math.sqrt((point01.X - point02.X)**2 + (point01.Y - point02.Y)**2)

        return dist


    def get_center_of_curves(self, curve_list):

        sum_x = 0
        sum_y = 0
        point_count = 0

        for curve in curve_list:

            divisions = curve.DivideByCount(20, True)
            point_list = [curve.PointAt(d) for d in divisions]

            for p in point_list:

                sum_x += p.X
                sum_y += p.Y

                point_count += 1

        avrg_x = sum_x/point_count
        avrg_y = sum_y/point_count

        avrg_p = rg.Point3d(avrg_x, avrg_y, curve_list[0].PointAtStart.Z)

        return avrg_p


    def resample_points_by_length(self, curves, length):

        """resamples the curve by count of points
        """

        resampled_points = []

        for c in curves:

            divisions = c.DivideByLength(length, True)
            points = [c.PointAt(d) for d in divisions]

            for p in points:

                resampled_points.append(p)

            resampled_points.append(points[0])

        return resampled_points


    def match_curve_direction(self, curves):

        """matches all curves directions
        """

        for i in range(0, len(curves)-1):

            vector01 = rg.Vector3d(curves[i].PointAt(0)) - rg.Vector3d(curves[i].PointAt(.001))
            vector02 = rg.Vector3d(curves[i+1].PointAt(0)) - rg.Vector3d(curves[i+1].PointAt(.001))

            vector_angle = rg.Vector3d.VectorAngle(vector01, vector02)

            if vector_angle > math.pi/4:

                curves[i+1].Reverse()


    def regenerate_curves(self, curves, regenerate_index):

        """regenerates missing curves caused by inconsistencies in the mesh
        """

        regenerate_dict = self.get_regenerate_dict(regenerate_index)

        # print regenerate_dict

        regenerated_curves = curves

        step = 0

        for i, index in enumerate(regenerate_dict):

            if i == step:

                mean_dict = {}

                tween_curves = rg.Curve.CreateTweenCurvesWithMatching(curves[index-1],
                    curves[index], regenerate_dict[index]+1)

                for t in tween_curves[::-1]:

                    regenerated_curves.insert(index, t)

                step += (regenerate_dict[index]+1)

        return regenerated_curves


    def get_regenerate_dict(self, regenerate_index):

        """regenerates the curve dictionary
        """

        regenerate_dict = {}

        factor = 0

        for i, index in enumerate(regenerate_index[::-1]):

            try:

                if abs(index - regenerate_index[-i]) == 1:

                    factor += 1

                else:

                    factor = 0

            except:

                factor = 0

            regenerate_dict[index] = factor

        sorted_regenerate_dict = collections.OrderedDict(sorted(regenerate_dict.items()))

        return sorted_regenerate_dict


    def reconstruct_curves(self, curves, layer_height):

        """reconstructs curves on the same height to form closed curves
        """

        regenerate_index = []

        reconst_curves = []

        nested_c_list = self.reorganize_by_height(curves)

        for i, c_list in enumerate(nested_c_list):

            if len(c_list) == 1:

                for c in c_list:

                    closed = c.MakeClosed(layer_height*2)
                    reconst_curves.append(c)

            else:

                reconst_c = rg.Curve.JoinCurves(c_list, layer_height*2)

                if len(reconst_c) == 1:

                    reconst_curves.append(reconst_c[0])

                elif len(reconst_c) == len(c_list):

                    for c in c_list:

                        try:

                            closed = c.MakeClosed(layer_height*2)
                            reconst_curves.append(c)

                        except:

                            continue

                else:

                    regenerate_index.append(i)

        return regenerate_index, reconst_curves


    # def reorganize_by_height(self, curves, layer_height):
    #
    #     """reorganizes a list of curves into lists with the same height
    #     """
    #
    #     step = 0
    #
    #     nested_c_list = []
    #
    #     for h in range(int(curves[-1].PointAtStart.Z/layer_height)+1):
    #
    #         c_list = []
    #
    #         for i, c in enumerate(curves[step:]):
    #
    #             z_value = c.PointAtStart.Z
    #
    #             height_bool = self.is_almost_equal(h * layer_height, z_value)
    #
    #             if height_bool:
    #
    #                 c_list.append(c)
    #
    #             else:
    #
    #                 step += len(c_list)
    #                 break
    #
    #         nested_c_list.append(c_list)

        # fixed_list = []
        #
        # for i, curves in enumerate(nested_c_list[1:]):
        #
        #     if len(curves) > 1 and len(nested_c_list[i-1]) > 1:
        #
        #         dist_curves = curves[0].PointAtStart.DistanceTo(nested_c_list[i-1][0].PointAtStart)
        #
        #         if dist_curves > self.layer_height:
        #
        #             curves = curves[1:]+curves[0:1]
        #             fixed_list.append(curves)
        #
        #     else:
        #
        #             fixed_list.append(curves)
        #
        #     # for curve in curves:

        return nested_c_list


    def reorganize_by_length(self, curves, layer_height):

        """reorganizes a list of curves into dictionaries with lengths as keys
        """

        step = 0

        nested_c_list = []

        for h in range(int(curves[-1].PointAtStart.Z/layer_height)+1):

            c_dict = {}

            for i, c in enumerate(curves[step:]):

                z_value = c.PointAtStart.Z

                height_bool = self.is_almost_equal(h * layer_height, z_value)

                if height_bool:

                    c_length = c.GetLength()
                    c_dict[c_length] = c

                else:

                    step += len(c_dict)
                    break

            nested_c_list.append(c_dict)

        return nested_c_list


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon


    def define_start_end(self, max_z, min_z, trim_top_bottom):

        """defines de start and end of the mesh slicing in the z axis
        """

        trim_value = max_z/200

        if not trim_top_bottom:

            trim_value = 0

        start_contour = trim_value + min_z
        end_contour = max_z - trim_value

        return start_contour, end_contour


    def move_to_origin(self, curves):

        """moves a list of points to 0,0,0
        """
        point_sample = rs.CurvePoints(curves[0])

        min_z = point_sample[0].Z

        self.target = rg.Point3d(0, 0, min_z)

        target_transform = rg.Transform.Translation(-self.target.X,
            -self.target.Y, -self.target.Z)

        self.target.Transform(target_transform)

        for n in curves:

            n.Transform(target_transform)


    def del_doubles(self, curves, layer_height):

        single_curves = []

        nested_c_list = self.reorganize_by_length(curves, layer_height)

        for c_dict in nested_c_list:

            try:

                sorted_c_dict = collections.OrderedDict(sorted(c_dict.items()))

                c = sorted_c_dict[sorted_c_dict.keys()[-1]]

                c.MakeClosed(layer_height*2)

                single_curves.append(c)

            except:

                continue

        return single_curves


    def smooth_curves(self, curves, layer_height, smoothing_factor):

        """ smoothens a curve
        """
        new_curves = []

        success_bool = False

        for c in curves:

            if c:

                try:

                    c = c.Smooth(smoothing_factor, True, True, False, False, 0)
                    c.MakeClosed(layer_height*2)
                    new_curves.append(c)
                    success_bool = True

                except:

                    success_bool = False
                    break

        if success_bool:

            return new_curves

        else:

            return curves


    def fix_self_intersections(self, curves, trim_threshold, layer_height):

        """ fixes self intersetions in curves
        """
        fixed_curves = []

        for c in curves:

            int_events = rg.Intersect.Intersection.CurveSelf(c, 0.001)

            c_trimed = c.Duplicate()

            if int_events:

                while True:

                    c_trimed, int_bool = self.trim_curve(int_events, c_trimed, trim_threshold, layer_height)

                    if int_bool:

                        int_events = rg.Intersect.Intersection.CurveSelf(c_trimed, 0.001)

                    else:

                        break

                fixed_curves.append(c_trimed)

            else:

                fixed_curves.append(c_trimed)

        return fixed_curves


    def trim_curve(self, int_events, c_trimed, trim_threshold, layer_height):

        """trims a curve based on self intersections
        """

        int_bool = False

        for int in int_events:

            parA = int.ParameterA
            # print parA
            parB = int.ParameterB
            # print parB

            if abs(parA - parB) < trim_threshold:

                int_bool = True

                try:

                    c_trimed = c_trimed.Trim(parB, parA)
                    c_trimed.MakeClosed(layer_height*2)
                    break

                except:

                    self.trim_error = True
                    print "trim error"
                    break

        return c_trimed, int_bool
