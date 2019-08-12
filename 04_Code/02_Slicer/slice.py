
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
        self.line_definition = layer_height*5
        self.intersection_buffer = 0.04
        self.fix_self_intersections_bool = fix_self_intersections
        self.smooth_curves_bool = smooth_curves


    def contour_mesh(self, mesh):

        """creates curves from a mesh by contouring the mesh
        """

        self.min_z, self.max_z = self.get_min_max_z(mesh)

        self.start_contour, self.end_contour = self.define_start_end(120, 180, True)

        self.contour_curves = rg.Mesh.CreateContourCurves(mesh,
            rg.Point3d(0,0,self.start_contour),
            rg.Point3d(0,0,self.end_contour), self.layer_height)

        self.seaming_points = []

        self.move_to_origin(self.contour_curves)

        self.contour_curves = self.fix_errors(self.contour_curves)

        # self.contour_curves, self.seaming_points, self.nested_c_list = self.auto_align_seams(self.contour_curves)

        slice_curves = self.auto_align_seams(self.contour_curves)

        # self.contour_curves = self.shortest_path(self.contour_curves, self.nested_c_list)

        # self.contour_curves = self.shortest_path_2(self.contour_curves, self.nested_c_list, self.seaming_points)

        self.seaming_points = [slice_curve[2] for slice_curve in slice_curves]

        self.contour_curves = self.shortest_path_3(slice_curves)

        # return self.contour_curves, self.seaming_points, self.nested_c_list
        return self.contour_curves, self.seaming_points


    def shortest_path_3(self, slice_curves, max_branch_diff=15):

        buffer = 3
        max_list_len = len(slice_curves)
        shortest_path_list = []
        loop_slice = slice_curves[0]

        while True:

            if len(shortest_path_list) >= max_list_len-1:
                break

            else:
                index_list = []
                loop_index = []
                slice_curves_dup = slice_curves[:]
                count = 0

                for i, slice_curve in enumerate(slice_curves):

                    if count == max_branch_diff:
                        break

                    if i == 0:
                        compare_slice = loop_slice

                    else:
                        compare_slice = shortest_path_list[-1]

                    curve,z_height,seam,center,area = slice_curve
                    _,_,loop_s,loop_c,loop_a = compare_slice

                    dist = min(center.DistanceTo(loop_c),
                        seam.DistanceTo(loop_s))

                    if dist<buffer and abs(area-loop_a)<2000:
                        # print area
                        shortest_path_list.append(slice_curves[i])
                        index_list.append(i)
                        count += 1

                    else:
                        loop_index.append(i)

                if len(loop_index) == 0:
                    loop_slice = shortest_path_list[-1]

                else:
                    loop_slice = slice_curves[loop_index[0]]

                if len(index_list) == 0:
                    shortest_path_list.append(slice_curves[0])
                    print slice_curves[0][1]

                slice_curves_dup = [i for j, i in enumerate(slice_curves_dup) if j not in index_list]
                slice_curves = slice_curves_dup[:]

        shortest_path_list.pop(0)
        contour_curves = [slice_curve[0] for slice_curve in shortest_path_list]

        return contour_curves


    def shortest_path_2(self, curves, nested_c_list, seams, max_branch_diff=15):

        buffer = 15
        shortest_path_list = []
        loop_point = seams[0]

        while True:

            if len(shortest_path_list) > 100:

                break

            else:

                index_list = []
                loop_index = []
                curves_duplicate = curves[:]
                seams_duplicate = seams[:]
                count = 0

                for i, seam in enumerate(seams):

                    if count == max_branch_diff:

                        break

                    elif self.two_d_distance(seam, loop_point) < buffer:

                        shortest_path_list.append(curves[i])
                        index_list.append(i)
                        count += 1

                    else:

                        loop_index.append(i)

                if len(loop_index) == 0:

                    loop_point = shortest_path_list[-1].PointAtStart
                    # print loop_point.Z

                else:

                    loop_point = seams[loop_index[0]]

                curves_duplicate = [i for j, i in enumerate(curves_duplicate) if j not in index_list]
                seams_duplicate = [i for j, i in enumerate(seams_duplicate) if j not in index_list]

                curves = curves_duplicate[:]
                seams = seams_duplicate[:]

        return shortest_path_list


    def shortest_path(self, curves, nested_curve_list, max_branch_diff=15):

        shortest_path_list = []
        loop_start = 0
        loop_end = max_branch_diff
        max_branch_count = 0

        for curves in nested_curve_list:

            num_curves = len(curves)

            if max_branch_count < num_curves:

                max_branch_count = num_curves

        while True:

            if loop_end >= len(nested_curve_list)*self.layer_height:

                break

            for loop in range(max_branch_count):

                for i, curves in enumerate(nested_curve_list[loop_start:loop_end]):

                    if len(curves) > loop:

                        shortest_path_list.append(curves[loop])
                        # print curves[0].PointAtStart.Z

            loop_start += max_branch_diff
            loop_end += max_branch_diff

        return shortest_path_list


    def fix_errors(self, curves):

        fixed_curves = []

        avg_length = 0

        for c in curves:

            c_length = c.GetLength()

            avg_length += c_length

        avg_length = avg_length/len(curves)

        for c in curves:

            c_length = c.GetLength()

            if c_length > avg_length/10:

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
                curve_dict = {}

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


    def generate_slice_curve(self, curve, z_height, seam):

        center = self.get_center_of_curves([curve])
        area = rg.AreaMassProperties.Compute(curve)
        area = area.Area
        slice_curve = [curve, z_height, seam, center, area]

        return slice_curve


    def auto_align_seams(self, curve_list, buffer=5):

        """aligns all seams of all curves
        """

        nested_curve_list = self.reorganize_by_height(curve_list)

        # seaming_points = []
        # aligned_curves = []
        slice_curves = []


        for i, curve_list in enumerate(nested_curve_list):

            center_point = self.get_center_of_curves(curve_list)

            if i==0:

                _, v = curve_list[0].ClosestPoint(center_point)
                v = curve_list[0].PointAt(v)
                seaming_points = [v]

            for j, c in enumerate(curve_list):

                closed = c.MakeClosed(self.layer_height*10)
                c.Domain = rg.Interval(0, 1)

                _, domain = c.ClosestPoint(center_point)
                point = c.PointAt(domain)

                dist_seam_center = self.two_d_distance(seaming_points[-1], point)

                if dist_seam_center > buffer:

                    _, domain = c.ClosestPoint(seaming_points[-1])
                    point = c.PointAt(domain)

                seam_success = c.ChangeClosedCurveSeam(domain)
                slice_curve = self.generate_slice_curve(c, c.PointAtStart.Z, point)
                slice_curves.append(slice_curve)
                seaming_points.append(point)
                # aligned_curves.append(c)

        # seaming_points.pop(0)
        # nested_curve_list = self.arrange_curves(nested_curve_list)

        # return aligned_curves, seaming_points, nested_curve_list
        return slice_curves


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


    def match_curve_direction(self, curves):

        """matches all curves directions
        """

        for i in range(0, len(curves)-1):

            vector01 = rg.Vector3d(curves[i].PointAt(0)) - rg.Vector3d(curves[i].PointAt(.001))
            vector02 = rg.Vector3d(curves[i+1].PointAt(0)) - rg.Vector3d(curves[i+1].PointAt(.001))

            vector_angle = rg.Vector3d.VectorAngle(vector01, vector02)

            if vector_angle > math.pi/4:

                curves[i+1].Reverse()


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon


    def define_start_end(self,  min_z, max_z, trim_top_bottom):

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
            -self.target.Y, -self.target.Z+0.001)

        self.target.Transform(target_transform)

        for n in curves:

            n.Transform(target_transform)
