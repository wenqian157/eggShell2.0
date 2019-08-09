
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

        self.start_contour, self.end_contour = self.define_start_end(365, 410, True)

        self.contour_curves = rg.Mesh.CreateContourCurves(mesh,
            rg.Point3d(0,0,self.start_contour),
            rg.Point3d(0,0,self.end_contour), self.layer_height)

        self.seaming_points = []

        self.move_to_origin(self.contour_curves)

        self.contour_curves = self.fix_errors(self.contour_curves)

        self.contour_curves, self.seaming_points, self.nested_c_list = self.auto_align_seams(self.contour_curves)

        self.contour_curves = self.shortest_path(self.contour_curves, self.nested_c_list)

        return self.contour_curves, self.seaming_points


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


    def auto_align_seams(self, curve_list):

        """aligns all seams of all curves
        """

        nested_curve_list = self.reorganize_by_height(curve_list)

        seaming_points = []
        aligned_curves = []

        for curves in nested_curve_list:
            for c in curves:
                aligned_curves.append(c)

        for i, curve_list in enumerate(nested_curve_list):

            center_point = self.get_center_of_curves(curve_list)

            for j, c in enumerate(curve_list):

                closed = c.MakeClosed(self.layer_height*10)
                c.Domain = rg.Interval(0, 1)

                if len(curve_list) == 1 and len(seaming_points) > 0:

                    closest_seam_point = seaming_points[0]
                    _, v = c.ClosestPoint(closest_seam_point)

                else:

                    _, v = c.ClosestPoint(center_point)

                seam_success = c.ChangeClosedCurveSeam(v)
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
            -self.target.Y, -self.target.Z)

        self.target.Transform(target_transform)

        for n in curves:

            n.Transform(target_transform)
