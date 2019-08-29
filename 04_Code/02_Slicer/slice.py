
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections
import sliceCurve
reload(sliceCurve)
from sliceCurve import SliceCurve


class Slice():
    """This class slices a mesh
    """

    def __init__(self, layer_height, fix_self_intersections,
        smooth_curves, line_definition):

        self.layer_height = layer_height
        self.line_definition = line_definition
        self.intersection_buffer = 0.04
        self.fix_self_intersections_bool = fix_self_intersections
        self.smooth_curves_bool = smooth_curves


    def contour_mesh(self, mesh):
        """creates curves from a mesh by contouring the mesh
        """

        self.contour_curves = self.create_contour_curves(mesh)
        self.slice_curves = self.create_sliced_curves(self.contour_curves)
        self.slice_curves = self.auto_align_seams(self.slice_curves, 5)
        # self.attractor_points_seams(self.slice_curves, self.attractor_points)
        self.slice_curves = self.shortest_path(self.slice_curves)

        return self.slice_curves
        # return self.contour_curves


    def create_contour_curves(self, mesh):
        """calls the createcontourcurves command from rhino,
        moves it to origin, erases unwanted curves and aligns seams
        """
        min_z, max_z = self.get_min_max_z(mesh)
        start_contour, end_contour = self.define_start_end(15, 250, True)
        slice_curves = []
        contour_curves = rg.Mesh.CreateContourCurves(mesh,
            rg.Point3d(0,0,start_contour), rg.Point3d(0,0,end_contour),
            self.layer_height)
        self.move_to_origin(contour_curves)
        contour_curves = self.fix_errors(contour_curves)

        return contour_curves


    def create_sliced_curves(self, curve_list):
        """creates type slicecurves from curves
        """
        slice_curves = []

        for c in curve_list:
            slice_curve = SliceCurve(c, self.line_definition)
            if slice_curve.area:slice_curves.append(slice_curve)
            else: print None

        return slice_curves


    def shortest_path(self, slice_curves, max_branch_diff=15):
        """finds the shortest path based on max_branch_diff
        """
        buffer = 3
        max_list_len = len(slice_curves)
        shortest_path_list = []
        loop_slice = slice_curves[0]

        while True:

            if len(shortest_path_list) >= max_list_len-1: break
            else:
                index_list, loop_index = [], []
                slice_curves_dup = slice_curves[:]
                count = 0

                for i, slice_curve in enumerate(slice_curves):
                    if count == max_branch_diff: break

                    if i == 0: compare_slice = loop_slice
                    else: compare_slice = shortest_path_list[-1]

                    dist = min(slice_curve.center.DistanceTo(compare_slice.center),
                        slice_curve.seam.DistanceTo(compare_slice.seam))

                    if dist<buffer and abs(slice_curve.area-compare_slice.area)<2000:
                        shortest_path_list.append(slice_curves[i])
                        index_list.append(i)
                        count += 1

                    else: loop_index.append(i)

                if len(loop_index) == 0: loop_slice = shortest_path_list[-1]
                else: loop_slice = slice_curves[loop_index[0]]

                if len(index_list) == 0: shortest_path_list.append(slice_curves[0])

                slice_curves_dup = [i for j, i in enumerate(slice_curves_dup) if j not in index_list]
                slice_curves = slice_curves_dup[:]

        # shortest_path_list.pop(0)
        return shortest_path_list


    def fix_errors(self, curves):
        """fixes errors in curves
        """

        fixed_curves = []
        avg_length = 0

        for c in curves:
            c_length = c.GetLength()
            avg_length += c_length

        avg_length = avg_length/len(curves)

        for c in curves:
            c_length = c.GetLength()

            if c_length > avg_length/10:
                closed = c.MakeClosed(self.layer_height*100)
                c.Domain = rg.Interval(0, 1)
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


    def reorganize_by_height(self, slice_curves):
        """organize curves by height
        """

        temp_list = []
        nested_list = []

        for i, s in enumerate(slice_curves[:-2]):

            c_height_01 = s.seam.Z
            c_height_02 = slice_curves[i+1].seam.Z

            if self.is_almost_equal(c_height_01, c_height_02):

                temp_list.append(s)

            else:

                temp_list.append(s)
                nested_list.append(temp_list)
                temp_list = []

        return nested_list


    def attractor_points_seams(self, slice_curves, attractor_points):
        """align seams based on attractor points
        """

        aligned_slice_curves = []

        for i, c in enumerate(slice_curves):

            min_dist = 99999

            for j, a in enumerate(attractor_points) :

                _, domain = c.curve.ClosestPoint(a)
                v = c.curve.PointAt(domain)
                dist_v = v.DistanceTo(c.curve.PointAtStart)

                if dist_v < min_dist:

                    min_dist = dist_v
                    min_v = domain

            seam_success = c.curve.ChangeClosedCurveSeam(min_v)

        # return slice_curves


    def auto_align_seams(self, slice_curves, buffer=5):
        """aligns all seams of all curves
        """

        aligned_slice_curves = []
        nested_slice_curves = self.reorganize_by_height(slice_curves)

        for i, s_list in enumerate(nested_slice_curves):
            center_point = self.get_center_slice_curves(s_list)

            if i==0:
                _, v = s_list[0].curve.ClosestPoint(center_point)
                v = s_list[0].curve.PointAt(v)
                seaming_points = [v]

            for j, c in enumerate(s_list):
                _, domain = c.curve.ClosestPoint(center_point)
                point = c.curve.PointAt(domain)
                dist_seam_center = self.two_d_distance(seaming_points[-1], point)

                if dist_seam_center > buffer:
                    _, domain = c.curve.ClosestPoint(seaming_points[-1])
                    point = c.curve.PointAt(domain)

                seam_success = c.curve.ChangeClosedCurveSeam(domain)
                c.SCurve = c.curve
                aligned_slice_curves.append(c)

        return aligned_slice_curves


    def get_center_slice_curves(self, slice_curves):
        """gets center of several slicecurve type
        """

        sum_x = 0
        sum_y = 0

        for s in slice_curves:
            sum_x += s.center.X
            sum_y += s.center.Y

        avrg_x = sum_x/len(slice_curves)
        avrg_y = sum_y/len(slice_curves)
        avrg_p = rg.Point3d(avrg_x, avrg_y, s.center.Z)

        return avrg_p


    def two_d_distance(self, point01, point02):
        """gets 2D distance between two points
        """
        dist = math.sqrt((point01.X - point02.X)**2 + (point01.Y - point02.Y)**2)
        return dist


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

        if not trim_top_bottom: trim_value = 0

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
