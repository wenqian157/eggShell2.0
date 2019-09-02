
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
        self.slice_curves, branch_points, branch_curves = self.auto_align_seams(self.slice_curves, 0.005)
        self.slice_curves = self.shortest_path(self.slice_curves, 5)

        return self.slice_curves, branch_points, branch_curves
        # return self.contour_curves


    def create_contour_curves(self, mesh):
        """calls the createcontourcurves command from rhino,
        moves it to origin, erases unwanted curves and aligns seams
        """
        min_z, max_z = self.get_min_max_z(mesh)
        start_contour, end_contour = self.define_start_end(min_z+10, max_z-10, True)
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

        return slice_curves


    def group_curves_seam(self, slice_curves, buffer=20):
        """groups a list of curves into sub lists
        based on distance from nested_seam_list
        """
        pair = False
        for i,s01 in enumerate(slice_curves):
            for j,s02 in enumerate(slice_curves):
                if i!=j:
                    dist = self.two_d_distance(s01.seam,s02.seam)
                    if dist<buffer:
                        pair=(i,j)
                        return pair

        return pair


    def join_close_curves(self, slice_curves, buffer=15):
        """joins curves that come close
        """
        nested_slice_curves = self.reorganize_by_height(slice_curves)
        joined_curves = []

        for s_list in nested_slice_curves:
            s_list_dup = s_list[:]
            for i in range(3):
                if i==0: pair=self.group_curves_seam(s_list_dup, buffer)
                else: pair=self.group_curves_seam(s_list_dup, buffer)
                if pair:
                    points = s_list_dup[pair[0]].points + s_list_dup[pair[1]].points + [s_list_dup[pair[1]].points[0]]
                    # points = s_list_dup[pair[0]].points + s_list_dup[pair[1]].points
                    new_curve = rg.PolylineCurve(points)
                    # new_curve.MakeClosed(99999)
                    new_slice_curve = SliceCurve(new_curve, self.line_definition)
                    s_list_dup.pop(pair[0])
                    s_list_dup.pop(pair[1]-1)
                    s_list_dup.insert(pair[0],new_slice_curve)
                else:
                    break
            for s in s_list_dup:
                joined_curves.append(s)

        return joined_curves



    def shortest_path(self, slice_curves, max_branch_diff=10):
        """finds the shortest path based on max_branch_diff
        """
        slice_curves = self.join_close_curves(slice_curves, 15)

        buffer = 2.5
        max_list_len = len(slice_curves)
        shortest_path_list = []
        loop_slice = slice_curves[0]

        while True:

            if len(shortest_path_list) >= max_list_len-1: break

            index_list, loop_index = [], []
            slice_curves_dup = slice_curves[:]
            count = 0

            for i, slice_curve in enumerate(slice_curves):
                if count == max_branch_diff: break

                if i == 0: compare_slice = loop_slice
                else: compare_slice = shortest_path_list[-1]

                dist = min(slice_curve.center.DistanceTo(compare_slice.center),
                    slice_curve.seam.DistanceTo(compare_slice.seam))

                if dist<buffer and abs(slice_curve.area-compare_slice.area)<2000 and abs(slice_curve.seam.Z-compare_slice.seam.Z)<self.layer_height*1.5:
                # if dist<buffer:
                    shortest_path_list.append(slice_curves[i])
                    index_list.append(i)
                    count += 1

                else: loop_index.append(i)

            if len(loop_index) == 0: loop_slice = shortest_path_list[-1]
            else: loop_slice = slice_curves[loop_index[0]]

            if len(index_list) == 0: shortest_path_list.append(slice_curves[0])

            slice_curves_dup = [i for j, i in enumerate(slice_curves_dup) if j not in index_list]
            slice_curves = slice_curves_dup[:]

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
            if p.Z < min_z:min_z = p.Z
            elif p.Z > max_z:max_z = p.Z

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


    def shift_seam(self, slice_curve, target_point, shift_ratio=0.01):
        """shifts seam of curve towards desired point
        """
        length = slice_curve.curve.GetLength()
        # shift = length*shift_ratio
        shift = shift_ratio
        seam_bool=True
        dist = slice_curve.seam.DistanceTo(target_point)
        point01 = slice_curve.curve.PointAt(shift)
        dist01 = point01.DistanceTo(target_point)
        point02 = slice_curve.curve.PointAt(-shift)
        dist02 = point02.DistanceTo(target_point)

        if dist01<dist: seam_domain,seam_point = shift,point01
        elif dist02<dist: seam_domain,seam_point = -shift,point02
        else: seam_bool=False

        if seam_bool:
            seam_success = slice_curve.curve.ChangeClosedCurveSeam(seam_domain)
            set_domain = rg.Interval(0,1)
            slice_curve.curve.Domain = set_domain
            slice_curve.SCurve = slice_curve.curve
            return seam_success,seam_point
        else: return False,False


    def change_seam(self, slice_curve, target_point):
        """changes seam of curve closest to
        desired point
        """
        _, domain = slice_curve.curve.ClosestPoint(target_point)
        point = slice_curve.curve.PointAt(domain)
        seam_success = slice_curve.curve.ChangeClosedCurveSeam(domain)
        set_domain = rg.Interval(0,1)
        slice_curve.curve.Domain = set_domain
        slice_curve.SCurve = slice_curve.curve

        return seam_success, point


    def closest_point_from_curve(self, curve, point_list):
        """finds closest point from curve
        """
        min_dist = 99999

        for p in point_list:
            _, domain = curve.ClosestPoint(p)
            point = curve.PointAt(domain)
            dist = self.two_d_distance(p, point)
            if min_dist>dist: min_dist,min_p=dist,p

        return min_p

    def closest_point_from_points(self, point, point_list):
        """finds closest point from list of point
        """
        min_dist = 99999

        for i,p in enumerate(point_list):
            dist = self.two_d_distance(p, point)
            if min_dist>dist>1: min_dist,min_p,min_i=dist,p,i

        return min_p,min_i


    def find_branch_split_points(self, slice_curves):
        """returns point_list where branching happens
        """
        branch_points = []
        branch_curves = []
        nested_slice_curves = self.reorganize_by_height(slice_curves)

        for i, s_list in enumerate(nested_slice_curves[:-2]):
            curve_count01 = len(s_list)
            curve_count02 = len(nested_slice_curves[i+1])
            if curve_count01 != curve_count02:
                if curve_count01>curve_count02: s = s_list
                else: s = nested_slice_curves[i+1]
                pair = self.group_curves_center(s)
                branch_curves.extend([s[pair[0]].curve, s[pair[1]].curve])
                pair = self.closest_points_from_points(s[pair[0]].points,
                    s[pair[1]].points)
                avrg_p = self.avrg_between_points(pair)
                branch_points.append(avrg_p)

        return branch_points, branch_curves


    def avrg_between_points(self, point_list):
        """finds the average point in a list of points
        """
        sum_x = 0
        sum_y = 0
        sum_z = 0

        for p in point_list:
            sum_x += p.X
            sum_y += p.Y
            sum_z += p.Z

        avrg_x = sum_x/len(point_list)
        avrg_y = sum_y/len(point_list)
        avrg_z = sum_z/len(point_list)
        avrg_p = rg.Point3d(avrg_x, avrg_y, avrg_z)

        return avrg_p


    def closest_points_from_points(self, point_list01, point_list02):
        """finds the two closest points from two lists of points
        """
        min_dist = 99999

        for p01 in point_list01:
            for p02 in point_list02:
                dist = self.two_d_distance(p01,p02)
                if dist<min_dist: min_dist,pair=dist,(p01,p02)

        return pair


    def group_curves_center(self, slice_curves):
        """groups a list of curves into sub lists
        """
        min_dist = 99999

        for i,s01 in enumerate(slice_curves):
            for j,s02 in enumerate(slice_curves):
                if i!=j:
                    dist = self.two_d_distance(s01.center,s02.center)
                    if dist<min_dist: min_dist,pair=dist,(i,j)

        return pair


    def auto_align_seams(self, slice_curves, shift_ratio=0.005):
        """aligns all seams of all curves
        """
        aligned_slice_curves = []
        nested_slice_curves = self.reorganize_by_height(slice_curves)
        nested_seam_list = []

        branch_points, branch_curves = self.find_branch_split_points(slice_curves)

        for i, s_list in enumerate(nested_slice_curves):
            temp_seam_list = []
            for j, c in enumerate(s_list):
                center_point = self.get_center_slice_curves(c,s_list)
                if i==0:
                    _,v = self.change_seam(c, center_point)
                    temp_seam_list.append(v)
                else:
                    closest_seam = self.closest_point_from_curve(c.curve,
                        nested_seam_list[i-1])
                    _,v = self.change_seam(c, closest_seam)

                    min_dist = 99999
                    dist = min_dist
                    min_b = closest_seam
                    for b in branch_points:
                        if c.center.Z<=b.Z:dist=b.DistanceTo(c.center)
                        if min_dist>dist: min_dist,min_b=dist,b
                    _,v_shift = self.shift_seam(c, min_b, shift_ratio)
                    if v_shift:temp_seam_list.append(v_shift)
                    else: temp_seam_list.append(v)

                aligned_slice_curves.append(c)

            nested_seam_list.append(temp_seam_list)

        return aligned_slice_curves, branch_points, branch_curves


    def get_center_slice_curves(self, slice_curve, slice_curves, buffer=100):
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

        # dist_centers = self.two_d_distance(slice_curve.center, avrg_p)
        # centers = [s.center for s in slice_curves]
        # min_p,min_i = self.closest_point_from_points(slice_curve.center,
        #     centers)
        # dist_next = self.two_d_distance(slice_curve.center, min_p)
        #
        # if dist_centers<buffer:
        #     return avrg_p
        # elif dist_next<buffer:
        #     return min_p
        # else:
        #     return slice_curve.center


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
