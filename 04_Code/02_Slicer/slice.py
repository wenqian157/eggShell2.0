
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import math

import sliceMath
reload (sliceMath)

import sliceCurve
reload(sliceCurve)
from sliceCurve import SliceCurve

import shortestPath
reload (shortestPath)
from shortestPath import ShortestPath

import graphClass
reload(graphClass)
from graphClass import Graph

from itertools import combinations


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
        branch_points, branch_curves = [],[]

        self.contour_curves = self.create_contour_curves(mesh)
        self.slice_curves = self.create_sliced_curves(self.contour_curves)
        self.slice_curves, branch_points, branch_curves = self.auto_align_seams(self.slice_curves, 1)
        self.construct_tcp_frames(self.slice_curves)
        self.slice_curves = self.shortest_path_one(self.slice_curves)
        self.slice_curves = self.shortest_path_two(self.slice_curves, 7)

        return self.slice_curves, branch_points, branch_curves
        # return self.slice_curves


    def construct_tcp_frames(self, slice_curves_base):
        """constructs tcp frames
        """

        len_points = 0
        for s in slice_curves_base:
            len_points += len(s.points)

        nested_slice_curves = self.reorganize_by_height(slice_curves_base)
        tcp_vector = [-1,0,0]
        tcp_frame_list = []

        for i,slice_curves in enumerate(nested_slice_curves):
            for s in slice_curves:
                min_i,min_c = self.closest_curve_to_point(s.seam, nested_slice_curves[i-1])
                # closest_curve = nested_slice_curves[i-1][min_i]

                for f in s.frames:
                    if i == 0:
                        f.cantiliver = 0
                    else:
                        normal=f.calc_cantiliver_normal(f.point, min_c.curve)
                        f.cantiliver = abs(sliceMath.angle_two_vectors(normal,[0,0,1]))
                    # print f.cantiliver


    def closest_curve_to_point(self, point, curve_list):

        min_dist = 999999

        for i,c in enumerate(curve_list):
            dist = sliceMath.three_d_distance(point,c.seam)
            if min_dist>dist:
                min_dist,min_i,min_c = dist,i,c

        # print min_dist

        return i,min_c


    def shortest_path_one(self, slice_curves):
        """reorganizes slice_curves by shortest path
        """
        nested_slice_curves = self.reorganize_by_height(slice_curves)
        shortest_path = []

        for s_curves in nested_slice_curves:
            new_list = [s_curves[0]]
            s_curves_copy = s_curves[1:]

            while True:
                if len(new_list) >= len(s_curves):
                    break
                else:
                    min_dist = 999999
                    for i,s in enumerate(s_curves_copy):
                        dist = sliceMath.two_d_distance(s.seam, new_list[-1].seam)
                        if dist<min_dist: min_s,min_i = s,i

                    new_list.append(s)
                    s_curves_copy.pop(min_i)

            for n in new_list:
                shortest_path.append(n)

        return shortest_path
        # nested_slice_curves = self.reorganize_by_height(slice_curves)
        #
        # for s_curves in nested_slice_curves:
        #     seam_start = s_curves[0].seam
        #     seam_list = [s.seam for s in s_curves[1:]]
        #     _,max_i,_ = sliceMath.farthest_point_from_point(seam_start,seam_list)
        #     # print max_i
        #     graph = Graph()
        #     comb = combinations([i for i in range(len(s_curves))], 2)
        #     for i in list(comb):
        #         weight = sliceMath.two_d_distance(s_curves[i[0]].seam,
        #             s_curves[i[1]].seam)
        #         graph.add_edge(i[0], i[1], weight)
        #     shortest_path = graphClass.dijsktra(graph, 0, max_i+1)
            # print shortest_path
            # edge_len = 0
            # edge_list = []
            # for i,s_a in enumerate(s_curves):
            #     for j,s_b in enumerate(s_curves[1:]):
            #         if i!=j+1:
            #             # edge_len+=1
            #             weight = sliceMath.two_d_distance(s_a.seam,s_b.seam)
            #             edge_list.append([i, j+1, weight])
            # graph = Graph()
            # for e in edge_list:
            #     print e[0], e[1], e[2]
            #     graph.add_edge(e[0], e[1], e[2])
            # # print len(s_curves), min_i
            # shortest_path = graphClass.dijsktra(graph, 0, min_i)
            # # print shortest_path


    def create_contour_curves(self, mesh):
        """calls the createcontourcurves command from rhino,
        moves it to origin, erases unwanted curves and aligns seams
        """
        min_z, max_z = self.get_min_max_z(mesh)
        start_contour, end_contour = self.define_start_end(720, 850, True)
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
                    dist = sliceMath.two_d_distance(s01.seam,s02.seam)
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



    def shortest_path_two(self, slice_curves, max_branch_diff=10):
        """finds the shortest path based on max_branch_diff
        """
        slice_curves = self.join_close_curves(slice_curves, 10)

        buffer = 10
        max_list_len = len(slice_curves)
        shortest_path_list = []
        loop_slice = slice_curves[0]
        count_a = 0

        while True:

            # if len(shortest_path_list) >= max_list_len-1: break
            if count_a >= max_list_len-1: break

            index_list, loop_index = [], []
            slice_curves_dup = slice_curves[:]
            count_b = 0

            for i, slice_curve in enumerate(slice_curves):
                if count_b == max_branch_diff:
                    if len(loop_index) == 0:
                        loop_slice = shortest_path_list[-1]
                        # print shortest_path_list[-1]
                    else:loop_slice = slice_curves[loop_index[0]]
                    trans_p_a = rg.Point3d(shortest_path_list[-1].seam[0],
                        shortest_path_list[-1].seam[1], shortest_path_list[-1].seam[2]+15)
                    trans_p_b = rg.Point3d(loop_slice.seam[0],
                        loop_slice.seam[1], loop_slice.seam[2]+15)
                    trans_curve = rg.PolylineCurve([trans_p_a, trans_p_b])
                    slice_curve_trans = SliceCurve(trans_curve, self.line_definition,False)
                    slice_curve_trans.transition_curve = True
                    for f in slice_curve_trans.frames:
                        f.cantiliver = 0
                        f.transition = True
                    shortest_path_list.append(slice_curve_trans)
                    break

                if i == 0: compare_slice = loop_slice
                else: compare_slice = shortest_path_list[-1]

                dist = max(sliceMath.three_d_distance(slice_curve.center, compare_slice.center),
                    sliceMath.three_d_distance(slice_curve.seam, compare_slice.seam))

                # if dist<buffer and abs(slice_curve.area-compare_slice.area)<2000 and abs(slice_curve.seam[2]-compare_slice.seam[2])<self.layer_height*1.1:
                if dist<buffer and abs(slice_curve.seam[2]-compare_slice.seam[2])<self.layer_height*1.1:

                    # slice_curve.pop_last_point()
                    # if count != 0:slice_curve.frames.pop(0)
                    # else:slice_curve.frames.pop(-1)
                    shortest_path_list.append(slice_curve)
                    count_a += 1
                    index_list.append(i)
                    count_b += 1

                else: loop_index.append(i)

            if len(loop_index) == 0: loop_slice = shortest_path_list[-1]
            else: loop_slice = slice_curves[loop_index[0]]

            if len(index_list) == 0:
                shortest_path_list.append(slice_curves[0])
                count_a += 1

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

            if sliceMath.is_almost_equal(c_height_01, c_height_02):

                temp_list.append(s)

            else:

                temp_list.append(s)
                nested_list.append(temp_list)
                temp_list = []

        return nested_list


    def shift_seam(self, slice_curve, target_point, shift_ratio=0.01, loops=3):
        """shifts seam of curve towards desired point
        """
        length = slice_curve.curve.GetLength()
        shift = shift_ratio/length
        seam_bool=True
        tuple_list = []

        for i in range(-loops,loops+1):
            point = slice_curve.curve.PointAt(shift*i)
            dist = sliceMath.three_d_distance(point, target_point)
            tuple_list.append((dist,point,shift*i))

        sorted_list = sorted(tuple_list, key=lambda tup: tup[0])
        seam_point, seam_domain = sorted_list[0][1], sorted_list[0][2]

        seam_success = slice_curve.curve.ChangeClosedCurveSeam(seam_domain)
        set_domain = rg.Interval(0,1)
        slice_curve.curve.Domain = set_domain
        slice_curve.SCurve = slice_curve.curve
        return seam_success,seam_point


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
        # print min_p

        for p in point_list:
            _, domain = curve.ClosestPoint(p)
            point = curve.PointAt(domain)
            dist = sliceMath.three_d_distance(p, point)
            if min_dist>dist:
                min_dist = dist
                min_p=p

        return min_p, min_dist

    def closest_point_from_points(self, point, point_list):
        """finds closest point from list of point
        """
        min_dist = 99999

        for i,p in enumerate(point_list):
            # dist = self.two_d_distance(p, point)
            dist = sliceMath.three_d_distance(p, point)
            if min_dist>dist:
                min_dist,min_p,min_i=dist,p,i

        return min_p,min_dist


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
                if curve_count01>curve_count02:s = s_list
                else:s = nested_slice_curves[i+1]
                pair = self.group_curves_center(s, 20)
                if pair:
                    branch_curves.extend([s[pair[0]].curve, s[pair[1]].curve])
                    pair,_ = sliceMath.closest_points_from_points(s[pair[0]].points,
                        s[pair[1]].points)
                    avrg_p = sliceMath.get_three_d_center_of_points(pair)
                    avrg_p = rg.Point3d(avrg_p[0], avrg_p[1], avrg_p[2])
                    branch_points.append(avrg_p)

        return branch_points, branch_curves


    def group_curves_center(self, slice_curves, buffer=50):
        """groups a list of curves into sub lists
        """
        min_dist = buffer
        pair = None

        for i,s01 in enumerate(slice_curves):
            for j,s02 in enumerate(slice_curves):
                if i!=j:
                    _,dist = sliceMath.closest_points_from_points(s01.points, s02.points)
                    if dist<min_dist:min_dist,pair=dist,(i,j)

        return pair


    def generate_branch_lines(self, branch_points):
        """generates a collection of lines
        from branch points
        """
        pass


    def auto_align_seams(self, slice_curves, shift_ratio=0.005, buffer=1):
        """aligns all seams of all curves
        """
        aligned_slice_curves, nested_seam_list = [],[]
        branch_points, branch_curves = self.find_branch_split_points(slice_curves)
        nested_slice_curves = self.reorganize_by_height(slice_curves)

        for i, s_list in enumerate(nested_slice_curves):
            temp_seam_list = []

            for j, c in enumerate(s_list):
                branch_points_z = [b for b in branch_points if b[2]>c.center[2]-5]

                if i==0:
                    center_points = [s.center for s in s_list]
                    center_point = sliceMath.get_two_d_center_of_points(center_points)
                    center_point = rg.Point3d(center_point[0], center_point[1], center_point[2])
                    _,v = self.change_seam(c, center_point)
                    # temp_seam_list.append(v)

                else:
                    point_change_seam, dist_seam = self.closest_point_from_curve(c.curve,
                        nested_seam_list[i-1])
                    _,v = self.change_seam(c, point_change_seam)

                    if len(branch_points_z)>0:

                        if dist_seam>self.layer_height*10:
                            point_change_seam, dist_branch = self.closest_point_from_curve(c.curve,
                                branch_points_z)
                            _,v = self.change_seam(c, point_change_seam)

                        min_b,dist_center=self.closest_point_from_points(c.seam,
                        branch_points_z)
                        if min_b and dist_center<200 and c.area>1500:
                            _,v = self.shift_seam(c, min_b, shift_ratio)
                            # temp_seam_list.append(v)
                        # else: min_b=False
                        # if v_shift:temp_seam_list.append(v_shift)
                        # else: temp_seam_list.append(v)

                temp_seam_list.append(v)

                aligned_slice_curves.append(c)

            nested_seam_list.append(temp_seam_list)

        return aligned_slice_curves, branch_points, branch_curves


    def get_center_slice_curves(self, slice_curves):
        """gets center of several slicecurve type
        """

        sum_x = 0
        sum_y = 0

        for s in slice_curves:
            sum_x += s.center[0]
            sum_y += s.center[1]

        avrg_x = sum_x/len(slice_curves)
        avrg_y = sum_y/len(slice_curves)
        avrg_p = rg.Point3d(avrg_x, avrg_y, s.center[2])
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


    def match_curve_direction(self, curves):
        """matches all curves directions
        """
        for i in range(0, len(curves)-1):

            vector01 = rg.Vector3d(curves[i].PointAt(0)) - rg.Vector3d(curves[i].PointAt(.001))
            vector02 = rg.Vector3d(curves[i+1].PointAt(0)) - rg.Vector3d(curves[i+1].PointAt(.001))
            vector_angle = rg.Vector3d.VectorAngle(vector01, vector02)

            if vector_angle > math.pi/4:
                curves[i+1].Reverse()


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
