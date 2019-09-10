
import Rhino.Geometry as rg
import sliceMath
reload(sliceMath)

"""import SliceFrame
"""
import sliceFrame
reload(sliceFrame)
from sliceFrame import SliceFrame

class SliceCurve(object):
    """This class creates a slice curve
    """

    def __init__(self, curve, line_definition=10,
        append_bool=True):
        """initiates the class with self.curve empty
        """
        self.curve = curve
        self.line_definition = line_definition
        self.append_bool = append_bool
        self.seam = curve.PointAtStart
        self.points = self.get_polyline_points(curve, self.append_bool)
        self.center = sliceMath.get_two_d_center_of_points(self.points)
        self.area = sliceMath.area_of_polygon(self.points)
        # self.spiral, self.spiral_points = self.get_spiral(self.points)
        self.frames = self.generate_sliceFrames(self.points)
        self.transition_curve = False


    @property
    def SCurve(self):
        """gets SCurve
        """
        return self.curve


    @SCurve.setter
    def SCurve(self, curve):
        """sets SCure
        """
        self.seam = curve.PointAtStart
        self.append_first_bool = True
        self.points = self.get_polyline_points(curve, self.append_bool)
        self.center = sliceMath.get_two_d_center_of_points(self.points)
        self.area = sliceMath.area_of_polygon(self.points)
        # self.spiral, self.spiral_points = self.get_spiral(self.points)
        self.frames = self.generate_sliceFrames(self.points)


    @SCurve.deleter
    def SCurve(self):
        """deletes SCurve
        """
        self.seam = None
        self.center = None
        self.area = None
        self.points = None
        self.resampled_points = None
        self.frames = None
        self.spiral = None


    def pop_first_point(self):
        """pops first point on curve
        """
        self.points.pop(0)
        new_curve = rg.PolylineCurve(self.points)
        self.SCurve = new_curve

    def pop_last_point(self):
        """pops first point on curve
        """
        self.points.pop(0)
        new_curve = rg.PolylineCurve(self.points)
        self.SCurve = new_curve


    def get_spiral(self, points, layer_height=1.0):
        """turns a planar curve into a spiral
        """

        move_dist = float(layer_height)/float(len(points))
        new_points = []

        for i, point in enumerate(points):
            move_trans = rg.Transform.Translation(0,0,move_dist*(i%len(points)))
            point.Transform(move_trans)

        return rg.PolylineCurve(points), points


    def generate_sliceFrames(self, points):
        """generate list of sliceFrames
        """
        frames = []
        for p in points:
            frame = SliceFrame(p)
            frames.append(frame)

        return frames


    def delete_points_same_direction(self, points, buffer=0.05):
        """deletes very close points
        """
        while True:
            pop_list = False
            for i,p in enumerate(points):
                if (i+1)<len(points) and i!=0:
                    vector_a = sliceMath.substract_two_vectors(p, points[i-1])
                    vector_b = sliceMath.substract_two_vectors(points[i+1], p)
                    angle = sliceMath.angle_two_vectors(vector_a, vector_b)
                    if abs(angle)<buffer:
                        pop_list = True
                        points.pop(i)
            if not pop_list:
                break

        return points


    def calculate_normals(self, closest_curve):
        """calculates the cantiliver normal for each point
        on the curve
        """
        self.normals = []

        for i,p in enumerate(self.points):
            _, c_double = closest_curve.ClosestPoint(p)
            c_point = closest_curve.PointAt(c_double)

            vector01 = rg.Vector3d(c_point)
            vector02 = rg.Vector3d(p)

            p_normal = vector02 - vector01
            p_normal.Unitize()
            angle = sliceMath.angle_two_vectors(rg.Vector3d(0,0,1), p_normal)

            if angle < 0.45:
                p_normal = z_vector

            # perp_vector = rg.Vector3d(0,0,1)
            # p_normal = p_normal + perp_vector
            # p_normal = p_normal + perp_vector

            self.normals.append(p_normal)


    def get_polyline_points(self, curve, append_first=True):
        """get points from polyline
        """

        polyline = curve.ToPolyline()
        segments = polyline.GetSegments()
        # points = [s[0] for s in segments]
        # points = self.delete_points_same_direction(points)
        if append_first:
            points = [s[0] for s in segments]
            points = self.delete_points_same_direction(points)
            points.append(segments[0][0])
        else:
            points = []
            points.extend([segments[0][0],segments[0][1]])

        return points
