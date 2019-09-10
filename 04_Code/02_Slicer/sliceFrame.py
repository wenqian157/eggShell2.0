
import Rhino.Geometry as rg
import sliceMath
reload (sliceMath)

class SliceFrame(object):
    """This class creates a slice point
    """

    def __init__(self, point):
        """initiates the class
        """
        self.point = point
        self.cantiliver = None
        self.frame = self.calc_frame(self.point)
        self.transition = False


    def calc_frame(self, point):
        """calculate frame
        """
        tcp_vector = [-1,0,0]
        normal = [0,0,1]

        point_y = sliceMath.cross_product(tcp_vector, normal)
        vector_y = rg.Vector3d(rg.Point3d(point_y[0], point_y[1], point_y[2]))
        point_x = sliceMath.cross_product(normal, vector_y)
        vector_x = rg.Vector3d(rg.Point3d(point_x[0], point_x[1], point_x[2]))
        tcp_frame = rg.Plane(point, vector_y, vector_x)

        self.frame = tcp_frame
        return tcp_frame


    def calc_cantiliver_normal(self, point, curve):
        """calculates cantiliver normal
        """
        bool, c_double = curve.ClosestPoint(point)
        c_point = curve.PointAt(c_double)
        vector01 = rg.Vector3d(c_point)
        vector02 = rg.Vector3d(point)
        p_normal = vector02 - vector01
        p_normal.Unitize()

        return p_normal
