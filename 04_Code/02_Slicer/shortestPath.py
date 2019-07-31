import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections

class ShortestPath():

    """This class finds the shortest path between a list of ordered points
    """

    def __init__(self):

        pass

    def generate_path(self, point_list, layer_height):

        height_nested_list = self.order_by_height(point_list, layer_height)

        shortest_path = self.get_shortest_path(height_nested_list)

        return shortest_path, height_nested_list


    def get_shortest_path(self, nested_list):

        shortest_path = []

        # print len(nested_list)

        for i, point_list in enumerate(nested_list):

            cluster_list, standard_dist, avrg_center = self.order_by_cluster(point_list)
            len_cluster_list = len(cluster_list)

            for j, c in enumerate(cluster_list):

                if len(shortest_path) == 0:

                    # print c

                    temp_list = self.shift_list_index(c, avrg_center)

                    print avrg_center

                    for p in temp_list:

                        shortest_path.append(p)

                else:

                    temp_list = self.shift_list_index(c, shortest_path[-1])

                    print len(temp_list)

                    for p in temp_list:

                        shortest_path.append(p)

        return shortest_path


    def order_by_cluster(self, point_list):

        standard_dist = self.two_d_distance(point_list[0], point_list[1])

        temp_list = []
        nested_list = []

        avrg_center = self.get_avrg_center(point_list)

        for i, p01 in enumerate(point_list[:-1]):

            p02 = point_list[i+1]
            dist_p01_p02 = self.two_d_distance(p01, p02)

            if dist_p01_p02 < standard_dist*2:

                temp_list.append(p01)
                # print p01

                if i == (len(point_list) - 2):

                        temp_list.append(p02)
                        temp_list.append(temp_list[0])
                        # print len(temp_list)
                        # temp_list.append(temp_list[0])
                        # print len(temp_list)
                        nested_list.append(temp_list)
                        temp_list = []

            else:

                temp_list.append(p01)
                # print temp_list[0]
                temp_list.append(temp_list[0])
                print len(temp_list)
                # print "out"
                # temp_list.append(p02)
                # temp_list.append(temp_list[0])
                nested_list.append(temp_list)
                temp_list = []

        return nested_list, standard_dist, avrg_center


    def two_d_distance(self, point01, point02):

        dist = math.sqrt((point01.X - point02.X)**2 + (point01.Y - point02.Y)**2)

        return dist


    def order_by_height(self, point_list, layer_height):

        nested_list = []

        count = 0

        largest_height = point_list[-1].Z
        num_layers = int(largest_height/layer_height + 1)

        for layer in range(num_layers):

            temp_list = []

            for i, p in enumerate(point_list[count:]):

                if self.is_almost_equal(p.Z, layer*layer_height):

                    temp_list.append(p)

                else:

                    count += i

                    break

            nested_list.append(temp_list)

        return nested_list


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon


    def shift_index(self, seq, shift=1):

        return seq[shift:] + seq[:shift+1]


    def shift_list_index(self, point_list, closest_point):

        min_dist = 999999999

        for i, p in enumerate(point_list):

            dist = self.two_d_distance(p, closest_point)

            if dist < min_dist:

                min_dist = dist
                min_index = i

        # print i
        # print point_list
        point_list = self.shift_index(point_list, min_index)
        # print point_list

        return point_list


    def get_avrg_center(self, point_list):

        sum_x = 0
        sum_y = 0

        for p in point_list:

            sum_x += p.X
            sum_y += p.Y

        avrg_x = sum_x/len(point_list)
        avrg_y = sum_y/len(point_list)

        avrg_p = rg.Point3d(avrg_x, avrg_y, point_list[0].Z)

        return avrg_p
