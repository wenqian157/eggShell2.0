
import Rhino.Geometry as rg

class ImportTxt():

    def __init__(self, file_path):

        self.file_path = file_path

    def get_points(self):

        self.points = []

        with open(self.file_path, "r") as f:

            for line in f.readlines():

                data = line[line.find("[")+1 : line.find("]")].rstrip().split(',')

                point = rg.Point3d(float(data[0]), float(data[1]), float(data[2]))

                self.points.append(point)

        return self.points
