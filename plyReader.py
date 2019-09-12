import os
import numpy as np

class DirtyPlyFile:
    def __init__(self, filename):
        self.filename = filename
        self.contents = None
        self.elements = None
        self.vertex = None
        self._read_contents()
        self._read_header()
        self._read_vertex()

    def _read_contents(self):
        if os.path.isfile(self.filename):
            with open(self.filename, 'r') as file_object:
                self.contents = file_object.readlines()
        else:
            raise FileNotFoundError(f"{self.filename} does not exist.")

    def _read_header(self):
        self.elements = []
        for line in self.contents[:self.contents.index('end_header\n')]:
            split_line = line.split()
            if split_line[0] == 'element':
                self.elements.append((split_line[1], int(split_line[2])))

    def _read_vertex(self):
        pre_vertex = self.contents.index('end_header\n')+1
        vertex = 0
        for elem, num in self.elements:
            if elem == 'vertex':
                vertex = num
                break
            else:
                pre_vertex = pre_vertex+num
        self.vertex = np.array([[float(y) for y in x.split()] for x in self.contents[pre_vertex:pre_vertex+vertex]])
        self.contents = []
