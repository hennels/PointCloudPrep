import os
import numpy as np
from scipy.spatial import cKDTree as KDTree

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

class DistFunc:
    def __init__(self, plyfile, chan=3, offset=0.0, normalize_data=False, n_jobs=-1):
        self.filename = plyfile
        self.offset = offset
        self.plyfile = None
        self.data = None
        self.n_jobs = n_jobs
        self.chan = chan
        self.load_ply()
        self.set_data()
        if normalize_data:
            self.normalize_data()
        self.make_tree()

    def load_ply(self):
        self.plyfile = DirtyPlyFile(self.filename)

    def set_data(self):
        if self.plyfile is None:
            self.load_ply()
        self.data = self.plyfile.vertex[:, :self.chan]

    def normalize_data(self):
        self.data = (self.data-self.data.mean(0, keepdims=True))/self.data.std(0, keepdims=True)

    def make_tree(self):
        self.tree = KDTree(self.data)

    def sample_dist(self, X):
        return self.tree.query(X, n_jobs=self.n_jobs)[0]-self.offset

    def random_sample_cube(self, n=100):
        X = 2*np.random.rand(n,3)-1
        return X, self.tree.query(X, n_jobs=self.n_jobs)[0]-self.offset
    
    def random_sample_sphere(self, n=100, R=1.0):
        X = np.random.rand(n,3)
        phi = 2*np.pi*X[:,0]
        theta = np.arccos(2*X[:,1]-1)
        r = R*np.cbrt(X[:,2])
        stheta = np.sin(theta)
        X[:,0] = stheta*np.cos(phi)
        X[:,1] = stheta*np.sin(phi)
        X[:,2] = np.cos(theta)
        X = r.reshape(-1,1)*X
        return X, self.tree.query(X, n_jobs=self.n_jobs)[0]-self.offset
