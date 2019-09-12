from scipy.spatial import cKDTree as KDTree
from plyReader import DirtyPlyFile

class DistFunc:
    def __init__(self, plyfile, chan=3, offset=0.0, normalize_data=True, n_jobs=-1):
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
