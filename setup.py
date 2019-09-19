from setuptools import setup

setup(
   name='cloudDist',
   version='0.0.1',
   description='A module for interpreting ply files as signed distance functions.',
   author='Henry Nelson',
   author_email='nels8279@umn.edu',
   packages=['cloudDist'],  #same as name
   install_requires=['numpy', 'scipy', 'matplotlib'], #external packages as dependencies
   scripts=[]
)
