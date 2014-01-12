## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ompl_demo','ompl_gui'],
    package_dir{'ompl_demo': './src',
                'ompl_gui': './src'},
    requires=['genpy', 'numpy', 'roslib', 'rospkg']
)

setup(**d)

