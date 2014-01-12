## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ompl_gui', 'ompl_demo'],
    package_dir{'': 'src'},
    requires=['genpy', 'numpy', 'roslib', 'rospkg']
)

setup(**d)

