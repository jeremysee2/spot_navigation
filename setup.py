from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["spot_navigation"],
    scripts=["test/spot_nav_test.py", "test/nav_ros_test.py"],
    package_dir={"": "src"},
)

setup(**d)
