from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   version="0.3.1",
   packages=["scf4_control"],
   package_dir={"": "src"}
)

setup(**d)