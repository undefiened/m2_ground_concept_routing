from pkg_resources import parse_requirements
from setuptools import setup, Extension, find_packages
# from Cython.Build import cythonize

with open('requirements.txt') as f:
    required = f.read().splitlines()


setup(
    name='routing_M2',
    version='0.1',
    author='Leo',
    packages=find_packages(include=['ground_routing.*', 'ground_routing']),
    # ext_modules=cythonize('hexagonal_path_planner/misc_functions.pyx'),
    install_requires=required,
    zip_safe=False
)
