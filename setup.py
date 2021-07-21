from setuptools import setup, Extension, find_packages
# from Cython.Build import cythonize

setup(
    name='routing_M2',
    version='0.1',
    author='Leo',
    packages=find_packages(include=['ground_routing.*', ]),
    # ext_modules=cythonize('hexagonal_path_planner/misc_functions.pyx'),
    zip_safe=False
)
