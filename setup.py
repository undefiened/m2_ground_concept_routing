from setuptools import setup, Extension
from Cython.Build import cythonize

setup(
    name='routing_hex',
    version='0.1',
    author='Leo',
    ext_modules=cythonize('misc_functions.pyx'),
    zip_safe=False
)
