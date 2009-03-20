#!/usr/bin/python

"""
setup.py file for SWIG example
"""

from distutils.core import setup, Extension


example_module = Extension('_ProcessArcher',
                           sources=['ProcessArcher_wrap.cxx', 'ProcessArcher.cpp'],
                           )

setup (name = 'ProcessArcher',
       version = '0.1',
       author      = "SWIG Docs",
       description = """Simple swig example from docs""",
       ext_modules = [example_module],
       py_modules = ["ProcessArcher"],
       )
