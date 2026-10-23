#!/usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 29, 2025"
__project__   = "PyBEAR"
__version__   = "1.0.0"
__status__    = "Production"

from setuptools import find_namespace_packages, setup

setup(
        name='PyBEAR',
        version=__version__,
        description='BEAR SDK python package',
        author='Westwood Robotics Corporation',
        author_email='info@westwoodrobotics.io',
        url='https://github.com/Westwood-Robotics/PyBEAR',
        license='Apache License, Version 2.0',
        packages=find_namespace_packages(include=['pybear*']),
        install_requires=['pyserial', 'termcolor']
)
