#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
import glob
import os

version = "0.3.0"

print(find_packages(exclude=['tests', 'examples']))

setup(
    name='impact-meco',
    version=version,
    author="MECO-Group",
    author_email="joris.gillis@kuleuven.be",
    description="DIRAC MPC",
    license='LICENSE',
    url='https://gitlab.kuleuven.be/meco-software/impact',
    packages=find_packages(exclude=['tests', 'examples']),
    include_package_data=True,
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    install_requires=[
        'casadi>=3.5,<4.0',
        'pyyaml',
        'lxml',
        'rockit-meco>=0.1.35',
    ],
    download_url='https://gitlab.kuleuven.be/meco-software/impact/-/archive/v%s/impact-v%s.tar.gz' % (version, version)
)
