from setuptools import find_packages
from setuptools import setup

setup(
    name='chessbot_controller',
    version='0.0.0',
    packages=find_packages(
        include=('chessbot_controller', 'chessbot_controller.*')),
)
