from setuptools import setup

setup(name='pypinocchio',
    version='0.1',
    description='A pip package implementing a simple wrapper of the pinocchio robot model.',
    url='https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    license='MIT',
    packages=['pypinocchio'],
    install_requires=['pinocchio','numpy'],
    zip_safe=False)