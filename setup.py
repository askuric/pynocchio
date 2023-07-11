from setuptools import setup

setup(name='pynocchio',
    version='1.4.1',
    description='A pip package implementing a simple wrapper of the pinocchio robot model.',
    url='https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    license='MIT',
    packages=['pynocchio'],
    requires_python='>=3.7',
    install_requires=['numpy','meshcat'],
    zip_safe=False)
