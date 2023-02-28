from setuptools import setup

setup(name='pynocchio',
    version='0.2',
    description='A pip package implementing a simple wrapper of the pinocchio robot model.',
    url='https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    license='MIT',
    packages=['pynocchio'],
    install_requires=['numpy'],
    zip_safe=False)
