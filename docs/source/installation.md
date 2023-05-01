# Installation

All you need to do to install it is:
```
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```
And include it to your python project
```python
import pynocchio.RobotWrapper
```

### Installing from source

In order to install the package from source clone the git repository 
```sh
git clone https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```
and install it using 
```sh
cd pynocchio
pip install -e .
```
then your can make import the `pynocchio` package as usual in your applications.

## Testing the code

Furthermore if installing from source you can run all the unit tests in order to verify that the installation went well
```shell
pip install pytest 
pytest tests/*
```

and coverage report

```shell
pip install coverage
coverage run -m pytest tests/test.py
coverage report -m
```

and for html version

```shell
coverage html
```

Automatically generated coverage report is [here](coverage/htmlcov)