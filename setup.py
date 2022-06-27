from setuptools import setup

setup(
    name='fanuc_motion_program_exec',
    version='0.0.2',
    description='Simple module to execute motion commands on FANUC robots and log results',
    url='https://github.com/eric565648/fanuc_motion_program_exec',
    py_modules=['fanuc_motion_program_exec_client'],
    install_requires=[
        'numpy'
    ]
)