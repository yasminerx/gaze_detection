from setuptools import setup, find_packages

setup(
    name="gaze_lib",    
    version="0.1",
    packages=find_packages(),
    install_requires=[       
        "filterpy"
    ],
    author="yasmine",
    description="gaze detection python module",
    python_requires='>=3.7',
)