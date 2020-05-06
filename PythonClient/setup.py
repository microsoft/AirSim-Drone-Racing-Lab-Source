import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="airsimdroneracinglab",
    version="1.0.2",
    author="Microsoft, Ratnesh Madaan, Sai Vemprala",
    author_email="ratnesh.madaan@microsoft.com, sai.vemprala@microsoft.com",
    description="Python package for AirSim Drone Racing Lab, built on Microsoft AirSim - an open source simulator based on Unreal Engine for autonomous vehicles from Microsoft AI & Research",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/microsoft/AirSim-Drone-Racing-Lab",
    packages=setuptools.find_packages(),
    license="MIT",
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ),
    install_requires=["msgpack-rpc-python", "numpy"],
)
