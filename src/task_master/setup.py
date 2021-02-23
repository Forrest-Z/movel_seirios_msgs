import setuptools

setuptools.setup(
    name="mission_control",
    version="0.0.1",
    author="Ryan Lee",
    author_email="ryan@movel.ai",
    description="An auxilliary task management system",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
)
