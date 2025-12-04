from setuptools import setup

setup(
    name="lemonlib",
    version="0.1.0",
    description="Library used by frc 5113",
    url="https://github.com/FRC5113/LemonLib",
    author="Combustible Lemons",
    author_email="robotics@mtps.us",
    license="Unlicense",
    packages=["lemonlib"],
    install_requires=[
        "phoenix6==25.1.0",
        "photonlibpy==2025.1.1",
        "robotpy-apriltag==2025.2.1",
        "robotpy-commands-v2==2025.1.1",
        "robotpy-wpilib-utilities==2025.0.0",
        "robotpy-wpimath==2025.2.1",
        "robotpy-wpiutil==2025.2.1",
        "typing_extensions==4.12.2",
        "wpilib==2025.2.1",
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
    ],
)
