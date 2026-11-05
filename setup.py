from setuptools import setup

package_name = "sim2real"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "calibration.json"]),
        ("share/" + package_name + "/launch", ["launch/sim2real.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeajune0221",
    maintainer_email="jeajune11117@gmail.com",
    description="Sim joint angles to the real JAX arm",
    license="TODO",
    entry_points={
        "console_scripts": [
            "sim2real_node = sim2real.node:main",
            "calibrate = sim2real.calibrate:main",
            "play_trajectory = sim2real.play:main",
        ],
    },
)
