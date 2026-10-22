from glob import glob

from setuptools import setup

package_name = "ram_bug"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="T",
    maintainer_email="n.thanadett@gmail.com",
    description="RAM-Bug goal-reaching evader for TurtleBot3",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ram_bug_node = ram_bug.ram_bug_node:main",
            "trap_chaser_node = ram_bug.trap_chaser_node:main",
            "peer_link_node = ram_bug.peer_link_node:main",
        ],
    },
)
