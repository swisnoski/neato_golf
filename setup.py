from setuptools import find_packages, setup

package_name = "neato_golf_donkey_kong"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swisnoski",
    maintainer_email="swisnoski@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "neato_tracker = neato_golf_donkey_kong.neato_tracker:main",
            "neato_sort = neato_golf_donkey_kong.sort:main",
            "neato_bbox = neato_golf_donkey_kong.bbox:main",
        ],
    },
)
