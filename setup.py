from setuptools import setup

setup(
    name="quest2ros",
    version="0.0.0",
    maintainer="mcw",
    maintainer_email="mwelle@kth.se",
    description="The quest2ros package",
    packages=["quest2ros"],
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    license="TODO",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/quest2ros"]),
        ("share/quest2ros", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "quest2ros = quest2ros.ros2quest:main",
        ],
    },
)
