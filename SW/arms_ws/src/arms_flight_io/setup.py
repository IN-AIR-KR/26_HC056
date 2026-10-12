from setuptools import setup

setup(
    name="arms_flight_io",
    version="0.1.0",
    packages=["arms_flight_io"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/arms_flight_io"]),
                ("share/arms_flight_io", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "arms_crsf_io_node = arms_flight_io.crsf_io_node:main",
        "sitl_bridge_node = arms_flight_io.sitl_bridge_node:main",
    ]},
)
