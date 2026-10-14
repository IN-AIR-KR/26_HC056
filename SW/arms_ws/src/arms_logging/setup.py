from glob import glob
from setuptools import setup


setup(
    name="arms_logging",
    version="0.1.0",
    packages=["arms_logging"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/arms_logging"]),
        ("share/arms_logging", ["package.xml"]),
        ("share/arms_logging/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "flight_recorder = arms_logging.flight_recorder:main",
    ]},
)
