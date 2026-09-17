from setuptools import find_packages, setup
from glob import glob

package_name = "arms_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        # 도커는 config/ 를 볼륨으로 직접 마운트하지만, 호스트에서 노드를 띄울 땐
        # (SITL detection:=host) share 에서 --params-file 로 읽어야 한다.
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="A.R.M.S. detection nodes (fusion, YOLO docker)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arms_detection_node = arms_detection.arms_detection_node:main",
        ],
    },
)
