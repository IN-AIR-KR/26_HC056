import os
from glob import glob
from setuptools import find_packages, setup

package_name = "arms_sim"


def _tree(src_dir):
    """src_dir 아래 모든 파일을 share/<pkg>/<같은 경로> 로 설치하는 data_files 항목.
    ament_python 의 data_files 는 디렉토리 재귀를 안 해줘서 (meshes/ 등) 직접 나열."""
    out = []
    for root, _, files in os.walk(src_dir):
        if files:
            out.append((os.path.join("share", package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        # 시뮬 자산: assets/{models,worlds}(Gazebo) + assets/px4/airframes(PX4 ROMFS).
        #   메시가 몇 MB 라 colcon build --symlink-install 권장.
        *_tree("assets"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arms",
    maintainer_email="dev@arms.local",
    description="A.R.M.S. SITL 전용 지원: PX4 SITL 런치 + Gazebo 자산 + 표적 심판(referee) + 튜닝/개발 콘솔(panel). 실기체 미사용.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "referee = arms_sim.referee:main",
            "panel = arms_sim.panel:main",
            "sitl_reset_manager = arms_sim.sitl_reset_manager:main",
            "sitl_viz = arms_sim.sitl_viz:main",
            "launcher = arms_sim.launcher:main",
        ],
    },
)
