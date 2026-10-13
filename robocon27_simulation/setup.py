from glob import glob

from setuptools import find_packages, setup

package_name = 'robocon27_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/meshes", glob("meshes/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        (
            "share/" + package_name + "/worlds",
            glob("worlds/*.sdf") + glob("worlds/*.xacro"),
        ),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        (
            "share/" + package_name + "/worlds/meshes",
            glob("worlds/meshes/*"),
        ),



    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed-amr',
    maintainer_email='mohamed.amr9327@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_teleop = robocon27_simulation.wheel_teleop:main',
        ],
    },
)
