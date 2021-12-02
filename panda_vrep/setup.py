from setuptools import setup
from glob import glob

package_name = 'panda_vrep'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.*')),
        ('share/' + package_name + '/models', glob('models/*.*')),
        ('share/' + package_name + '/urdf', glob('urdf/*.*')),
        ('lib/' + package_name, [package_name+"/" + el for el in ["sim.py", "simConst.py","remoteApi.so", "trajectory_follower.py"]])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florent Audonnet',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vrep_spawn= panda_vrep.vrep_spawn:main"
        ],
    },
)
