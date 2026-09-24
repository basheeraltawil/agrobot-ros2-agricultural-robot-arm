from glob import glob

from setuptools import find_packages, setup

package_name = 'aibomech_agrobot_tasks'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Basheer Al-Tawil',
    maintainer_email='basheer.altawil@gmail.com',
    description='Agricultural task nodes for the AIBOMECH AgroBot: harvesting, transplanting, weeding, inspection.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strawberry_harvest = aibomech_agrobot_tasks.scenarios.strawberry_harvest:main',
            'seedling_transplant = aibomech_agrobot_tasks.scenarios.seedling_transplant:main',
            'precision_weeding = aibomech_agrobot_tasks.scenarios.precision_weeding:main',
            'plant_inspection = aibomech_agrobot_tasks.scenarios.plant_inspection:main',
            'estop = aibomech_agrobot_tasks.estop:main',
        ],
    },
)
