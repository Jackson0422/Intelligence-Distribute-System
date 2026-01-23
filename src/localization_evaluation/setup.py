import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'localization_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'models', 'tb3_basic'), glob('models/tb3_basic/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_baseline = localization_evaluation.track_baseline:main',
            'pose_eval = localization_evaluation.pose_eval_node:main',
            'track_multibot = localization_evaluation.track_multibot:main',
            'pose_eval_multibot = localization_evaluation.pose_eval_multibot:main',
            'decentralized_coloc_agent = localization_evaluation.decentralized_coloc_agent:main',
            'pose_eval_coloc = localization_evaluation.pose_eval_coloc:main',
            'ground_truth_logger = localization_evaluation.ground_truth_logger:main',
        ],
    },
)
