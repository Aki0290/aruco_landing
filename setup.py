from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_landing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launchディレクトリの中にある.launch.pyファイルをすべてインストール対象に含める
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akihiro',
    maintainer_email='nakayama.akihiro.p5@dc.tohoku.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'landing_node = aruco_landing.landing_node:main', # async mainをラップ
        ],
    },
)
