from setuptools import setup

package_name = 'tracker_kf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='akash',
    description='Kalman filter tracker node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tracker_kf_node = tracker_kf.tracker_kf_node:main',
        ],
    },
)
