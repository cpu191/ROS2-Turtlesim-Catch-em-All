from setuptools import find_packages, setup

package_name = 'turtle_intercept'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='giathinh1907@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_manager=turtle_intercept.turtle_manager:main",
            "turtle_intercept=turtle_intercept.turtle_intercept:main"
        ],
    },
)
