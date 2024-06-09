from setuptools import find_packages, setup

package_name = 'straight_test_2'

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
    maintainer='brandon',
    maintainer_email='22807632@sun.ac.za',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "Moving_forward = straight_test_2.test_moving_forward:main",
            "Opt_Moving_forward = straight_test_2.Opt_test_moving_forward:main"
        ],
    },
)
