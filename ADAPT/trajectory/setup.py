from setuptools import setup

package_name = 'adapt_trajp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh',
    maintainer_email='harshawardhan.patil@stud.hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'traj = adapt_trajp.trajectory_planner:main',
        'dum_path = adapt_trajp.dummy_path:main',
        'dum_state = adapt_trajp.dummy_state:main',
        'park = adapt_trajp.park:main'
        ],
    },
)
