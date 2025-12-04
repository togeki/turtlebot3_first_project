from setuptools import find_packages, setup

package_name = 'tb3_first_py'

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
    maintainer='tgq',
    maintainer_email='tgq@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_move = tb3_first_py.square_move:main',
            'avoid_move = tb3_first_py.avoid_move:main',
            'simple_avoid = tb3_first_py.simple_avoid:main',
        ],
    },
)
