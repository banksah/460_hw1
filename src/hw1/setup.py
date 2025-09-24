from setuptools import find_packages, setup

package_name = 'hw1'

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
    maintainer='Andrew Banks',
    maintainer_email='ahbanks@crimson.ua.edu',
    description='Simple package using turtlesim to draw a circle',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle = hw1.circle:main',
	    'rectangle = hw1.rectangle:main',
	    'diamond = hw1.diamond:main',
	    'random = hw1.random:main',
        ],
    },
)
