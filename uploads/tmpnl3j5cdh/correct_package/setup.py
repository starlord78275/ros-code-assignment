from setuptools import setup

package_name = 'pick_and_place_ur5'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Intern',
    maintainer_email='intern@mavenai.com',
    description='Pick and place package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_and_place = pick_and_place_ur5.pick_and_place:main'
        ],
    },
)
