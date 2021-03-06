from setuptools import setup

package_name = 'gamepad_emulation'

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
    maintainer='root',
    maintainer_email='max.ehrhardt@hotmail.de',
    description='Emulate gamepad with keyboard inputs.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_emulation_node = gamepad_emulation.gamepad_emulation:main'
        ],
    },
)
