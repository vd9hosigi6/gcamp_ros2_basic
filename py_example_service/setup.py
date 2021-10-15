from setuptools import setup

package_name = 'py_example_service'

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
    maintainer='hp',
    maintainer_email='vd9hosigi6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'example_node = py_example_service.example_node:main',
                'example_client = py_example_service.example_client:main',
                'example_server = py_example_service.example_server:main',
        ],
    },
)
