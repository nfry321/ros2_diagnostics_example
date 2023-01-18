from setuptools import setup

package_name = 'diagnostics_example'

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
    maintainer='nick',
    maintainer_email='nick.fry@menapia.tech',
    description='Basic diagnostics publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diagnostics_example = diagnostics_example.diagnostics_example:main"
        ],
    },
)
