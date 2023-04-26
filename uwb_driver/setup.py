from setuptools import setup

package_name = 'uwb_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    
    install_requires=[
        "setuptools",
        "pyuwb @ git+https://bitbucket.org/decargroup/uwb_interface@main"
    ],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='charles.cossette@mail.mcgill.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["uwb_node = uwb_driver.uwb_node:main"]
            },
)
