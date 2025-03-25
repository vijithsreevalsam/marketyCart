from setuptools import setup

package_name = 'cart_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amen',
    maintainer_email='AA5508@live.mdx.ac.uk',
    description='Supermarket navigation package with GUI',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = cart_navigation.gui:main',
        ],
    },
)