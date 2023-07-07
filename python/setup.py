from setuptools import setup

package_name = 'rclpy_pkg_template'

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
    maintainer='tony',
    maintainer_email='hoony3355@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_image_sub = rclpy_pkg_template.rgb_image_subscriber:main',
            'rgbd_image_sub = rclpy_pkg_template.rgbd_image_subscriber:main'
        ],
    },
)
