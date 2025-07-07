from setuptools import find_packages, setup

package_name = 'py_pkg'

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
    maintainer='Nihiro Tamura',
    maintainer_email='tamura.nihiro@irl.sys.es.osaka-u.ac.jp',
    description='a package for research',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_pub=py_pkg.pid_pub:main',
            'pid_sub=py_pkg.pid_sub:main',
            'pid_stop_pub=py_pkg.pid_stop_pub:main',
            'pid_stop_lpf_pub=py_pkg.pid_stop_lpf_pub:main',
            'pid_lpf_pub=py_pkg.pid_lpf_pub:main',
            'pid_test=py_pkg.pid_test:main',
            'pid_stop_lpf_pub_optimize=py_pkg.pid_stop_lpf_pub_optimize:main',
            'pot_desired_pub=py_pkg.pot_desired_pub:main',
            'pot_desired_float_pub=py_pkg.pot_desired_float_pub:main',
            'pot_desired_conti_pub=py_pkg.pot_desired_conti_pub:main',
            'pot_desired_conti_float_pub=py_pkg.pot_desired_conti_float_pub:main',
            'pot_desired_conti_linear_float_pub=py_pkg.pot_desired_conti_linear_float_pub:main',
            'pot_desired_conti_enter_float_pub=py_pkg.pot_desired_conti_enter_float_pub:main',
            'pot_realized_conti_pub_test=py_pkg.pot_realized_conti_pub_test:main',
            'pot_realized_pub_test=py_pkg.pot_realized_pub_test:main',
            'write_data_csv=py_pkg.write_data_csv:main',
            'write_data_Ballistic_csv=py_pkg.write_data_Ballistic_csv:main',
            'pub_sub_test=py_pkg.pub_sub_test:main',
            'optimize_pid=py_pkg.optimize_pid:main',
            'optimize_test=py_pkg.optimize_test:main',
            'optimize_ROS2=py_pkg.optimize_ROS2:main',
            'optimize_ROS2_float=py_pkg.optimize_ROS2_float:main',
            'optimize_ADRC_ROS2_float=py_pkg.optimize_ADRC_ROS2_float:main',
            'optimize_Ballistic_ROS2=py_pkg.optimize_Ballistic_ROS2:main'
        ],
    },
)
