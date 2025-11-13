from setuptools import setup

package_name = 'drone_vision'

setup(
    name       = package_name,
    version    = '0.0.1',
    packages   = [package_name],
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
    ],
    install_requires = ['setuptools'],
    zip_safe      = True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description   = 'Pacote ROS 2 para detecção de linha azul e mangueira vermelha.',
    license='TODO: License declaration',
    tests_require = ['pytest'],
    entry_points  = {
        'console_scripts': ['line_detector = drone_vision.line_detector:main'],
    },
)
