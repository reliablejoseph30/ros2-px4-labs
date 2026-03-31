from setuptools import setup
import os
from glob import glob
package_name = 'assurance_harness'
setup(
name=package_name,
version='0.0.1',
packages=[package_name],
data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='your_name',
maintainer_email='your_email@example.com',
description='ROS 2 assurance harness: risk proxy + evidence logging + RViz marker',
license='Apache-2.0',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'telemetry_gate_node = assurance_harness.telemetry_gate_node:main',
'risk_model_node = assurance_harness.risk_model_node:main',
'evidence_logger_node = assurance_harness.evidence_logger_node:main',
'viz_node = assurance_harness.viz_node:main',
'fault_injector_node = assurance_harness.fault_injector_node:main',
],
},
)
