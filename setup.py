from setuptools import setup, find_packages

setup(
    name='mesh_semantic_mapping',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/mesh_semantic_mapping']),
        ('share/mesh_semantic_mapping', ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='vitor',
    author_email='vitorfonseca2205@gmail.com',
    maintainer='vitor',
    maintainer_email='vitorfonseca2205@gmail.com',
    url='https://github.com/todo/mesh_semantic_mapping',
    download_url='https://github.com/todo/mesh_semantic_mapping/releases',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Semantic mesh mapping package for robot perception and object detection',
    long_description='A ROS2 package for semantic mesh mapping using YOLO object detection and point cloud processing.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
