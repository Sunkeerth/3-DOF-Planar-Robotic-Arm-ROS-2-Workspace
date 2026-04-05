# setup.py – tells Python and ROS 2 how to build and install the 'planar_arm_kinematics' package.
# This file is required for any Python-based ROS 2 package that contains nodes.

# Import the helper functions from setuptools.
# find_packages: automatically discovers all Python packages (directories with __init__.py) inside your source.
# setup: the main function that describes your package to the build system.
from setuptools import find_packages, setup

# The name of the package (must match the folder name and package.xml).
package_name = 'planar_arm_kinematics'

# Call setup() with all the metadata and instructions.
setup(
    # Name of the package (used when you run 'colcon build --packages-select ...').
    name=package_name,

    # Version number (increment when you make major changes).
    version='0.0.1',

    # Automatically find all Python sub‑packages inside the src folder.
    # Exclude the 'test' folder because it contains unit tests, not runtime code.
    packages=find_packages(exclude=['test']),

    # Data files that are not Python source code but need to be installed.
    # These are usually resource files, configuration files, or package.xml.
    data_files=[
        # Tell the ROS 2 resource index that this package exists.
        # The ament_index (resource index) uses this to locate your package.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),   # This empty file marks the package.

        # Copy the package.xml file to the share directory (so rosdep and other tools can find it).
        ('share/' + package_name, ['package.xml']),
    ],

    # List of Python dependencies required to run this package.
    # 'setuptools' is always needed for building.
    install_requires=['setuptools'],

    # If True, the installed package can be loaded from a zip file (usually safe).
    zip_safe=True,

    # Your name (as the package maintainer).
    maintainer='suneerth',

    # Your email address (users can contact you with questions).
    maintainer_email='suneerth@todo.todo',

    # Short description of what the package does (appears in 'ros2 pkg list').
    description='FK and IK nodes for 3-DOF planar arm',

    # Open‑source license (Apache 2.0 is common for ROS 2 packages).
    license='Apache-2.0',

    # Entry points: where to find executable scripts.
    # These become system commands that you can run from the terminal after building and sourcing.
    entry_points={
        'console_scripts': [
            # Format: 'command_name = python_module_path:function_name'
            # After installation, typing 'fk_solver' in a terminal runs planar_arm_kinematics/fk_solver.py's main().
            'fk_solver   = planar_arm_kinematics.fk_solver:main',
            # Similarly, 'ik_solver' runs the ik_solver node.
            'ik_solver   = planar_arm_kinematics.ik_solver:main',
            # 'test_ik_client' runs the test client.
            'test_ik_client = planar_arm_kinematics.test_ik_client:main',
        ],
    },
)