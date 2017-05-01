from setuptools import setup, find_packages

setup(
    name="fire_robot_driver",
    version="1.0",
    packages=['fire_robot_driver'] + find_packages('cv2', 'cv_kernel'),
    entry_points={
        'console_scripts': [
            'fire_robot_driver_run = fire_robot_driver.driver:run'
        ]
    }
)
