camera_parameters = {
    'resolution': (1920, 1080),
    'fps': 30,
    'exposure': 0.1,
    'brightness': 0.5,
}

robot_settings = {
    'max_speed': 1.0,  # meters per second
    'welding_temperature': 1500,  # degrees Celsius
    'welding_duration': 5,  # seconds
}

def get_camera_parameters():
    return camera_parameters

def get_robot_settings():
    return robot_settings