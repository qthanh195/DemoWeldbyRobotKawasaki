# Robot Welding Camera Project

This project implements a robotic welding system that utilizes a camera for monitoring and control. The system is designed to automate the welding process, ensuring precision and efficiency.

## Project Structure

```
robot-welding-camera
├── src
│   ├── main.py                # Entry point of the application
│   ├── camera
│   │   └── camera_controller.py # Manages camera operations
│   ├── robot
│   │   └── robot_controller.py  # Controls robot movements and welding
│   ├── utils
│   │   └── helpers.py          # Utility functions for logging and calculations
│   └── config.py               # Configuration settings for camera and robot
├── requirements.txt            # Project dependencies
├── README.md                   # Project documentation
└── tests
    ├── test_camera.py          # Unit tests for camera functionalities
    └── test_robot.py           # Unit tests for robot functionalities
```

## Setup Instructions

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/robot-welding-camera.git
   cd robot-welding-camera
   ```

2. **Install dependencies:**
   It is recommended to use a virtual environment. You can create one using:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows use `venv\Scripts\activate`
   ```
   Then install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

To run the application, execute the following command:
```bash
python src/main.py
```

## Components

- **CameraController**: This class handles all camera-related operations, including initialization, image capture, and release of the camera.
- **RobotController**: This class manages the robotic movements and welding actions, providing methods to initialize the robot, move to specific positions, and start the welding process.
- **Helpers**: Utility functions that assist with logging messages and performing calculations, such as distance measurement.
- **Configuration**: Contains all necessary configuration settings for both the camera and the robot.

## Testing

Unit tests are provided for both the camera and robot functionalities. To run the tests, use:
```bash
pytest tests/
```

## License

This project is licensed under the MIT License. See the LICENSE file for more details.