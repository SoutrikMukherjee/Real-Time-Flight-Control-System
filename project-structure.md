# RTFCS Project Structure

```
rtfcs/
├── README.md                      # Main project documentation
├── LICENSE                        # MIT License
├── CMakeLists.txt                # Main CMake build configuration
├── CONTRIBUTING.md               # Contribution guidelines
├── CHANGELOG.md                  # Version history
├── .gitignore                    # Git ignore rules
├── .gitlab-ci.yml               # CI/CD pipeline configuration
│
├── include/rtfcs/               # Public header files
│   ├── flight_controller.hpp    # Main flight controller interface
│   ├── types.hpp               # Common data types
│   ├── attitude_controller.hpp  # Attitude control algorithms
│   ├── position_controller.hpp  # Position control algorithms
│   ├── sensor_fusion.hpp       # Sensor fusion (EKF)
│   ├── safety_monitor.hpp      # Safety and redundancy
│   ├── mission_planner.hpp     # Autonomous mission execution
│   ├── telemetry.hpp          # Telemetry interface
│   ├── logger.hpp             # Logging system
│   ├── config_parser.hpp      # Configuration parsing
│   ├── hardware_interface.hpp  # Hardware abstraction layer
│   └── version.hpp            # Version information
│
├── src/                        # Implementation files
│   ├── main.cpp               # Main entry point
│   ├── flight_controller.cpp  # Flight controller implementation
│   ├── attitude_controller.cpp
│   ├── position_controller.cpp
│   ├── sensor_fusion.cpp
│   ├── safety_monitor.cpp
│   ├── mission_planner.cpp
│   ├── telemetry.cpp
│   ├── logger.cpp
│   ├── config_parser.cpp
│   ├── hardware_interface.cpp
│   │
│   ├── control/               # Control algorithms
│   │   ├── pid_controller.cpp
│   │   ├── adaptive_controller.cpp
│   │   ├── motor_mixer.cpp
│   │   └── trajectory_generator.cpp
│   │
│   ├── filters/               # Digital filters
│   │   ├── kalman_filter.cpp
│   │   ├── complementary_filter.cpp
│   │   ├── notch_filter.cpp
│   │   └── low_pass_filter.cpp
│   │
│   ├── navigation/            # Navigation algorithms
│   │   ├── waypoint_navigator.cpp
│   │   ├── obstacle_avoidance.cpp
│   │   ├── path_planner.cpp
│   │   └── geofence.cpp
│   │
│   └── sensors/               # Sensor drivers
│       ├── imu_driver.cpp
│       ├── gps_driver.cpp
│       ├── barometer_driver.cpp
│       ├── magnetometer_driver.cpp
│       └── rangefinder_driver.cpp
│
├── tests/                      # Unit and integration tests
│   ├── test_main.cpp
│   ├── test_flight_controller.cpp
│   ├── test_attitude_controller.cpp
│   ├── test_position_controller.cpp
│   ├── test_sensor_fusion.cpp
│   ├── test_safety_monitor.cpp
│   ├── test_filters.cpp
│   ├── test_navigation.cpp
│   ├── test_helpers.hpp
│   └── test_data/
│       ├── sensor_logs/
│       └── flight_logs/
│
├── config/                     # Configuration files
│   ├── quadcopter_default.yaml
│   ├── fixedwing_default.yaml
│   ├── hexacopter_default.yaml
│   ├── safety_limits.yaml
│   └── test_config.yaml
│
├── scripts/                    # Utility scripts
│   ├── deploy.sh              # Deployment script
│   ├── configure.py           # Configuration wizard
│   ├── pre-commit.sh          # Pre-commit hooks
│   ├── analyze_logs.py        # Log analysis tool
│   ├── calibrate_sensors.py   # Sensor calibration
│   └── generate_docs.sh       # Documentation generation
│
├── tools/                      # Development tools
│   ├── simulator/             # SITL simulator
│   │   ├── physics_engine.cpp
│   │   ├── sensor_simulator.cpp
│   │   └── visualization.cpp
│   │
│   ├── ground_station/        # Simple GCS
│   │   ├── main.py
│   │   ├── telemetry_viewer.py
│   │   └── mission_planner.py
│   │
│   └── log_analyzer/          # Flight log analysis
│       ├── parser.cpp
│       ├── analyzer.cpp
│       └── report_generator.cpp
│
├── docs/                       # Documentation
│   ├── Doxyfile.in           # Doxygen configuration
│   ├── architecture.md        # System architecture
│   ├── api_reference.md      # API documentation
│   ├── tuning_guide.md       # PID tuning guide
│   ├── safety_manual.md      # Safety procedures
│   ├── hardware_setup.md     # Hardware installation
│   └── images/               # Diagrams and images
│
├── third_party/               # External dependencies
│   ├── googletest/           # Google Test framework
│   ├── FreeRTOS/            # FreeRTOS kernel
│   └── mavlink/             # MAVLink protocol
│
├── missions/                  # Example missions
│   ├── survey_grid.mission
│   ├── waypoint_circle.mission
│   └── return_home_test.mission
│
└── hardware/                  # Hardware-specific files
    ├── schematics/           # PCB schematics
    ├── pin_mappings/         # GPIO configurations
    └── device_trees/         # Device tree overlays
```

## Key Components

### Core Systems
- **Flight Controller**: Central orchestrator managing all subsystems
- **Attitude Controller**: Stabilizes vehicle orientation using cascaded PID loops
- **Position Controller**: Maintains position and velocity using GPS/optical flow
- **Sensor Fusion**: Extended Kalman Filter combining multiple sensor inputs
- **Safety Monitor**: Redundancy management and failsafe logic

### Sensor Drivers
- **IMU**: High-rate inertial measurement (1000Hz)
- **GPS**: Position and velocity (25Hz with RTK support)
- **Barometer**: Altitude and climb rate (100Hz)
- **Magnetometer**: Heading reference (100Hz)
- **Rangefinder**: Precision altitude for landing

### Navigation Features
- **Mission Planner**: Autonomous waypoint navigation
- **Obstacle Avoidance**: Real-time path adjustment
- **Geofencing**: Virtual boundary enforcement
- **Precision Landing**: Vision-based landing

### Safety Features
- **Triple Redundancy**: Critical sensors and calculations
- **Watchdog Timers**: Hardware and software monitoring
- **Black Box**: Crash-survivable flight recorder
- **Emergency Procedures**: Automated failsafe responses

## Build Targets

```bash
# Main executable
rtfcs_flight_controller

# Test executable  
rtfcs_tests

# Documentation
make docs

# Installation package
make package
```

## Development Workflow

1. **Feature Development**
   - Create feature branch
   - Implement with tests
   - Run pre-commit checks
   - Submit pull request

2. **Testing**
   - Unit tests (Google Test)
   - Integration tests
   - Hardware-in-the-loop
   - Flight testing

3. **Documentation**
   - Doxygen API docs
   - User manuals
   - Safety procedures
   - Tuning guides

4. **Deployment**
   - Cross-compilation
   - Target deployment
   - Configuration
   - Verification