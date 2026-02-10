# ROS 2 Image Processing System

### Description
This repository contains a distributed ROS 2 system with two nodes: a Publisher node that streams live camera frames (or synthetic images if no camera is available) and a Subscriber node that receives them. The system performs real-time image processing by overlaying timestamps onto each frame and saving both the images and JSON metadata to a persistent local directory.

### Installation
1. Clone the repository:
```
git clone https://github.com/ardacihan/ros2_ws_demo.git
```
```
cd planblue_ros2_ws
```

2. Build the Docker image:

```
docker-compose build
```

### Execution

#### Option 1: Run everything in one terminal
```
docker-compose up
```

This will launch both the publisher and subscriber automatically.

#### Option 2: Run nodes in separate terminals

Terminal 1 (Publisher):
```
docker-compose run --name ros_dev ros2 bash
```
```
ros2 run image_processor image_publisher
```

Terminal 2 (Subscriber):
```
docker exec -it ros_dev bash
```
```
ros2 run image_processor image_listener
```

### Output
All results are saved to the `./output` folder:

- Images: ./output/images/
- Metadata: ./output/metafiles/

### .gitignore
Place this file in your project root to avoid committing unnecessary files:

