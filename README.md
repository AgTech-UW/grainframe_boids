# grainframe_boids

Reynolds boids flocking in ROS 2 Humble, packaged two ways:

- **Centralized** one node owns all N boids in numpy arrays, publishes a single `PoseArray`. Ground-truth reference.
- **Decentralized** N independent nodes, each owning one boid, each publishing its own `Odometry` and subscribing to its neighbors over DDS. This is the architecture that maps onto a real fleet of robots.

Both versions share the same `boids_logic.py` math module. The decentralized version also spawns a `flock_viewer` node that dynamically discovers `/boid_*/odom` topics and republishes a unified `PoseArray` for RViz. To see the arrows, in RViz change Fixed Frame to `world` and add the topic boids/poses. The size of the arrows much be addjusted in RViz but every boid will be published under one topic:

## Quick start

Open in VS Code with the Dev Containers extension. "Reopen in Container" rebuilds the Humble environment.

Inside the container:

```bash
cd /workspaces/grainframe_boids
colcon build --packages-select boids --symlink-install
source install/setup.bash
```

### Decentralized (the main one)

```bash
ros2 launch boids boids.launch.py num_boids:=20
```

Spawns 20 boid nodes. 

On Linux running RViz from inside a container: first run `xhost +local:root` on your host, which lets the container draw windows on your display), then `export DISPLAY=:1` inside the container
# (tells GUI apps which display to use... :1 is the desktop screen, and :0 is welcome screen for modern Linux GUIs.

Open RViz in a second terminal.

```bash
rviz2
```

In RViz set Fixed Frame to `world`, Add -> By topic -> `/boids/poses` -> PoseArray.

While that's running, spawn an extra boid from another sourced terminal:

```bash
ros2 run boids single_boid --ros-args -p boid_id:=99 -p num_boids:=20
```

The viewer discovers it within ~1 second and it shows up in RViz.

### Centralized

```bash
ros2 run boids boids_node
```

Same RViz setup; the topic is also `/boids/poses`.

## Layout
