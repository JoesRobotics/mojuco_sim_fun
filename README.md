# README.md

# Fun Robotics Simulations with MuJoCo

Welcome! This project contains three fun robotics simulation tasks using [MuJoCo](https://mujoco.org/):
- **Franka FR3** drawing a picture of itself
- **Skydio X2 Drone** flying through three gates
- **Boston Dynamics Spot** performing a playful dance

Each robot gets its own simple environment and control script, organized to be easy to run and extend.

## Project Previews

### Franka FR3 Drawing
![FR3 Drawing](media/fr3_drawing.gif)

The Franka FR3 robot arm holds a marker and draws a simple sketch of itself on an easel!

### Skydio Drone Gate Navigation
![Skydio Drone](media/skydio_gate.gif)

The Skydio X2 drone autonomously flies through three colorful circular gates.

### Spot Robot Dance
![Spot Dance](media/spot_dance.gif)

Boston Dynamics Spot shows off a fun little dance routine (head bobs, spins, paw wave).

## How to Run

1. **Install MuJoCo 2.3+**
   ```bash
   pip install mujoco
   ```

2. **Install additional Python dependencies**
   ```bash
   pip install numpy
   ```

3. **Clone this repository**
   ```bash
   git clone https://github.com/yourname/robotics-fun-simulations.git
   cd robotics-fun-simulations
   ```

4. **Navigate to any project folder and run**
   ```bash
   cd fr3_drawing
   python fr3_drawing.py
   ```
   or
   ```bash
   cd skydio_gate
   python skydio_gate.py
   ```
   or
   ```bash
   cd spot_dance
   python spot_dance.py
   ```

## Project Structure

```bash
robotics-fun-simulations/
├── fr3_drawing/
│   ├── fr3_drawing.py
│   └── fr3_drawing_scene.xml
├── skydio_gate/
│   ├── skydio_gate.py
│   └── gate_world.xml
├── spot_dance/
│   ├── spot_dance.py
│   └── spot_dance_scene.xml
├── media/
│   ├── fr3_drawing.gif
│   ├── skydio_gate.gif
│   └── spot_dance.gif
└── README.md
```

## Notes
- The robot models are simplified for demonstration purposes.
- For best performance, use a machine with a decent GPU for rendering MuJoCo scenes.
- You can record new GIFs using screen recorders or directly through MuJoCo frame capture.

## License
This project is provided for educational and demonstration purposes. Enjoy simulating and modifying!

---

⭐ If you like this project, feel free to fork, extend, and share your own robot simulations!
