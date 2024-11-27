Robot Properties Unitree B1
---------------------

### What it is

Collection of configuration files for the Unitree B1 robot.

### Installation

1. Install Pinocchio if you have not done so already.

   The simplest way to do so is using Conda:

   ```
   conda install -c conda-forge pinocchio
   ```

   Alternatively you can use any of the ways listed here: https://stack-of-tasks.github.io/pinocchio/download.html.

2. Install bullet_utils:

  ```
  git clone git@github.com:machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

3. Install robot_properties_b1:

### Examples

Below are a few examples. You find more in the `demos/` folder.

**Loading the B1 in PyBullet**

```python
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_b1.b1wrapper import B1Robot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(B1Robot)
```
