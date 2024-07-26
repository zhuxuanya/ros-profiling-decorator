# ros-time-profiler

This ROS node is dedicated to profiling the execution time of functions, offering detailed insights through profiling and timing decorators. It helps in evaluating the performance of specific functions under real-time conditions.

## Installation

Navigate to the `src` directory of the catkin workspace (e.g., `~/catkin_ws`):

```bash
cd ~/catkin_ws/src
```

Clone the repository into the `src` directory and rename it to `time_profiler`:

```bash
git clone https://github.com/zhuxuanya/ros-time-profiler.git time_profiler
```

Return to the catkin workspace root and rebuild:

```bash
cd ..
catkin_make
```

Source the environment to update the workspace:

```bash
source devel/setup.bash
```

## Usage

Before using the decorators, make sure the `debug` parameter in launch file is set to `True` to enable detailed profiling and timing logs. Apply the decorator to the function as follows:

- **Profiling**:

```python
from time_profiler.profiler import *

@profiling_decorator
def function():
    # code
```

- **Timing**:

```python
from time_profiler.profiler import *

@timing_decorator
def function():
    # code
```

Run ROS node as usual, and observe the logs for detailed execution metrics.
