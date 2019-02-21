## robot_primitives

A python library defining primitives commonly used in robotic applications (e.g. paths, areas, heuristics, etc.)

## Code Example

A minimal usage example to create a field and plot it.

```python
import robot_primitives as rp

domain = rp.areas.Domain.from_box_corners((0,0), (10,10))
```

## Installation

To install run:

```
python setup.py install
```
or 
```
python setup.py develop
```