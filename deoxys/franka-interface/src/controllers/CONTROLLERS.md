# Controllers

## General

- Transformation Matrices are represented in `Eigen::Affine3d`

- Orientations
  - Axis Angle:
  - Quaternion:
  - Euler Angles: Currently I don't consider Euler Angle Case.
  
- Vectors: `Eigen::VectorXd`

``` c++
	Eigen::VectorXd q(dim);
```

- Jacobian

### Franka Frames
1. `kJoint1`
1. `kJoint2`
1. `kJoint3`
1. `kJoint4`
1. `kJoint5`
1. `kJoint6`
1. `kJoint7`
1. `kFlange`
1. `kEndEffector`
1. `kStifness`



## Operational Space Controller



## Gripper Control


## Joint Positions
TODO



## Cartesian Positions
TODO
