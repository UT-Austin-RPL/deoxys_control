


# Eigen Usage

## Rotation Matrix
1. For a rotation matrix, we use `linear()` to extract the rotation
   matrix part. In Eigen library, `linear()` contains not only
   rotation information, but also reflection, scaling, shear
   components. However, since we are using homogeneous matrices for
   transformations, it will contain the rotation information. We do
   not use `rotation()` because it will have some internal
   computations to compute the rotational part, hence taking more time
   in compuation.
