# DOC_Problemas

To create object and access function:

    doc = doc_functions();
    doc.FUNCTION();

##  Direction Cosine Matrix

###  [C =  C_from_to(from, to)

This function computes a rotation matrix between two reference frames.

    input   from (3x3 array): base of origin    (base must be specified with column vectors)
    input   to   (3x3 array): base of destine

    output  C    (3x3 array): rotation matrix


###  [C =  C1(theta)

This function computes a rotation matrix for an angle 'theta' around first base axis (x).

    input   theta (float): angle of rotation

    output  C    (3x3 array): rotation matrix

###  [C =  C2(theta)

This function computes a rotation matrix for an angle 'theta' around second base axis (y).

    input   theta (float): angle of rotation

    output  C    (3x3 array): rotation matrix

###  [C =  C3(theta)

This function computes a rotation matrix for an angle 'theta' around third base axis (z).

    input   theta (float): angle of rotation

    output  C (3x3 array): rotation matrix


### C =  C323(t1, t2, t3)

This function computes a rotation matrix for a 323 sequence of rotation (z,y,z).

    input   t1 (float): angle of first rotation, around third axis (z)
    input   t2 (float): angle of second rotation, around second axis (y)
    input   t3 (float): angle of third rotation, around third axis (z)

    output  C  (3x3 array): rotation matrix

### C =  C123(t1, t2, t3)

This function computes a rotation matrix for a 123 sequence of rotation (x,y,z).

    input   t1 (float): angle of first rotation, around first axis (x)
    input   t2 (float): angle of second rotation, around second axis (y)
    input   t3 (float): angle of third rotation, around third axis (z)

    output  C  (3x3 array): rotation matrix

### theta1 =  theta1_321(C)

This function extract the angle rotated in the first rotation for a 321 rotation matrix (x,y,z).

    input   C (3x3 array): rotation matrix

    output  theta1  (float):  angle of rotation in the first rotation, around the third axis (z)

### theta2 =  theta2_321(C)
This function extract the angle rotated in the second rotation for a 321 rotation matrix (x,y,z).

    input   C (3x3 array): rotation matrix

    output  theta2  (float):  angle of rotation in the second rotation, around the second axis (y)

### theta3 =  theta3_321(C)
This function extract the angle rotated in the third rotation for a 321 rotation matrix (x,y,z).

    input   C (3x3 array): rotation matrix

    output  theta3  (float):  angle of rotation in the third rotation, around the first axis (x)

## Eigenaxis

### C =  C_from_e(phi, e)
Function to compute a DCM from Euler eigenaxis.

    input phi (float):     angle rotated around Euler eigenaxis
    input e   (3x1 array): axis representation components

    output C (3x3 array): rotation matrix


###  phi, e =  Eigenaxis(C21)
Function to compute Euler eigenaxis from a DCM.

    input C (3x3 array): rotation matrix

    output phi (float):     angle rotated around Euler eigenaxis
    output e   (3x1 array): axis representation components

## Quaternions
### q =  Quaternions_from_C(C)
Function to get the corresponding quaternion from DCM.

    input C   (3x3 array): rotation matrix

    output q (4x1 array) : quaternion [q1;q2;q3;q4]

### q =  Quaternions_from_e(phi, e)
Function to get the corresponding quaternion from Euler eigenaxis.

    input phi (float):     angle rotated around Euler eigenaxis
    input e   (3x1 array): axis representation components

    output q  (4x1 array): quaternion [q1;q2;q3;q4]


## Attitude determination
### Cb, Ci, Cbi =  Triad_Method(ub, vb, ui, vi)
Function to determine rotation matrix out of two vectors with two known representations in two reference frames (b stands for body and i for inertial but it does not mean anything about that).

    input ub (3x1 array): first vector in body frame
    input vb (3x1 array): second vector in body frame
    input ui (3x1 array): first vector in inertial frame
    input vi (3x1 array): second vector in inertial frame

    output Cb   (3x3 array): matrix with body triad
    output Ci   (3x3 array): matrix with inertial triad
    output Cbi  (3x3 array): DCM from TRIAD method

### I, C =  Principal_Inertia(J)
Get principal inertia matrix from an inertia matrix and the DCM to make the axis rotation from current to principal.

    input J (3x3 array): inertia matrix written in current frame

    output I (3x3 array): inertia matrix written in principal inertia axis
    output C (3x3 array): DCM from current to principal

### I =  Inertia_Matrix(I1, I2, I3)
Creates an inertia matrix out of its three eigenvalues

    input I1 (float): first eigenvalue
    input I2 (float): second eigenvalue
    input I3 (float): third eigenvalue

    output I (3x3 array): inertia matrix


### I =  Inertia_Cylinder(m, r, h)
Computes inertia matrix for a cylinder with mass, radius and height known.

    input m (float): total mass
    input r (float): radius
    input h (float): height

    output I (3x3 array): inertia matrix

### eq =  Euler_Equation(I, w, dw, T)
Returns Euler equation as a logical array (for future implementations in other functions).

    input I   (3x3 array): inertia matrix expressed in principal inertia axis
    input w   (3x1 array): rotation speed rate expressed in principal inertia axis
    input dw  (3x1 array): angular acceleration expressed in principal inertia axis
    input T   (3x1 array): external torque expressed in principal inertia axis

    output eq (3x1 logical array): Euler equation

Euler equation:

      T = I·dw + w x I·w

### T =  Kinetic_Energy(I, w)
Gets kinetic energy from a system in rotation.

    input I   (3x3 array): inertia matrix expressed in principal inertia axis
    input w   (3x1 array): rotation speed rate expressed in principal inertia axis

    output T  (float): kinetic energy

### h =  Angular_Momentum_Iw(I, w)
Gets angular momentum from a system in rotation.

    input I   (3x3 array): inertia matrix expressed in principal inertia axis
    input w   (3x1 array): rotation speed rate expressed in principal inertia axis

    output h  (3x1 array): angular momentum

### nutation_angle =  Nutation(h)
Gets nutation angle out of angular momentum

    input h  (3x1 array): angular momentum

    output nutation_angle (float): nutation angle

### precession_rate =  Precession_Rate(I, h)
Gets precession_rate out of angular momentum and inertia matrix (in principal inertia axis)

    input h   (3x1 array): angular momentum
    input I   (3x3 array): inertia matrix expressed in principal inertia axis

    output precesion_rate (float): precesion rate
