# By @diegoomataix: https://github.com/diegoomataix/ADCS_PyLibrary.git

import numpy as np
from sympy.physics.mechanics import ReferenceFrame, dot, cross
from sympy import *
init_printing(use_latex='mathjax')


################################################################################
#%% Non-spherical Earth Harmonics Functions:
################################################################################

def non_spherical_pot(n, m, subs=False, Jnmval=None, lamb_nm_val=None, h_orbit=None):
    """
    Returns the Potential and radial, North-South and East-Weest accelerations 
    based on the potential formulation for the gravity field of the Earth.
    
    Input:
        - n(int): degree of the spherical harmonic
        - m(int): order of the spherical harmonic
        - subs(bool): if True, substitute the following values for symbols, acc units in m/s^2. 
            - R_e: 6.371e6
            - G: 6.6743e-11
            - M: 5.972167867791379e+24
            - Jnm: Jnmval
            - lamb_nm: lamb_nm_val
            - h_orbit: h_orbit (input as orbital altitude, not radius)
    Output:
        - Pn: polynomial for degree n
        - Pnm: polynomial for degree n, order m (@ x= sin(phi) )
        - Unm(sym): potential formulation for the spherical harmonic
        - a_r, a_phi, a_lamb: radial, North-South and East-Weest accelerations
    """
    # Define symbols:
    from sympy.abc import x, G, M, r
    phi, R_e, Jnm, lamb, lamb_nm = symbols(
        'phi R_e J_{nm} lambda \lambda_{nm}')

    # Calculate the polynomial and the potential:
    Pnderiv = (1-x**2)**n
    Pn = (1/((-2)**n*np.math.factorial(n)) * (Pnderiv.diff((x, n)))).simplify()
    Pnm = ((1-x**2)**(m/2)*(Pn.diff((x, m)))).subs(x, sin(phi)).simplify()

    # Calculate the potential
    Unm = G*M/r * (R_e/r)**n * Jnm * Pnm * cos(m*(lamb - lamb_nm))

    # Calculate the accelerations:
    a_r = -diff(Unm, r)
    a_phi = - 1/r * diff(Unm, phi)
    a_lamb = - 1/(r*cos(phi)) * diff(Unm, lamb)

    if subs is True:
        a_r = a_r.subs(R_e, 6.371e6).subs(
            G, 6.6743e-11).subs(r, h_orbit+6.371e6).subs(M, 5.972167867791379e+24).subs(Jnm, Jnmval).subs(lamb_nm, lamb_nm_val).simplify()
        a_phi = a_phi.subs(R_e, 6.371e6).subs(
            G, 6.6743e-11).subs(r, h_orbit+6.371e6).subs(M, 5.972167867791379e+24).subs(Jnm, Jnmval).subs(lamb_nm, lamb_nm_val).simplify()
        a_lamb = a_lamb.subs(R_e, 6.371e6).subs(
            G, 6.6743e-11).subs(r, h_orbit+6.371e6).subs(M, 5.972167867791379e+24).subs(Jnm, Jnmval).subs(lamb_nm, lamb_nm_val).simplify()

    return Pn, Pnm, Unm, a_r, a_phi, a_lamb
