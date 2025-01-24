use crate::vec3::*;
use crate::colour::*;

// transforms from "stationary" reference frame to reference frame v
// direction must be normalized if it represents light
// v is less than unit length; c = 1
// x is the part of direction in the same direction as v
// that is to say, x is velocity, not position
pub fn lorentz(direction: &mut Vec3, colour: &mut Colour, v: Vec3) {
    if v.length_squared() < 1e-12 {
	// not moving at relativistic speed anyway, avoid division by zero
	return;
    }

    // generate standard variables
    let gamma = 1.0 / (1.0 - v.length_squared());
    let xv = direction.dot(v);
    let invtimedialation = 1.0 / (gamma * (1.0 + xv));

    let x = xv * v / v.length_squared();

    // replace motion in x with gamma(x+v)
    *direction += gamma * (x + v) - x;
    *direction *= invtimedialation;

    // wavelength, so time dialation means increasing the wavelength
    colour.mul_wavelength(invtimedialation);
}
