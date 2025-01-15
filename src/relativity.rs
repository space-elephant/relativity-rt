use crate::vec3::*;
use crate::colour::*;

// direction must be normalized if it represents light
// v is less than unit length; c = 1
// x is the part of direction in the same direction as v
pub fn lorentz(direction: &mut Vec3, colour: &mut Colour, v: Vec3) {
    let gamma = 1.0 / (1.0 - v.length_squared());
    let xv = direction.dot(v);
    let timedialation = gamma * (1.0 + xv);

    let x = xv * v / v.length_squared();

    // replace motion in x with gamma(x+v)
    *direction += gamma * (x + v) - x;
    *direction /= timedialation;

    // wavelength, so time dialation means increasing the wavelength
    colour.mul_wavelength(timedialation);
}
