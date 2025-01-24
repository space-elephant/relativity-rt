use crate::vec3::*;

// everything here from https://raytracing.github.io/books/RayTracingInOneWeekend.html
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Point3,
    pub direction: Vec3,
}

// function of time, which should be positive but its length is arbitrary
// gives the location of a point on the ray
impl Ray {
    pub fn new(origin: Point3, direction: Vec3) -> Self {
	Ray {
	    origin,
	    direction,
	}
    }

    pub fn at(&self, t: f64) -> Point3 {
	self.origin + self.direction * t
    }
}
