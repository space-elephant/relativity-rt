use crate::{Ray, Point3, Vec3};
use std::cmp::{min, max};
use std::ops::*;

#[derive(Debug, Clone, Copy)]
pub struct Boundingbox {
    min: Point3,
    max: Point3,
}

impl Default for Boundingbox {
    fn default() -> Self {
	Boundingbox {
	    min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
	    max: Point3::new(-f64::INFINITY, -f64::INFINITY, -f64::INFINITY),
	}
    }
}

impl Boundingbox {
    pub fn new(min: Point3, max: Point3) -> Self {
	Boundingbox {
	    min,
	    max,
	}
    }

    pub fn from_point(point: Point3) -> Self {
	Self::new(point, point)
    }
    
    pub fn centeroid(self) -> Point3 {
	0.5 * (self.min + self.max)
    }
    
    pub fn isempty(self) -> bool {
	self.min.x >= self.max.x || self.min.y >= self.max.y || self.min.z >= self.max.z
    }

    pub fn extent(self) -> Vec3 {
	self.max - self.min
    }

    pub fn intersect_ray(self, ray: Ray) -> Range<f64> {
	let a = ray.direction.x;
	let b = ray.origin.x;
	let slabl = (self.min.x - b) / a;
	let slabh = (self.max.x - b) / a;
	let xslab = if a > 0.0 {slabl..slabh} else {slabh..slabl};
	
	let a = ray.direction.y;
	let b = ray.origin.y;
	let slabl = (self.min.y - b) / a;
	let slabh = (self.max.y - b) / a;
	let yslab = if a > 0.0 {slabl..slabh} else {slabh..slabl};
	
	let a = ray.direction.z;
	let b = ray.origin.z;
	let slabl = (self.min.z - b) / a;
	let slabh = (self.max.z - b) / a;
	let zslab = if a > 0.0 {slabl..slabh} else {slabh..slabl};

	xslab.start.min(yslab.start).min(zslab.start) .. xslab.end.min(yslab.end).min(zslab.end)
    }
}

impl Add for Boundingbox {
    type Output = Self;

    fn add(self, other: Self) -> Self {
	let minx = self.min.x.min(other.min.x);
	let miny = self.min.y.min(other.min.y);
	let minz = self.min.z.min(other.min.z);

	let maxx = self.max.x.max(other.max.x);
	let maxy = self.max.y.max(other.max.y);
	let maxz = self.max.z.max(other.max.z);

	Boundingbox::new(
	    Point3::new(minx, miny, minz),
	    Point3::new(maxx, maxy, maxz), 
	)
    }
}

impl AddAssign<Boundingbox> for Boundingbox {
    fn add_assign(&mut self, other: Self) {
	*self = *self + other;
    }
}

impl Mul for Boundingbox {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
	let minx = self.min.x.max(other.min.x);
	let miny = self.min.y.max(other.min.y);
	let minz = self.min.z.max(other.min.z);

	let maxx = self.max.x.min(other.max.x);
	let maxy = self.max.y.min(other.max.y);
	let maxz = self.max.z.min(other.max.z);

	Boundingbox::new(
	    Point3::new(minx, miny, minz),
	    Point3::new(maxx, maxy, maxz), 
	)
    }
}

impl MulAssign<Boundingbox> for Boundingbox {
    fn mul_assign(&mut self, other: Self) {
	*self = *self * other;
    }
}
