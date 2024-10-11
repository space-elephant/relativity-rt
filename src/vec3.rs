use std::ops::*;

#[derive(Clone, Copy, Debug)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Vec3 {
    fn default() -> Self {
	Vec3::new(0.0, 0.0, 0.0)
    }
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
	Vec3 {x, y, z}
    }

    pub fn length_squared(self) -> f64 {
	self.dot(self)
    }

    pub fn length(self) -> f64 {
	self.length_squared().sqrt()
    }

    pub fn normalized(self) -> Self {
	self / self.length()
    }

    pub fn dot(self, other: Self) -> f64 {
	self.x*other.x + self.y*other.y + self.z*other.z
    }

    pub fn cross(self, other: Self) -> Self {
	Vec3::new(
	    self.y*other.z - self.z*other.y,
	    self.z*other.x - self.x*other.z,
	    self.x*other.y - self.y*other.x,
	)
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self {
	Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Add<Vec3> for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
	Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl AddAssign<Vec3> for Vec3 {
    fn add_assign(&mut self, other: Self) {
	*self = *self + other;
    }
}

impl Sub<Vec3> for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
	self + -other
    }
}

impl SubAssign<Vec3> for Vec3 {
    fn sub_assign(&mut self, other: Self) {
	*self = *self - other;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
	Vec3::new(self.x * other, self.y * other, self.z * other)
    }
}

impl MulAssign<f64> for Vec3 {
    fn mul_assign(&mut self, other: f64) {
	*self = *self * other;
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;

    fn mul(self, other: Vec3) -> Vec3 {
	other * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;

    fn div(self, other: f64) -> Self {
	self * (1.0 / other)
    }
}

impl DivAssign<f64> for Vec3 {
    fn div_assign(&mut self, other: f64) {
	*self = *self / other;
    }
}

pub type Point3 = Vec3;
pub type Colour3 = Vec3;
