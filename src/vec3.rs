use std::ops::*;
use rand::Rng;

#[derive(Clone, Copy, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

// vector representing direction and length, where length can be any float
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
    // some functions copied from https://raytracing.github.io/books/RayTracingInOneWeekend.html
    pub fn grey(value: f64) -> Self {
	Colour3::new(value, value, value)
    }
    
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

    // reflect incidant ray direction about unit normal
    pub fn reflect(self, normal: Self) -> Self {
	self - 2.0 * self.dot(normal) * normal
    }

    // unlike reflect, takes normalized vector for self
    pub fn refract(self, normal: Self, rel_refractive_index: f64) -> Option<Self> {
	let cos = (-self.dot(normal)).min(1.0);
	let sinsq = 1.0 - cos*cos;
	if rel_refractive_index*rel_refractive_index * sinsq > 1.0 {
	    return None;
	}
	
	let perpendicular = rel_refractive_index * (self + cos * normal);
	// absolute value to protect against error causing slightly negative
	let parallel = -(1.0 - perpendicular.length_squared()).abs().sqrt() * normal;
	Some(perpendicular + parallel)
    }

    pub fn random() -> Self {
	let mut rng = rand::thread_rng();
	Vec3::new(rng.gen(), rng.gen(), rng.gen())
    }

    pub fn random_unit() -> Self {
	loop {
	    let vector = Self::random();
	    if vector.length_squared() <= 1.0 && vector.length_squared() > 1e-160 {
		return vector.normalized();
	    }
	}
    }

    pub fn random_on_hemisphere(normal: Vec3) -> Self {
	let result = Self::random_unit();
	if result.dot(normal) < 0.0 {
	    -result
	} else {
	    result
	}
    }

    pub fn add_gamma(self) -> Self {
	Colour3::new(
	    linear_add_gamma(self.x),
	    linear_add_gamma(self.y),
	    linear_add_gamma(self.z),
	)
    }

    pub fn near_zero(self) -> bool {
	const S: f64 = 1e-8;
	self.x.abs() < S && self.y.abs() < S && self.z.abs() < S
    }

    // find the greatest component, mainly used for exposure adjustment
    pub fn max_component(self) -> f64 {
	let xy = if self.x > self.y {self.x} else {self.y};
	if xy > self.z {xy} else {self.z}
    }
}

// add gamma for one channel, which is 1/2 in this case
fn linear_add_gamma(value: f64) -> f64 {
    if value < 0.0 {
	// clamp
	0.0
    } else {
	value.sqrt()
    }
}

// implementing numeric traits, again from ray tracing in one weekend
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

// allows the Axis enum to be used to choose an axis
impl Index<Axis> for Vec3 {
    type Output = f64;

    fn index(&self, index: Axis) -> &f64 {
	match index {
	    Axis::X => &self.x,
	    Axis::Y => &self.y,
	    Axis::Z => &self.z,
	}
    }
}

impl IndexMut<Axis> for Vec3 {
    fn index_mut(&mut self, index: Axis) -> &mut f64 {
	match index {
	    Axis::X => &mut self.x,
	    Axis::Y => &mut self.y,
	    Axis::Z => &mut self.z,
	}
    }
}

// type aliases
pub type Point3 = Vec3;
pub type Colour3 = Vec3;
