use crate::ray::*;
use crate::vec3::*;
use crate::material::*;
use crate::boundingbox::*;
use crate::colour::*;
use enum_dispatch::enum_dispatch;
use std::sync::Arc;
use std::f64::consts::PI;

// use the native rust type Range as an interval
pub type Range = std::ops::Range<f64>;

// request, stored on my custom stack
// contains the ray to be sent, as well as the attenuation and TTL (depth is actually backwards)
pub struct Request {
    pub ray: Ray,
    pub colour: Colour,
    pub depth: u32,
}

// the struct, from ray tracing in one weekend, about reporting what happened to a ray that has already hit
// the material reference counter does not count another reference; that is held by the object still
#[derive(Clone, Debug)]
pub struct HitRecord<'a> {
    pub ray: Ray,
    pub point: Point3,
    pub normal: Vec3,
    pub t: f64,
    pub backface: bool,
    pub material: &'a dyn Material,
}

// ensure the normal is always pointing outwards. It will already be normalized
impl<'a> HitRecord<'a> {
    pub fn with_ray(point: Point3, mut normal: Vec3, t: f64, ray: Ray, material: &'a dyn Material) -> Self {
	let backface = ray.direction.dot(normal) > 0.0;
	if backface {
	    normal = -normal
	}
	HitRecord {
	    ray,
	    point,
	    normal,
	    t,
	    backface,
	    material,
	}
    }
}

// Object is the "hittable" from rtweekend. Provides surface area for bvh construction
#[enum_dispatch]
pub trait Object {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord>;
    fn boundingbox(&self) -> Boundingbox;
    fn surfacearea(&self) -> f64;
}

// bvh can't do this, but primitives can: translate (as used for buildings in the example scene)
#[enum_dispatch]
pub trait Transform: Object {
    fn translate(&mut self, offset: Vec3);
}

pub fn translate_many(objects: &mut [impl Transform], offset: Vec3) {
    for object in objects {
	object.translate(offset);
    }
}

// rust enums are tagged unions. This one stores different types of primitives as a faster alternative to vtables (also, rust's dynamic dispatch requires that the objects are stored elsewhere, such as the heap)
#[derive(Debug, Clone)]
#[enum_dispatch(Object, Transform)]
pub enum Primitive {
    Sphere(Sphere),
    PlaneSeg(PlaneSeg),
    SmokeSphere(SmokeSphere),
}

// the simplest and first object, only for solid material (sphere, not a ball)
#[derive(Debug, Clone)]
pub struct Sphere {
    center: Point3,
    radius: f64,
    material: Arc<dyn Material>,
}

impl Sphere {
    pub fn new(material: Arc<dyn Material>, center: Point3, radius: f64) -> Sphere {
	Sphere {
	    center,
	    radius,
	    material,
	}
    }
}

impl Object for Sphere {
    // from ray tracing in one weekend, uses the quadratic formula, trying the exit intercept if the entry fails
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let offset = ray.origin - self.center;
	let a = ray.direction.length_squared();
	let h = ray.direction.dot(offset);
	let c = offset.length_squared() - self.radius*self.radius;
	let discriminant = h*h - a*c;
	if discriminant < 0.0 {
	    None
	} else {
	    // least intersection is the important one
	    let factor = discriminant.sqrt();
	    
	    let t = (-h - factor) / a;
	    if range.contains(&t) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(t),
		    normal: (ray.at(t) - self.center) / self.radius,
		    t,
		    backface: false,
		    material: &*self.material,
		});
	    }
	    
	    let t = (-h + factor) / a;
	    if range.contains(&t) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(t),
		    normal: -(ray.at(t) - self.center) / self.radius,
		    t,
		    backface: true,
		    material: &*self.material,
		});
	    }
	    None
	}
    }

    // creates axis aligned bounding box, for bvh
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }

    // standard formula
    fn surfacearea(&self) -> f64 {
	4.0 * PI * self.radius.powi(2)
    }
}

// based around one point, so just shift that
impl Transform for Sphere {
    fn translate(&mut self, offset: Vec3) {
	self.center += offset;
    }
}

// plane segments of any kind that can be generated with two vectors and an origin
#[derive(Debug, Clone)]
pub enum PlaneSegType {
    Triangle,
    Parallelogram,
    Ellipse,
}

#[derive(Debug, Clone)]
pub struct PlaneSeg {
    normal: Vec3,
    height: f64,// -D in standard form
    origin: Point3,
    w: Vec3,
    u: Vec3,
    v: Vec3,
    material: Arc<dyn Material>,
    planesegtype: PlaneSegType,
}

impl PlaneSeg {
    // generate with only what is given
    pub fn new(material: Arc<dyn Material>, origin: Point3, u: Vec3, v: Vec3, planesegtype: PlaneSegType) -> PlaneSeg {
	let n = u.cross(v);
	let normal = n.normalized();
	let height = normal.dot(origin);
	let w = n / n.length_squared();
	
	PlaneSeg {
	    normal,
	    height,
	    origin,
	    w,
	    u,
	    v,
	    material,
	    planesegtype,
	}
    }

    // make a triangle from a list of points, for conveniance
    pub fn new_triangle(material: Arc<dyn Material>, points: [Point3; 3]) -> PlaneSeg {
	Self::new(material, points[0], points[1] - points[0], points[2] - points[0], PlaneSegType::Triangle)
    }
}

impl Object for PlaneSeg {
    // the hit function, adapted from ray tracing the next week
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let t = (self.height - self.normal.dot(ray.origin)) / self.normal.dot(ray.direction);
	if !range.contains(&t) {
	    // including if t is Infinity or NaN
	    return None;
	}

	// we hit the plane, so determine if we hit the plane segment
	let point = ray.at(t);
	let offset = point - self.origin;
	let ufactor = self.w.dot(offset.cross(self.v));
	let vfactor = self.w.dot(self.u.cross(offset));

	// avoid lazy evaluation to reduce branches
	let offside = match self.planesegtype {
	    PlaneSegType::Triangle => (ufactor < 0.0) | (vfactor < 0.0) | (ufactor + vfactor > 1.0),
	    PlaneSegType::Parallelogram => (ufactor < 0.0) | (vfactor < 0.0) | (ufactor > 1.0) | (vfactor > 1.0),
	    PlaneSegType::Ellipse => ufactor.powi(2) + vfactor.powi(2) < 1.0,
	};
	if offside {
	    return None;
	}

	// dot product should be negative
	let backface = self.normal.dot(ray.direction) > 0.0;
	Some(HitRecord {
	    ray,
	    point,
	    normal: if backface {-self.normal} else {self.normal},
	    t,
	    backface,
	    material: &*self.material,
	})
    }

    // find the endpoints (if a polygon) and take the aabb that combines them all
    fn boundingbox(&self) -> Boundingbox {
	match self.planesegtype {
	    PlaneSegType::Triangle => {
		let mut result = Default::default();
		for offset in [Default::default(), self.u, self.v] {
		    result += Boundingbox::from_point(self.origin + offset);
		}
		result
	    },
	    PlaneSegType::Parallelogram => {
		let mut result = Default::default();
		for offset in [Default::default(), self.u, self.v, self.u + self.v] {
		    result += Boundingbox::from_point(self.origin + offset);
		}
		result
	    },
	    PlaneSegType::Ellipse => {
		// a bit larger than necessary: use the rectange
		let mut result = Default::default();
		for offset in [-self.u-self.v, self.u-self.v, -self.u+self.v, self.u+self.v] {
		    result += Boundingbox::from_point(self.origin + offset);
		}
		result
	    },
	}
    }

    // represents the surface of only one side
    fn surfacearea(&self) -> f64 {
	match self.planesegtype {
	    PlaneSegType::Triangle => self.u.cross(self.v).length() * 0.5,
	    PlaneSegType::Parallelogram => self.u.cross(self.v).length(),
	    PlaneSegType::Ellipse => self.u.cross(self.v).length() * PI,
	}
    }
}

// some redundant data, but the plane will be parallel
impl Transform for PlaneSeg {
    fn translate(&mut self, offset: Vec3) {
	self.origin += offset;
	self.height = self.normal.dot(self.origin);
    }
}

// this one is an actual ball, need to ensure collision is before the exit point
#[derive(Debug, Clone)]
pub struct SmokeSphere {
    center: Point3,
    radius: f64,
    material: Arc<dyn Material>,
    neg_inv_density: f64,
}

impl SmokeSphere {
    pub fn new(colour: ReflectionSpectrum, density: f64, center: Point3, radius: f64) -> SmokeSphere {
	SmokeSphere {
	    center,
	    radius,
	    material: Arc::new(Isotropic::new(colour)),
	    neg_inv_density: -1.0 / density,
	}
    }
}

impl Object for SmokeSphere {
    // randomly generate a distance, and then check if in range
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let offset = ray.origin - self.center;
	let a = ray.direction.length_squared();
	let h = ray.direction.dot(offset);
	let c = offset.length_squared() - self.radius*self.radius;
	let discriminant = h*h - a*c;
	if discriminant < 0.0 {
	    None
	} else {
	    let factor = discriminant.sqrt();

	    let mut t1 = (-h - factor) / a;
	    if t1 >= range.end {
		return None;
	    }
	    if t1 < range.start {
		t1 = range.start;
	    }

	    let collision = t1 + self.neg_inv_density * rand::random::<f64>().log10();

	    // the material will create the direction, but the position is done here
	    let t2 = (-h + factor) / a;
	    if collision <= t2 && range.contains(&collision) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(collision),
		    normal: (ray.at(t1) - self.center) / self.radius,
		    t: collision,
		    backface: false,
		    material: &*self.material,
		});
	    }
	    None
	}
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }

    fn surfacearea(&self) -> f64 {
	// still take the surface of the shell
	4.0 * PI * self.radius.powi(2)
    }
}

impl Transform for SmokeSphere {
    fn translate(&mut self, offset: Vec3) {
	self.center += offset;
    }
}
