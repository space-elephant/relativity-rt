use crate::ray::*;
use crate::vec3::*;
use crate::material::*;
use crate::boundingbox::*;
use crate::colour::*;
use enum_dispatch::enum_dispatch;
use std::sync::Arc;

pub type Range = std::ops::Range<f64>;

pub fn in_range(t: f64, range: Range) -> Option<f64> {
    if range.contains(&t) {
	Some(t)
    } else {
	None
    }
}

pub struct Request {
    pub ray: Ray,
    pub colour: Colour,
    pub depth: u32,
}

#[derive(Clone, Debug)]
pub struct HitRecord {
    pub ray: Ray,
    pub point: Point3,
    pub normal: Vec3,
    pub t: f64,
    pub backface: bool,
    pub material: Arc<dyn Material>,
}

impl HitRecord {
    pub fn with_ray(point: Point3, mut normal: Vec3, t: f64, ray: Ray, material: Arc<dyn Material>) -> Self {
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

#[enum_dispatch]
pub trait Object {
    // target will be None initialized, may add to any parts but must attenuate apropriately
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord>;
    fn boundingbox(&self) -> Boundingbox;
}

#[derive(Debug)]
#[enum_dispatch(Object)]
pub enum Primitive {
    Sphere(Sphere),
    PlaneSeg(PlaneSeg),
    SmokeSphere(SmokeSphere),
}

#[derive(Debug)]
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
		    material: self.material.clone(),
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
		    material: self.material.clone(),
		});
	    }
	    None
	}
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }
}

#[derive(Debug)]
pub enum PlaneSegType {
    Triangle,
    Parallelogram,
    Ellipse,
}

#[derive(Debug)]
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
    // try to maximize angle at point 0
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

    pub fn new_triangle(material: Arc<dyn Material>, points: [Point3; 3]) -> PlaneSeg {
	Self::new(material, points[0], points[1] - points[0], points[2] - points[0], PlaneSegType::Triangle)
    }
}

impl Object for PlaneSeg {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let t = (self.height - self.normal.dot(ray.origin)) / self.normal.dot(ray.direction);
	if !range.contains(&t) {
	    // including if t is Infinity or NaN
	    return None;
	}

	let point = ray.at(t);
	let offset = point - self.origin;
	assert!(offset.dot(self.normal).abs() < 0.000001);
	let ufactor = self.w.dot(offset.cross(self.v));
	let vfactor = self.w.dot(self.u.cross(offset));

	/*if ufactor + vfactor > 1.0 {
	    println!("{ufactor}, {vfactor}");
	}*/

	// avoid lazy evaluation to reduce branches
	let offside = match self.planesegtype {
	    PlaneSegType::Triangle => (ufactor < 0.0) | (vfactor < 0.0) | (ufactor + vfactor > 1.0),
	    PlaneSegType::Parallelogram => (ufactor < 0.0) | (vfactor < 0.0) | (ufactor > 1.0) | (vfactor > 1.0),
	    PlaneSegType::Ellipse => ufactor.powi(2) + vfactor.powi(2) < 1.0,
	};
	if offside {
	    return None;
	}

	// should be negative
	let backface = self.normal.dot(ray.direction) > 0.0;
	Some(HitRecord {
	    ray,
	    point,
	    normal: if backface {-self.normal} else {self.normal},
	    t,
	    backface,
	    material: self.material.clone(),
	})
    }
    
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
}

#[derive(Debug)]
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
	    
	    let mut t1 = (-h - factor) / a;
	    if t1 >= range.end {
		return None;
	    }
	    if t1 < range.start {
		t1 = range.start;
	    }

	    let collision = t1 + self.neg_inv_density * rand::random::<f64>().log10();
	    
	    let t2 = (-h + factor) / a;
	    if collision <= t2 && range.contains(&collision) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(collision),
		    normal: (ray.at(t1) - self.center) / self.radius,
		    t: collision,
		    backface: false,
		    material: self.material.clone(),
		});
	    }
	    None
	}
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }
}
