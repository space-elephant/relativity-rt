use std::rc::Rc;
use crate::ray::*;
use crate::vec3::*;
use crate::material::*;

type Range = std::ops::Range<f64>;

pub fn in_range(t: f64, range: Range) -> Option<f64> {
    if range.contains(&t) {
	Some(t)
    } else {
	None
    }
}

#[derive(Clone, Debug)]
pub struct HitRecord {
    pub ray: Ray,
    pub point: Point3,
    pub normal: Vec3,
    pub t: f64,
    pub backface: bool,
    pub material: Rc<dyn Material>,
}

impl HitRecord {
    pub fn with_ray(point: Point3, mut normal: Vec3, t: f64, ray: Ray, material: Rc<dyn Material>) -> Self {
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

pub trait Object {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord>;
}

pub struct Sphere {
    center: Point3,
    radius: f64,
    material: Rc<dyn Material>,
}

impl Sphere {
    pub fn new(material: Rc<dyn Material>, center: Point3, radius: f64) -> Sphere {
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
		})
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
		})
	    }
	    None
	}
    }
}

pub struct Group {
    objects: Vec<Box<dyn Object>>,
}

impl Group {
    pub fn new(objects: Vec<Box<dyn Object>>) -> Self {
	Group {
	    objects,
	}
    }
}

impl Object for Group {
    fn hit(&self, ray: Ray, mut range: Range) -> Option<HitRecord> {
	let mut best: Option<HitRecord> = None;
	for object in &self.objects {
	    if let Some(collision) = object.hit(ray, range.clone()) {
		range.end = collision.t;
		best = Some(collision);
	    }
	}
	best
    }
}
