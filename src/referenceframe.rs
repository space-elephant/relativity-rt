use crate::vec3::*;
use crate::objects::*;
use crate::boundingbox::*;
use crate::ray::*;
use std::cmp::Ordering;

use std::mem::{transmute, MaybeUninit};

fn choose_axis(objects: &[Primitive]) -> Axis {
    let mut range = Boundingbox::default();
    for object in objects {
	range += Boundingbox::from_point(object.boundingbox().centeroid());
    }
    let extent = range.extent();

    if extent.x > extent.y {
	if extent.x > extent.z {Axis::X} else {Axis::Z}
    } else {
	if extent.y > extent.z {Axis::Y} else {Axis::Z}
    }
}

fn choose_center(objects: &[Primitive], axis: Axis) -> f64 {
    let mut min = f64::INFINITY;
    let mut max = -f64::INFINITY;

    for object in objects {
	let value = object.boundingbox().centeroid()[axis];

	if value < min {
	    min = value;
	}
	if value > max {
	    max = value;
	}
    }

    return (min + max) * 0.5;
}

// consists of at least one primitive
#[derive(Debug)]
pub enum BvhBuilder {
    Primitive(Box<Primitive>),
    Split{
	bbox: Boundingbox,
	axis: Axis,
	left: Box<BvhBuilder>,
	right: Box<BvhBuilder>,
    },
}

impl BvhBuilder {
    pub fn new(objects: Vec<Primitive>) -> BvhBuilder {
	assert!(objects.len() != 0);
	match TryInto::<[_; 1]>::try_into(objects) {
	    Ok([object]) => {
		BvhBuilder::Primitive(Box::new(object))
	    },
	    Err(objects) => {
		let axis = choose_axis(&objects);
		let center = choose_center(&objects, axis);

		let mut left_objects = Vec::with_capacity(objects.len());
		let mut right_objects = Vec::with_capacity(objects.len());
		for object in objects {
		    match object.boundingbox().centeroid()[axis].partial_cmp(&center) {
			Some(Ordering::Less) => left_objects.push(object),
			Some(Ordering::Greater) => right_objects.push(object),
			_ => if left_objects.len() <= right_objects.len() {
			    left_objects.push(object);
			} else {
			    right_objects.push(object);
			}
		    }
		}
		
		let left = BvhBuilder::new(left_objects);
		let right = BvhBuilder::new(right_objects);

		BvhBuilder::Split {
		    bbox: left.boundingbox() + right.boundingbox(),
		    axis,
		    left: Box::new(left),
		    right: Box::new(right),
		}
	    }
	}
    }

    fn boundingbox(&self) -> Boundingbox {
	match self {
	    BvhBuilder::Primitive(object) => object.boundingbox(),
	    BvhBuilder::Split{bbox, ..} => *bbox,
	}
    }

    fn sequential_length(&self) -> usize {
	match self {
	    BvhBuilder::Primitive(_primitive) => 1,
	    BvhBuilder::Split{left, right, ..} => 1 + left.sequential_length() + right.sequential_length(),
	}
    }

    // returns the distance into the block that was written to
    unsafe fn into_copy_sequence(self, block: &mut [MaybeUninit<BvhNode>]) -> usize {
	match self {
	    BvhBuilder::Primitive(object) => {
		block[0].write(BvhNode::Primitive(*object));
		1
	    },
	    BvhBuilder::Split{bbox, axis, left, right} => {
		let (target, rest) = block.split_at_mut(1);
		
		let split = left.into_copy_sequence(rest);
		let end = right.into_copy_sequence(&mut rest[split..]) + split;
		// rightchild is now fully initialized by the second call
		let rightchild = &rest[split..end];
		target[0].write(
		    BvhNode::BvhBranch(
			BvhBranch {
			    bbox,
			    axis,
			    rightchild: unsafe {
				transmute::<&[MaybeUninit<BvhNode>], *const [BvhNode]>(rightchild)
			    },
			}
		    )
		);
		end + 1
	    },
	}
    }

    fn into_serialize(self) -> Bvh {
	let mut target: Box<[MaybeUninit<BvhNode>]> =
	    std::iter::repeat_with(MaybeUninit::uninit)
            .take(self.sequential_length())
            .collect();
	unsafe {
	    self.into_copy_sequence(&mut *target);
	    Bvh(
		transmute::<Box<[MaybeUninit<BvhNode>]>, Box<[BvhNode]>>(target),
		std::marker::PhantomPinned
	    )
	}
    }
}

pub struct Bvh(Box<[BvhNode]>, std::marker::PhantomPinned);

impl Object for Bvh {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	BvhNode::hit(&*self.0, ray, range)
    }
    
    fn boundingbox(&self) -> Boundingbox {
	self.0[0].boundingbox()
    }
}

impl Bvh {
    pub fn new(objects: Vec<Primitive>) -> Bvh {
	BvhBuilder::new(objects).into_serialize()
    }

    pub fn display(&self) {
	BvhNode::display(&self.0);
    }
}

#[derive(Debug)]
enum BvhNode {
    Primitive(Primitive),
    BvhBranch(BvhBranch),
}

impl BvhNode {
    fn boundingbox(&self) -> Boundingbox {
	match self {
	    Self::Primitive(object) => object.boundingbox(),
	    Self::BvhBranch(branch) => branch.bbox,
	}
    }

    fn display<'a>(bvh: &'a [Self]) {
	match &bvh[0] {
	    Self::Primitive(_object) => println!("primitive"),
	    Self::BvhBranch(branch) => {
		println!("branch: ");
		Self::display(&bvh[1..]);
		println!(", ");
		let rightchild = unsafe {
		    transmute::<*const [BvhNode], &'a [BvhNode]>(branch.rightchild)
		};
		Self::display(rightchild);
		println!("end");
	    },
	}
    }
    
    fn hit<'a>(bvh: &'a [Self], ray: Ray, mut range: Range) -> Option<HitRecord> {
	match &bvh[0] {
	    Self::Primitive(object) => object.hit(ray, range),
	    Self::BvhBranch(branch) => {
		let inrange = branch.bbox.intersect_ray(ray);
		if inrange.is_empty() {
		    return None;
		}

		let rightchild = unsafe {
		    transmute::<*const [BvhNode], &'a [BvhNode]>(branch.rightchild)
		};

		if ray.direction[branch.axis] > 0.0 {// left child, with lower values, first
		    match Self::hit(&bvh[1..], ray, range.clone()) {
			Some(rec) => {
			    range.end = rec.t;
			    Self::hit(rightchild, ray, range).or(Some(rec))
			},
			None => Self::hit(rightchild, ray, range),
		    }
		} else {
		    match Self::hit(rightchild, ray, range.clone()) {
			Some(rec) => {
			    range.end = rec.t;
			    Self::hit(&bvh[1..], ray, range).or(Some(rec))
			},
			None => Self::hit(&bvh[1..], ray, range),
		    }
		}
	    },
	}
    }
}

// rightchild points back into the Bvh, somewhere upwards of self
#[derive(Debug)]
struct BvhBranch {
    bbox: Boundingbox,
    rightchild: *const [BvhNode],
    axis: Axis,
}

// needed for the *const [BvhNode], which is known to have no mutable access so it's fine
unsafe impl Sync for BvhBranch {}
unsafe impl Send for BvhBranch {}
