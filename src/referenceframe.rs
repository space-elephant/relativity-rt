use crate::vec3::*;
use crate::objects::*;
use crate::boundingbox::*;
use crate::Ray::*;

use std::mem::{align_of, size_of, transmute};

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

enum BvhBuilderChildren {
    Primitive(Box<Primitive>),
    Split{
	axis: Axis,
	left: Box<BvhBuilder>,
	right: Box<BvhBuilder>,
    },
}

pub struct BvhBuilder {
    bbox: Boundingbox,
    children: BvhBuilderChildren,
}

impl BvhBuilder {
    fn new(objects: Vec<Primitive>) -> BvhBuilder {
	match TryInto::<[_; 1]>::try_into(objects) {
	    Ok([object]) => {
		BvhBuilder {
		    bbox: object.boundingbox(),
		    children: BvhBuilderChildren::Primitive(Box::new(object)),
		}
	    },
	    Err(objects) => {
		let axis = choose_axis(&objects);
		let center = choose_center(&objects, axis);

		let mut left_objects = Vec::with_capacity(objects.len());
		let mut right_objects = Vec::with_capacity(objects.len());
		for object in objects {
		    if object.boundingbox().centeroid()[axis] < center {
			left_objects.push(object);
		    } else {
			right_objects.push(object);
		    }
		}

		let left = BvhBuilder::new(left_objects);
		let right = BvhBuilder::new(right_objects);

		BvhBuilder {
		    bbox: left.bbox + right.bbox,
		    children: BvhBuilderChildren::Split {
			axis,
			left: Box::new(left),
			right: Box::new(right),
		    },
		}
	    }
	}
    }
}

enum BvhRef<'a> {
    Primitive(&'a Primitive),
    BvhBranch(&'a BvhBranch),
}

const OFFSET_PRIMITIVE: usize = size_of::<Primitive>() + align_of::<Primitive>();
const OFFSET_BRANCH: usize = size_of::<BvhBranch>() + align_of::<BvhBranch>();

// my reference points to bool. If true, succeeded (after alignment) by Primitive; otherwise BvhBranch
// is considered to own everything after itself
pub struct Bvh(bool, std::marker::PhantomPinned);

impl Bvh {
    fn collect<'a> (&'a self) -> BvhRef<'a> {
	unsafe {
	    // required for the construction of this object
	    // add moves by bytes
	    if self.0 {
		BvhRef::Primitive(
		    transmute::<*const Bvh, &'a Primitive> (
			(self as *const Bvh)
			    .add(align_of::<Primitive>())
		    )
		)
	    } else {
		BvhRef::BvhBranch(
		    transmute::<*const Bvh, &'a BvhBranch> (
			(self as *const Bvh)
			    .add(align_of::<BvhBranch>())
		    )
		)
	    }
	}
    }
}

impl Object for Bvh {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	todo!();
    }

    fn boundingbox(&self) -> Boundingbox {
	match self.collect() {
	    BvhRef::Primitive(primitive) {
		primitive.boundingbox()
	    },
	    BvhRef::BvhBranch(branch) {
		branch.bbox
	    }
	}
    }
}

struct BvhBranch {
    bbox: Boundingbox,
    rightchild: *const Bvh,
    axis: Axis,
}
