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
	match TryInto::<[_; 1]>::try_into(objects) {
	    Ok([object]) => {
		BvhBuilder::Primitive(Box::new(object)),
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

		BvhBuilder::Split {
		    bbox: left.bbox + right.bbox,
		    axis,
		    left: Box::new(left),
		    right: Box::new(right),
		}
	    }
	}
    }

    fn sequential_length(&self) -> usize {
	match self {
	    BvhBuilder::Primitive(primitive) => 1,
	    BvhBuilder::Split{left, right, ..} => 1 + left.sequential_length() + right.sequential_length(),
	}
    }

    // returns the distance into the block that was written to
    unsafe fn into_copy_sequence(self, block: &mut [MaybeUninit<BvhNode>]) -> usize {
	match self {
	    BvhBuilder::Primitive(object) => {
		unsafe {
		    block[0].write(BvhNode::Primitive(object));
		}
		1
	    },
	    BvhBuilder::Split{bbox, axis, left, right} => {
		let (target, block) = block.split_at_mut(1);
		
		let split = left.into_copy_sequence(block);
		let end = left.into_copy_sequence(&mut block[split..]) + split;
		unsafe {
		    let rightchild = &block[split..end];
		    block[0].write(
			BvhNode::BvhBranch(
			    BvhBranch {
				bbox,
				axis,
				rightchild,
			    }
			)
		    );
		}
		end
	    },
	}
    }

    fn into_serialize(self) -> Bvh {
	todo!()
    }
}

pub struct Bvh(Box<[BvhNode]>, std::marker::PhantomPinned);

enum BvhNode {
    Primitive(Primitive),
    BvhBranch(BvhBranch),
}

struct BvhBranch {
    bbox: Boundingbox,
    rightchild: const* [BvhNode],
    axis: Axis,
}
