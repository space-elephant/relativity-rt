use crate::vec3::*;
use crate::objects::*;
use crate::boundingbox::*;
use crate::ray::*;
use std::cmp::Ordering;
use std::mem::{transmute, MaybeUninit};

// used for surface area heuristic
const BUCKETS: usize = 8;

// whichever axis has the largest range of centroids, as in pbrt
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

// the simplest algorithm, not currently used
// split in the middle of the range
fn choose_center_middle(objects: &[Primitive], axis: Axis) -> f64 {
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

// pbrt-style surface area heuristic algorithm
// shared surface area aproximately equally among the two sides
fn choose_center_sah(objects: &[Primitive], axis: Axis) -> f64 {
    let mut min = f64::INFINITY;
    let mut max = -f64::INFINITY;

    // find the range of objects, and the spaces of each bucket
    for object in objects {
	let value = object.boundingbox().centeroid()[axis];

	if value < min {
	    min = value;
	}
	if value > max {
	    max = value;
	}
    }
    let bucketsize = (max - min) / BUCKETS as f64;

    let mut surfaceareas: [f64; BUCKETS] = [0.0; BUCKETS];

    for object in objects {
	// clamp for when it exactly hits the border
	let bucket = (((object.boundingbox().centeroid()[axis] - min) / bucketsize) as usize).min(BUCKETS-1);
	surfaceareas[bucket] += object.surfacearea();
    }

    // now choose whichever cutoff is the closest to even (lowest magnitude of error)
    // error is half of (the surface areas of objects on the right minus those on the left)
    let mut error: f64 = surfaceareas.iter().sum::<f64>() * 0.5;
    for bucket in 1..BUCKETS {
	let prev = error;
	error -= surfaceareas[bucket-1];
	if error <= 0.0 {
	    // if going further will make it worse
	    let bucket = if error < -prev && bucket != 1 {
		bucket - 1
	    } else {
		bucket
	    };
	    return min + bucket as f64 * bucketsize
	}
    }
    // if we get this far, the majority of objects are in the final bucket, so we isolate only them
    min + (BUCKETS - 1) as f64 * bucketsize
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
		// extract single primitive, and put it on the heap
		BvhBuilder::Primitive(Box::new(object))
	    },
	    Err(objects) => {
		// choose an axis and split around it
		let axis = choose_axis(&objects);
		let center = choose_center_sah(&objects, axis);

		// allocate double necessary space, ensuring no reallocations
		let mut left_objects = Vec::with_capacity(objects.len());
		let mut right_objects = Vec::with_capacity(objects.len());
		for object in objects {
		    match object.boundingbox().centeroid()[axis].partial_cmp(&center) {
			Some(Ordering::Less) => left_objects.push(object),
			Some(Ordering::Greater) => right_objects.push(object),
			// in case two objects share a centroid
			// e.g. outer surface of glass + inner surface of same + contained smoke
			_ => if left_objects.len() <= right_objects.len() {
			    left_objects.push(object);
			} else {
			    right_objects.push(object);
			}
		    }
		}

		// recursively generate the tree and save in the union
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

    // boundingbox can be calculated like an Object
    fn boundingbox(&self) -> Boundingbox {
	match self {
	    BvhBuilder::Primitive(object) => object.boundingbox(),
	    BvhBuilder::Split{bbox, ..} => *bbox,
	}
    }

    // used to create the sequential version: measured in size_of::<BvhNode>()
    fn sequential_length(&self) -> usize {
	match self {
	    BvhBuilder::Primitive(_primitive) => 1,
	    BvhBuilder::Split{left, right, ..} => 1 + left.sequential_length() + right.sequential_length(),
	}
    }

    // copy into slice of BvhNodes, as would be used to find an object
    // returns the distance into the block that was written to
    // all values before that will be initialized
    // recursive
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

    // uses previous recursive function to generate a Bvh
    // this function removes the MaybeUninit
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

// it has pointers into itself, so it cannot be moved, hence PhantomPinned
pub struct Bvh(Box<[BvhNode]>, std::marker::PhantomPinned);

// delegates everything to the BvhNode objects (except generation which is done by a BvhBuilder)
impl Object for Bvh {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	BvhNode::hit(&*self.0, ray, range)
    }
    
    fn boundingbox(&self) -> Boundingbox {
	self.0[0].boundingbox()
    }

    fn surfacearea(&self) -> f64 {
	BvhNode::surfacearea(&*self.0)
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

// nodes are only used in slices, so that getting the next value is safe
// more unsafe rust is probably the idiomatic solution
#[derive(Debug)]
enum BvhNode {
    Primitive(Primitive),
    BvhBranch(BvhBranch),
}

impl BvhNode {
    // nonrecursive functions like this don't need slices, but everything else does
    fn boundingbox(&self) -> Boundingbox {
	match self {
	    Self::Primitive(object) => object.boundingbox(),
	    Self::BvhBranch(branch) => branch.bbox,
	}
    }

    // display, for debugging
    // not used in current state
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

    // it enumerates the tree, collecting all the surface areas of primitives
    // also not used, since surface area is needed to build bvh, not once it exists
    fn surfacearea<'a>(bvh: &'a [Self]) -> f64 {
	match &bvh[0] {
	    Self::Primitive(object) => object.surfacearea(),
	    Self::BvhBranch(branch) => {
		let rightchild = unsafe {
		    transmute::<*const [BvhNode], &'a [BvhNode]>(branch.rightchild)
		};
		Self::surfacearea(&bvh[1..]) + Self::surfacearea(rightchild)
	    },
	}
    }

    // traverse the tree, finding what it hits
    // everything is immutable
    fn hit<'a>(bvh: &'a [Self], ray: Ray, mut range: Range) -> Option<HitRecord> {
	match &bvh[0] {
	    // base case
	    Self::Primitive(object) => object.hit(ray, range),
	    
	    Self::BvhBranch(branch) => {
		// will get into the downstream object before detecting it being out of range
		let inrange = branch.bbox.intersect_ray(ray);
		if inrange.is_empty() {
		    return None;
		}

		// rightchild is stored here, leftchild is &bvh[1..]
		let rightchild = unsafe {
		    transmute::<*const [BvhNode], &'a [BvhNode]>(branch.rightchild)
		};

		if ray.direction[branch.axis] > 0.0 {// left child, with lower values, first
		    match Self::hit(&bvh[1..], ray, range.clone()) {
			Some(rec) => {
			    // but they can intersect, so it is possible that we hit the other one first
			    range.end = rec.t;
			    Self::hit(rightchild, ray, range).or(Some(rec))
			},
			// we dont hit the first one, so check the second
			None => Self::hit(rightchild, ray, range),
		    }
		} else {
		    // same thing, but it's rightchild first
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

// a plain struct used to store the splitting of the nodes
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
