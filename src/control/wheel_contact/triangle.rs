//! Condition triangle queries without changing the track mesh. Clipping only
//! removes points outside the entire swept cylinder, with a safety margin.
use super::CONTACT_EPS;
use crate::math::{Isometry, Point, Real, Rotation, Vector};
use arrayvec::ArrayVec;
use parry::bounding_volume::BoundingVolume;
use parry::query::{details, ShapeCastOptions};
use parry::shape::{Cylinder, Shape, SupportMap, Triangle};

struct Patch {
    vertices: ArrayVec<Point<Real>, 16>,
    normal: Vector<Real>,
}

impl SupportMap for Patch {
    fn local_support_point(&self, direction: &Vector<Real>) -> Point<Real> {
        *self
            .vertices
            .iter()
            .max_by(|a, b| a.coords.dot(direction).total_cmp(&b.coords.dot(direction)))
            .unwrap()
    }
}

impl Patch {
    fn new(
        triangle: &Triangle,
        pose: &Isometry<Real>,
        start: &Isometry<Real>,
        direction: &Vector<Real>,
        distance: Real,
        cylinder: &Cylinder,
    ) -> Option<Self> {
        let local_start = Isometry::from_parts(Vector::zeros().into(), start.rotation);
        let mut local_end = local_start;
        local_end.translation.vector = direction * distance;
        let bounds = cylinder
            .compute_aabb(&local_start)
            .merged(&cylinder.compute_aabb(&local_end))
            .loosened(CONTACT_EPS * 10.0);

        // Use f64 for cancellation between large source vertices and the nearby
        // wheel origin. The actual convex query remains in the engine's Real type.
        let pose = pose.cast::<f64>();
        let origin = start.translation.vector.cast::<f64>();
        let mut vertices: ArrayVec<Point<f64>, 16> = [triangle.a, triangle.b, triangle.c]
            .into_iter()
            .map(|p| pose * p.cast::<f64>() - origin)
            .collect();
        let normal = (vertices[1] - vertices[0])
            .cross(&(vertices[2] - vertices[0]))
            .try_normalize(1.0e-12)?
            .cast::<Real>();
        for axis in 0..3 {
            for (sign, bound) in [
                (-1.0, -bounds.mins[axis] as f64),
                (1.0, bounds.maxs[axis] as f64),
            ] {
                let mut output = ArrayVec::new();
                let mut previous = *vertices.last()?;
                let mut previous_distance = sign * previous[axis] - bound;
                for current in &vertices {
                    let current_distance = sign * current[axis] - bound;
                    if (previous_distance <= 0.0) != (current_distance <= 0.0) {
                        output.push(
                            previous
                                + (current - previous)
                                    * (previous_distance / (previous_distance - current_distance)),
                        );
                    }
                    if current_distance <= 0.0 {
                        output.push(*current);
                    }
                    previous = *current;
                    previous_distance = current_distance;
                }
                vertices = output;
            }
        }
        if vertices.len() < 3 {
            return None;
        }
        // Clipping can leave repeated vertices at a box corner or a line along
        // its edge. Those have no face interior for the half-plane test below.
        let area: f64 = (1..vertices.len() - 1)
            .map(|i| {
                (vertices[i] - vertices[0])
                    .cross(&(vertices[i + 1] - vertices[0]))
                    .norm()
            })
            .sum();
        if area <= 1.0e-12 {
            return None;
        }
        Some(Self {
            vertices: vertices.into_iter().map(|p| p.cast::<Real>()).collect(),
            normal,
        })
    }

    fn contains(&self, point: &Point<Real>) -> bool {
        self.normal.dot(&(point - self.vertices[0])).abs() <= CONTACT_EPS
            && self
                .vertices
                .iter()
                .zip(self.vertices.iter().cycle().skip(1))
                .all(|(a, b)| {
                    let edge = b - a;
                    edge.cross(&(point - a)).dot(&self.normal) >= -CONTACT_EPS * edge.norm()
                })
    }

    fn valid_contact(
        &self,
        point: &Point<Real>,
        time: Real,
        rotation: &Rotation<Real>,
        direction: &Vector<Real>,
        cylinder: &Cylinder,
    ) -> bool {
        if !point.coords.iter().all(|x| x.is_finite()) || !self.contains(point) {
            return false;
        }
        let local = rotation.inverse() * (point.coords - direction * time);
        let radial_gap =
            ((local.x * local.x + local.z * local.z).sqrt() - cylinder.radius).max(0.0);
        let axial_gap = (local.y.abs() - cylinder.half_height).max(0.0);
        radial_gap * radial_gap + axial_gap * axial_gap <= CONTACT_EPS * CONTACT_EPS
    }
}

/// Returns travel from start, a world-space normal/point, and whether the face
/// solution was exact. Prefer exact faces over near-equal convex edge results.
pub(super) fn cast(
    triangle: &Triangle,
    pose: &Isometry<Real>,
    start: &Isometry<Real>,
    direction: &Vector<Real>,
    distance: Real,
    cylinder: &Cylinder,
) -> Option<(Real, Vector<Real>, Point<Real>, bool)> {
    let patch = Patch::new(triangle, pose, start, direction, distance, cylinder)?;
    let normal = if patch.normal.dot(direction) > 0.0 {
        -patch.normal
    } else {
        patch.normal
    };
    let axle = start.rotation * Vector::y();
    let axial = normal.dot(&axle);
    let radial = normal - axle * axial;
    let extent = cylinder.radius * radial.norm() + cylinder.half_height * axial.abs();
    let separation = -normal.dot(&patch.vertices[0].coords);
    let closing = -normal.dot(direction);

    // A cylinder cannot touch any part of a triangle before entering its plane
    // slab. Bound both casts and retries by this exact necessary condition.
    let (enter, exit) = if closing > 1.0e-6 {
        (
            ((separation - extent) / closing).max(0.0),
            ((separation + extent) / closing).min(distance),
        )
    } else if separation.abs() <= extent + CONTACT_EPS {
        (0.0, distance)
    } else {
        return None;
    };
    if enter > exit + CONTACT_EPS {
        return None;
    }
    let exit = exit.max(0.0);
    let enter = enter.min(exit);

    if closing >= super::MIN_SUPPORT_COS {
        let shoulder = if axial.abs() > 1.0e-3 {
            axial.signum()
        } else {
            0.0
        };
        let support = Point::from(
            direction * enter
                - radial
                    .try_normalize(CONTACT_EPS)
                    .unwrap_or_else(Vector::zeros)
                    * cylinder.radius
                - axle * (shoulder * cylinder.half_height),
        );
        let point = support - normal * normal.dot(&(support - patch.vertices[0]));
        if patch.contains(&point) {
            return Some((enter, normal, point + start.translation.vector, true));
        }
    }

    let position = Isometry::from_parts((direction * enter).into(), start.rotation);
    let options = ShapeCastOptions {
        max_time_of_impact: exit - enter,
        stop_at_penetration: true,
        compute_impact_geometry_on_penetration: true,
        target_distance: 0.0,
    };
    if let Some(hit) = details::cast_shapes_support_map_support_map(
        &position, direction, &patch, cylinder, options,
    ) {
        let time = enter + hit.time_of_impact;
        if time.is_finite()
            && time >= enter
            && time <= exit
            && patch.valid_contact(&hit.witness1, time, &start.rotation, direction, cylinder)
        {
            return Some((
                time,
                *hit.normal1,
                hit.witness1 + start.translation.vector,
                false,
            ));
        }
    }

    // A failed/invalid cast is not a suspension contact. Recompute it by bounded
    // conservative advancement on the small patch, starting at the plane bound.
    let mut time = enter;
    for _ in 0..16 {
        let position = Isometry::from_parts((direction * time).into(), start.rotation);
        let contact =
            details::contact_support_map_support_map(&position, &patch, cylinder, Real::MAX)?;
        if contact.dist <= CONTACT_EPS {
            return patch
                .valid_contact(&contact.point1, time, &start.rotation, direction, cylinder)
                .then_some((
                    time,
                    *contact.normal1,
                    contact.point1 + start.translation.vector,
                    false,
                ));
        }
        let closing = -contact.normal1.dot(direction);
        if closing <= 1.0e-6 || !contact.dist.is_finite() {
            return None;
        }
        time += (contact.dist / closing).max(CONTACT_EPS * 0.25);
        if time > exit {
            return None;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clipping_to_a_single_corner_cannot_create_a_face_contact() {
        let edge = 0.3 + CONTACT_EPS * 10.0;
        let triangle = Triangle::new(
            Point::new(edge, -0.4, edge),
            Point::new(edge + 1.0, -0.4, edge),
            Point::new(edge, -0.4, edge + 1.0),
        );
        assert!(cast(
            &triangle,
            &Isometry::identity(),
            &Isometry::identity(),
            &-Vector::y(),
            0.8,
            &Cylinder::new(0.1, 0.3)
        )
        .is_none());
    }
}
