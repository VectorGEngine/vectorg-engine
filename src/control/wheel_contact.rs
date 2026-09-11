//! Suspension-volume queries. Each wheel selects one support; candidates never add loads.
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet};
use crate::math::{Isometry, Point, Real, Rotation, Vector};
use crate::pipeline::{QueryFilter, QueryPipeline};
use parry::bounding_volume::{Aabb, BoundingVolume};
use parry::query::visitors::BoundingVolumeIntersectionsVisitor;
use parry::query::ShapeCastOptions;
use parry::shape::{Cylinder, Shape, TypedShape};

pub(super) const MIN_SUPPORT_COS: Real = 0.1;
const CONTACT_EPS: Real = 1.0e-4;

#[derive(Clone, Copy, Debug)]
pub(super) struct WheelSupport {
    pub collider: ColliderHandle,
    pub length: Real,
    pub point: Point<Real>,
    pub normal: Vector<Real>,
}

pub(super) struct WheelSweep {
    pub mount: Point<Real>,
    pub direction: Vector<Real>,
    pub axle: Vector<Real>,
    pub radius: Real,
    pub width: Real,
    pub min_length: Real,
    pub max_length: Real,
}

impl WheelSweep {
    fn planar_support(
        &self,
        shape: &dyn Shape,
        pose: &Isometry<Real>,
    ) -> Option<(Real, Vector<Real>, Point<Real>)> {
        let mut best: Option<(Real, Vector<Real>, Point<Real>)> = None;
        let mut face = |normal: Vector<Real>, plane: Point<Real>, halfspace: bool| {
            let incidence = -normal.dot(&self.direction);
            if incidence < MIN_SUPPORT_COS {
                return;
            }
            let axial = normal.dot(&self.axle);
            let radial = normal - self.axle * axial;
            let radius_normal = self.radius * radial.norm() + self.width * 0.5 * axial.abs();
            let length = (normal.dot(&(self.mount - plane)) - radius_normal) / incidence;
            if length > self.max_length + CONTACT_EPS
                || (!halfspace && length < self.min_length - CONTACT_EPS)
            {
                return;
            }
            let length = length.clamp(self.min_length, self.max_length);
            let center = self.mount + self.direction * length;
            let offset = -radial
                .try_normalize(CONTACT_EPS)
                .unwrap_or_else(Vector::zeros)
                * self.radius
                - self.axle
                    * (if axial.abs() > 1.0e-3 {
                        axial.signum()
                    } else {
                        0.0
                    })
                    * self.width
                    * 0.5;
            let support = center + offset;
            let point = support - normal * normal.dot(&(support - plane));
            // The finite face must actually contain the tire support. Otherwise
            // its edge/corner is handled by the convex shape cast below.
            let on_face = match shape.as_typed_shape() {
                TypedShape::Triangle(triangle) => {
                    // Barycentric projection loses precision on large triangles,
                    // especially at their shared diagonal. Test edge half-planes.
                    let point = pose.inverse_transform_point(&point);
                    let normal = triangle.normal().unwrap();
                    [
                        (triangle.a, triangle.b),
                        (triangle.b, triangle.c),
                        (triangle.c, triangle.a),
                    ]
                    .into_iter()
                    .all(|(a, b)| {
                        let edge = b - a;
                        edge.cross(&(point - a)).dot(&normal) >= -CONTACT_EPS * edge.norm()
                    })
                }
                _ => {
                    halfspace
                        || (shape.project_point(pose, &point, false).point - point).norm_squared()
                            <= CONTACT_EPS * CONTACT_EPS
                }
            };
            if !on_face {
                return;
            }
            if best.as_ref().map_or(true, |old| length < old.0) {
                best = Some((length, normal, point));
            }
        };
        match shape.as_typed_shape() {
            TypedShape::HalfSpace(plane) => face(
                pose * *plane.normal,
                Point::from(pose.translation.vector),
                true,
            ),
            TypedShape::Cuboid(cuboid) => {
                for axis in 0..3 {
                    for sign in [-1.0, 1.0] {
                        let n = Vector::ith(axis, sign);
                        face(
                            pose * n,
                            pose * Point::from(n * cuboid.half_extents[axis]),
                            false,
                        );
                    }
                }
            }
            TypedShape::Triangle(triangle) => {
                if let Some(normal) = triangle.normal() {
                    let mut normal = pose * *normal;
                    if normal.dot(&self.direction) > 0.0 {
                        normal = -normal;
                    }
                    face(normal, pose * triangle.a, false);
                }
            }
            _ => {}
        }
        best
    }

    pub fn cast(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        filter: QueryFilter,
        chassis: RigidBodyHandle,
    ) -> (Option<WheelSupport>, Option<Point<Real>>) {
        if !self.width.is_finite()
            || self.width <= 0.0
            || !self.radius.is_finite()
            || self.radius <= 0.0
            || self.max_length < self.min_length
        {
            return (None, None);
        }
        let cylinder = Cylinder::new(self.width * 0.5, self.radius);
        let rotation = Rotation::rotation_between(&Vector::y(), &self.axle)
            .unwrap_or_else(|| Rotation::new(Vector::x() * std::f64::consts::PI as Real));
        let start = Isometry::from_parts(
            (self.mount + self.direction * self.min_length)
                .coords
                .into(),
            rotation,
        );
        let distance = self.max_length - self.min_length;
        let mut end = start;
        end.translation.vector += self.direction * distance;
        let bounds = cylinder
            .compute_aabb(&start)
            .merged(&cylinder.compute_aabb(&end))
            .loosened(CONTACT_EPS);
        let mut best: Option<WheelSupport> = None;
        let mut rejected = None;
        queries.colliders_with_aabb_intersecting_aabb(&bounds, |handle| {
            let collider = &colliders[*handle];
            if collider.parent() != Some(chassis) && filter.test(bodies, *handle, collider) {
                visit_parts(
                    collider.shape(),
                    collider.position(),
                    &bounds,
                    &mut |shape, pose| {
                        let (length, normal, mut point) =
                            if let Some(hit) = self.planar_support(shape, pose) {
                                hit
                            } else {
                                let relative_pose = pose.inv_mul(&start);
                                let velocity = pose.inverse_transform_vector(&self.direction);
                                let options = ShapeCastOptions {
                                    max_time_of_impact: distance,
                                    stop_at_penetration: true,
                                    compute_impact_geometry_on_penetration: true,
                                    target_distance: 0.0,
                                };
                                let Ok(Some(hit)) = queries.query_dispatcher().cast_shapes(
                                    &relative_pose,
                                    &velocity,
                                    shape,
                                    &cylinder,
                                    options,
                                ) else {
                                    return;
                                };
                                (
                                    self.min_length + hit.time_of_impact,
                                    pose * *hit.normal1,
                                    pose * hit.witness1,
                                )
                            };
                        let incidence = -normal.dot(&self.direction);
                        if !incidence.is_finite() || incidence < MIN_SUPPORT_COS {
                            rejected = Some(point);
                            return;
                        }
                        let center = self.mount + self.direction * length;
                        // Choose the middle of a flat contact patch where possible. GJK can
                        // return either cylinder shoulder for the same flat road.
                        let axial = normal.dot(&self.axle);
                        let radial = normal - self.axle * axial;
                        if let Some(radial) = radial.try_normalize(CONTACT_EPS) {
                            let shoulder = if axial.abs() > 1.0e-3 {
                                axial.signum()
                            } else {
                                0.0
                            };
                            let support = center
                                - radial * self.radius
                                - self.axle * (shoulder * self.width * 0.5);
                            let on_plane = support - normal * normal.dot(&(support - point));
                            let projection = shape.project_point(pose, &on_plane, false);
                            if (projection.point - on_plane).norm_squared()
                                <= CONTACT_EPS * CONTACT_EPS
                            {
                                point = on_plane;
                            }
                        }
                        if !length.is_finite() || !point.coords.iter().all(|x| x.is_finite()) {
                            return;
                        }
                        let candidate = WheelSupport {
                            collider: *handle,
                            length,
                            point,
                            normal,
                        };
                        let replace = best.as_ref().map_or(true, |old| {
                            length < old.length - CONTACT_EPS
                                || ((length - old.length).abs() <= CONTACT_EPS
                                    && incidence > -old.normal.dot(&self.direction) + CONTACT_EPS)
                        });
                        if replace {
                            best = Some(candidate);
                        }
                    },
                );
            }
            true
        });
        (best, rejected)
    }
}

// Broad-phase bounds restrict work to the wheel's travel volume. Visit mesh
// triangles separately so a rejected triangle cannot mask another in that mesh.
fn visit_parts(
    shape: &dyn Shape,
    pose: &Isometry<Real>,
    bounds: &Aabb,
    visit: &mut impl FnMut(&dyn Shape, &Isometry<Real>),
) {
    let local_bounds = bounds.transform_by(&pose.inverse());
    match shape.as_typed_shape() {
        TypedShape::TriMesh(mesh) => {
            mesh.qbvh()
                .traverse_depth_first(&mut BoundingVolumeIntersectionsVisitor::new(
                    &local_bounds,
                    &mut |id: &u32| {
                        visit(&mesh.triangle(*id), pose);
                        true
                    },
                ));
        }
        TypedShape::Compound(compound) => {
            compound
                .qbvh()
                .traverse_depth_first(&mut BoundingVolumeIntersectionsVisitor::new(
                    &local_bounds,
                    &mut |id: &u32| {
                        let (part_pose, part) = &compound.shapes()[*id as usize];
                        visit_parts(part.as_ref(), &(pose * part_pose), bounds, visit);
                        true
                    },
                ));
        }
        TypedShape::HeightField(field) => field
            .map_elements_in_local_aabb(&local_bounds, &mut |_, triangle| visit(triangle, pose)),
        _ => visit(shape, pose),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::RigidBodyBuilder;
    use crate::geometry::ColliderBuilder;

    fn sample(x: Real, mesh: bool) -> WheelSupport {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(RigidBodyBuilder::dynamic());
        let mut colliders = ColliderSet::new();
        if mesh {
            colliders.insert(
                ColliderBuilder::trimesh(
                    vec![
                        Point::new(-2.0, 0.0, -2.0),
                        Point::new(0.0, 0.0, -2.0),
                        Point::new(0.0, 0.0, 2.0),
                        Point::new(-2.0, 0.0, 2.0),
                        Point::new(0.0, 0.1, -2.0),
                        Point::new(2.0, 0.1, -2.0),
                        Point::new(2.0, 0.1, 2.0),
                        Point::new(0.0, 0.1, 2.0),
                    ],
                    vec![
                        [0, 2, 1],
                        [0, 3, 2],
                        [1, 2, 7],
                        [1, 7, 4],
                        [4, 7, 6],
                        [4, 6, 5],
                    ],
                )
                .unwrap(),
            );
        } else {
            colliders.insert(
                ColliderBuilder::cuboid(2.0, 0.1, 2.0).translation(Vector::new(0.0, -0.1, 0.0)),
            );
            colliders.insert(
                ColliderBuilder::cuboid(1.0, 0.05, 2.0).translation(Vector::new(1.0, 0.05, 0.0)),
            );
        }
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        WheelSweep {
            mount: Point::new(x, 0.7, 0.0),
            direction: -Vector::y(),
            axle: Vector::z(),
            radius: 0.3,
            width: 0.2,
            min_length: 0.0,
            max_length: 0.6,
        }
        .cast(
            &bodies,
            &colliders,
            &queries,
            QueryFilter::default(),
            chassis,
        )
        .0
        .unwrap()
    }

    #[test]
    fn large_flat_faces_keep_exact_normals_and_centered_contact_patches() {
        for mesh in [false, true] {
            let mut bodies = RigidBodySet::new();
            let chassis = bodies.insert(RigidBodyBuilder::dynamic());
            let mut colliders = ColliderSet::new();
            if mesh {
                colliders.insert(
                    ColliderBuilder::trimesh(
                        vec![
                            Point::new(-10000.0, 0.0, -10000.0),
                            Point::new(10000.0, 0.0, -10000.0),
                            Point::new(10000.0, 0.0, 10000.0),
                            Point::new(-10000.0, 0.0, 10000.0),
                        ],
                        vec![[0, 2, 1], [0, 3, 2]],
                    )
                    .unwrap(),
                );
            } else {
                colliders.insert(
                    ColliderBuilder::cuboid(10000.0, 0.1, 10000.0)
                        .translation(Vector::new(0.0, -0.1, 0.0)),
                );
            }
            let mut queries = QueryPipeline::new();
            queries.update(&colliders);
            for x in [-27.0, 0.0, 27.0] {
                for steer in [-0.4, 0.0, 0.4] {
                    let sweep = WheelSweep {
                        mount: Point::new(x, 0.7, x),
                        direction: -Vector::y(),
                        axle: Rotation::new(Vector::y() * steer) * Vector::x(),
                        radius: 0.3,
                        width: 0.2,
                        min_length: 0.0,
                        max_length: 0.6,
                    };
                    let hit = sweep
                        .cast(
                            &bodies,
                            &colliders,
                            &queries,
                            QueryFilter::default(),
                            chassis,
                        )
                        .0
                        .unwrap();
                    assert_eq!(hit.normal, Vector::y(), "mesh={mesh}, {hit:?}");
                    assert!((hit.length - 0.4).abs() < 1.0e-6);
                    assert!((hit.point - Point::new(x, 0.0, x)).norm() < 1.0e-5);
                }
            }
        }
    }

    #[test]
    fn curb_support_rises_before_the_wheel_center_crosses_the_edge() {
        for mesh in [false, true] {
            let mut previous = sample(-0.4, mesh).length;
            for i in 1..=60 {
                let x = -0.4 + i as Real * 0.01;
                let hit = sample(x, mesh);
                let tire_height = if x <= 0.0 {
                    0.3_f64.max(0.1 + (0.09 - (x as f64).powi(2)).max(0.0).sqrt()) as Real
                } else {
                    0.4
                };
                assert!(
                    (hit.length - (0.7 - tire_height)).abs() < 0.002,
                    "mesh={mesh}, x={x}, {hit:?}"
                );
                assert!(
                    (hit.length - previous).abs() < 0.015,
                    "mesh={mesh}, x={x}, {hit:?}"
                );
                previous = hit.length;
            }
            assert!(sample(-0.1, mesh).length < 0.33);
        }
    }

    #[test]
    fn rejected_wall_does_not_hide_ground_in_the_same_mesh() {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(RigidBodyBuilder::dynamic());
        let mut colliders = ColliderSet::new();
        colliders.insert(
            ColliderBuilder::trimesh(
                vec![
                    Point::new(-2.0, 0.0, -2.0),
                    Point::new(0.0, 0.0, -2.0),
                    Point::new(0.0, 0.0, 2.0),
                    Point::new(-2.0, 0.0, 2.0),
                    Point::new(0.0, 2.0, -2.0),
                    Point::new(0.0, 2.0, 2.0),
                ],
                vec![[0, 2, 1], [0, 3, 2], [1, 2, 5], [1, 5, 4]],
            )
            .unwrap(),
        );
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        let (support, rejected) = WheelSweep {
            mount: Point::new(-0.1, 0.7, 0.0),
            direction: -Vector::y(),
            axle: Vector::z(),
            radius: 0.3,
            width: 0.2,
            min_length: 0.0,
            max_length: 0.6,
        }
        .cast(
            &bodies,
            &colliders,
            &queries,
            QueryFilter::default(),
            chassis,
        );
        assert!(rejected.is_some());
        let hit = support.unwrap();
        assert!((hit.length - 0.4).abs() < 0.001);
        assert!(hit.normal.y > 0.999);
    }

    #[test]
    fn width_reaches_a_lateral_ledge_and_filters_sensors_and_chassis() {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(RigidBodyBuilder::dynamic());
        let mut colliders = ColliderSet::new();
        colliders.insert_with_parent(
            ColliderBuilder::ball(0.5).translation(Vector::y() * 0.7),
            chassis,
            &mut bodies,
        );
        colliders.insert(
            ColliderBuilder::cuboid(2.0, 0.1, 2.0)
                .translation(Vector::y() * 0.3)
                .sensor(true),
        );
        let ground = colliders.insert(
            ColliderBuilder::cuboid(0.025, 0.05, 1.0).translation(Vector::new(0.08, -0.05, 0.0)),
        );
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        let mut sweep = WheelSweep {
            mount: Point::new(0.0, 0.7, 0.0),
            direction: -Vector::y(),
            axle: Vector::x(),
            radius: 0.3,
            width: 0.2,
            min_length: 0.0,
            max_length: 0.6,
        };
        let filter = QueryFilter::default().exclude_sensors();
        let hit = sweep
            .cast(&bodies, &colliders, &queries, filter, chassis)
            .0
            .unwrap();
        assert_eq!(hit.collider, ground);
        assert!((hit.length - 0.4).abs() < 0.001);
        sweep.width = 0.05;
        assert!(sweep
            .cast(&bodies, &colliders, &queries, filter, chassis)
            .0
            .is_none());
    }

    #[test]
    fn dropping_off_a_ledge_loses_contact_without_retaining_old_support() {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(RigidBodyBuilder::dynamic());
        let mut colliders = ColliderSet::new();
        colliders.insert(
            ColliderBuilder::cuboid(1.0, 0.05, 1.0).translation(Vector::new(-1.0, -0.05, 0.0)),
        );
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        let mut sweep = WheelSweep {
            mount: Point::new(-0.5, 0.7, 0.0),
            direction: -Vector::y(),
            axle: Vector::z(),
            radius: 0.3,
            width: 0.2,
            min_length: 0.0,
            max_length: 0.6,
        };
        assert!(sweep
            .cast(
                &bodies,
                &colliders,
                &queries,
                QueryFilter::default(),
                chassis
            )
            .0
            .is_some());
        sweep.mount.x = 0.4;
        assert!(sweep
            .cast(
                &bodies,
                &colliders,
                &queries,
                QueryFilter::default(),
                chassis
            )
            .0
            .is_none());
    }
}
