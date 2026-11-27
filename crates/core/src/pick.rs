//! Ray picking: a click in the world becomes a geometric selector.
//!
//! The viewer stays dumb: it sends a world-space ray and prints whatever
//! this module answers. The engine reuses the same face grouping and sharp
//! edge extraction that `add_sketch`, `fillet` and `chamfer` already trust,
//! so a picked reference is exactly a reference an agent can pass back —
//! no parallel face-ID namespace, no viewer-side geometry knowledge.
//!
//! Everything here is world-space: entity meshes are local, so each
//! candidate is transformed before the ray test. Ties are resolved by
//! entity id (the document's `BTreeMap` order) and then by edge order, so
//! the same scene and camera always answer with the same selector.

use std::collections::BTreeMap;

use wright_protocol::{
    EdgeSelector, EntityId, FaceSelector, Mesh, PickHit, Ray, Vec3,
};

use crate::{
    edges::{self, SharpEdge},
    mat4, plane, world_bbox, Entity, Error, Shape,
};

/// Cos of the face-pair angle that counts as a real edge. Matches the
/// fillet/chamfer default: a 32-segment cylinder seam (11.25°) is smooth,
/// a box corner (90°) is sharp.
const EDGE_ANGLE_DEG: f64 = 30.0;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn finite(v: [f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn length(v: [f64; 3]) -> f64 {
    dot(v, v).sqrt()
}

/// Nearest forward intersection with one triangle. Both windings hit; a
/// ray from inside a hollow-looking mesh still reports its wall.
fn intersect_triangle(origin: [f64; 3], dir: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<f64> {
    let e1 = sub(b, a);
    let e2 = sub(c, a);
    let p = cross(dir, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-12 {
        return None; // ray parallel to the triangle plane
    }
    let inv = 1.0 / det;
    let tvec = sub(origin, a);
    let u = dot(tvec, p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(tvec, e1);
    let v = dot(dir, q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = dot(e2, q) * inv;
    (t > 1e-9).then_some(t)
}

/// Distance from `p` to segment `a`–`b`.
fn point_segment_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub(b, a);
    let denom = dot(ab, ab);
    if denom < 1e-24 {
        return length(sub(p, a));
    }
    let t = (dot(sub(p, a), ab) / denom).clamp(0.0, 1.0);
    length(sub(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t]))
}

/// Cast `ray` at every solid in the document and answer with the nearest
/// hit. Sketches are display ribbons, not printable geometry, so they are
/// skipped (`Shape::Sketch`); a sketch never hides the solid behind it.
pub fn pick(
    entities: &BTreeMap<EntityId, Entity>,
    ray: &Ray,
    tolerance: Option<f64>,
) -> Result<Option<PickHit>, Error> {
    if !finite(ray.origin) {
        return Err(Error::Invalid("pick ray.origin must be a finite point".into()));
    }
    if !finite(ray.direction) || length(ray.direction) < 1e-12 {
        return Err(Error::Invalid(
            "pick ray.direction must be a non-zero vector (hint: eye → point, e.g. [0,0,-1] for top-down)"
                .into(),
        ));
    }
    if tolerance.is_some_and(|tol| !tol.is_finite() || tol <= 0.0) {
        return Err(Error::Invalid("pick tolerance must be a finite value > 0 (mm)".into()));
    }
    let dir = {
        let len = length(ray.direction);
        [ray.direction[0] / len, ray.direction[1] / len, ray.direction[2] / len]
    };

    let mut best: Option<(f64, EntityId, usize, Vec3)> = None;
    for (&id, entity) in entities {
        if matches!(entity.shape, Shape::Sketch { .. }) || entity.mesh.indices.is_empty() {
            continue;
        }
        let world = world_mesh(entity);
        for tri in 0..world.indices.len() / 3 {
            let i = &world.indices[tri * 3..tri * 3 + 3];
            let a = world.positions[i[0] as usize].map(f64::from);
            let b = world.positions[i[1] as usize].map(f64::from);
            let c = world.positions[i[2] as usize].map(f64::from);
            let Some(t) = intersect_triangle(ray.origin, dir, a, b, c) else {
                continue;
            };
            // Strict `<` keeps the lowest entity id for an exact tie, and
            // therefore makes the answer independent of hash order.
            if best.as_ref().is_none_or(|(bt, ..)| t < *bt) {
                best = Some((t, id, tri, [ray.origin[0] + dir[0] * t, ray.origin[1] + dir[1] * t, ray.origin[2] + dir[2] * t]));
            }
        }
    }

    let Some((distance, entity_id, tri, point)) = best else {
        return Ok(None);
    };
    let entity = &entities[&entity_id];
    let world = world_mesh(entity);

    // The planar patch under the hit gives the stable outward normal and
    // a point that selects this face among parallel ones.
    let faces = plane::planar_faces(&world)?;
    let face = faces.iter().find(|f| f.triangles.binary_search(&tri).is_ok());
    let (normal, near) = match face {
        Some(f) => {
            let p = sub(point, [f.normal[0] * (dot(f.normal, point) - f.offset), f.normal[1] * (dot(f.normal, point) - f.offset), f.normal[2] * (dot(f.normal, point) - f.offset)]);
            (f.normal, p)
        },
        None => {
            // A degenerate patch: fall back to the triangle's own normal.
            let i = &world.indices[tri * 3..tri * 3 + 3];
            let a = world.positions[i[0] as usize].map(f64::from);
            let b = world.positions[i[1] as usize].map(f64::from);
            let c = world.positions[i[2] as usize].map(f64::from);
            let n = cross(sub(b, a), sub(c, a));
            let l = length(n);
            let n = if l < 1e-12 { [0.0; 3] } else { [n[0] / l, n[1] / l, n[2] / l] };
            (n, point)
        },
    };

    let tol = tolerance.unwrap_or_else(|| {
        let bb = world_bbox(&entity.mesh, &entity.transform());
        let d = [
            bb.max[0] - bb.min[0],
            bb.max[1] - bb.min[1],
            bb.max[2] - bb.min[2],
        ];
        (length(d) * 0.01).max(0.01)
    });

    // Edge snapping reuses fillet's sharp-edge extraction. An open mesh is
    // pickable (its face still is), just without an edge answer.
    let edge = edges::sharp_edges(&world, EDGE_ANGLE_DEG)
        .ok()
        .and_then(|edges| nearest_edge(&edges, point, tol).cloned())
        .map(|e| {
            let d = e.direction();
            EdgeSelector {
                direction: d,
                near: e.midpoint(),
                near_radius: e.length() * 0.25 + tol,
            }
        });

    Ok(Some(PickHit {
        entity: entity_id,
        name: entity.name.clone(),
        distance,
        point,
        normal,
        face: FaceSelector { entity: entity_id, normal, near },
        edge,
    }))
}

/// The sharp edge closest to `point`, if one is within `tol`. Ties keep
/// the first edge in `sharp_edges` order, which is deterministic.
fn nearest_edge(edges: &[SharpEdge], point: Vec3, tol: f64) -> Option<&SharpEdge> {
    let mut best: Option<(f64, &SharpEdge)> = None;
    for e in edges {
        let d = point_segment_distance(point, e.a, e.b);
        if d <= tol && best.as_ref().is_none_or(|(bd, _)| d < *bd) {
            best = Some((d, e));
        }
    }
    best.map(|(_, e)| e)
}

/// Entity-local mesh in world space. Normals are left local on purpose:
/// both consumers here (face grouping and sharp edges) derive normals from
/// positions, so they stay correct for a rotated or non-uniformly scaled
/// entity.
fn world_mesh(entity: &Entity) -> Mesh {
    let m = entity.transform();
    Mesh {
        positions: entity.mesh.positions.iter().map(|&p| mat4::transform_point(&m, p)).collect(),
        normals: entity.mesh.normals.clone(),
        indices: entity.mesh.indices.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{geo, Engine};
    use wright_protocol::{Command, CommandResult, Scalar};

    fn num(v: f64) -> Scalar {
        Scalar::Number(v)
    }

    fn engine_with_box() -> (Engine, EntityId) {
        let mut e = Engine::new();
        e.execute(Command::CreateDocument { name: "pick".into(), doc: None }).unwrap();
        let entity = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "base".into(),
                center: [num(0.0), num(0.0), num(5.0)],
                size: [num(20.0), num(20.0), num(10.0)],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected {r:?}"),
        };
        (e, entity)
    }

    fn pick_cmd(e: &mut Engine, origin: [f64; 3], direction: [f64; 3], tolerance: Option<f64>) -> Option<PickHit> {
        match e
            .execute(Command::Pick { doc: 1, ray: Ray { origin, direction }, tolerance })
            .unwrap()
        {
            CommandResult::Picked { hit, .. } => hit,
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn a_top_down_ray_hits_the_top_face_and_its_selector_works() {
        let (mut e, base) = engine_with_box();
        let hit = pick_cmd(&mut e, [0.0, 0.0, 100.0], [0.0, 0.0, -1.0], None).unwrap();
        assert_eq!(hit.entity, base);
        assert_eq!(hit.name, "base");
        assert!((hit.distance - 90.0).abs() < 1e-9, "{}", hit.distance);
        assert!((hit.point[2] - 10.0).abs() < 1e-9, "{:?}", hit.point);
        assert!((hit.normal[2] - 1.0).abs() < 1e-9, "{:?}", hit.normal);
        assert!(hit.edge.is_none(), "the center of a face is far from every edge");

        // The face selector is the wire shape `add_sketch` already accepts.
        match e
            .execute(Command::AddSketch {
                doc: 1,
                name: "on_top".into(),
                plane: Some(wright_protocol::SketchPlane::Face {
                    entity: hit.face.entity,
                    near: Some([num(hit.face.near[0]), num(hit.face.near[1]), num(hit.face.near[2])]),
                    normal: Some(hit.face.normal),
                    angle_tol_deg: None,
                    expect: None,
                }),
            })
            .unwrap()
        {
            CommandResult::EntityCreated { .. } => {},
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn a_click_near_an_edge_returns_a_selector_that_matches_exactly_one_edge() {
        let (mut e, _base) = engine_with_box();
        // 0.2 mm inside the +X top rim (x = 10, z = 10), square in Y.
        let hit = pick_cmd(&mut e, [9.8, 0.0, 100.0], [0.0, 0.0, -1.0], None).unwrap();
        let edge = hit.edge.expect("a hit 0.2 mm from the top rim must snap to it");
        assert!((edge.direction[1].abs() - 1.0).abs() < 1e-9, "{:?}", edge.direction);
        assert!((edge.near[0] - 10.0).abs() < 1e-6, "{:?}", edge.near);
        assert!((edge.near[1]).abs() < 1e-6 && (edge.near[2] - 10.0).abs() < 1e-6);

        let filter = wright_protocol::EdgeFilter {
            direction: Some(edge.direction),
            near: Some(edge.near),
            near_radius: Some(edge.near_radius),
            ..Default::default()
        };
        let mut world = geo::box_mesh([20.0, 20.0, 10.0]);
        for p in &mut world.positions {
            p[2] += 5.0; // the box entity sits centred at z = 5
        }
        let all = edges::sharp_edges(&world, EDGE_ANGLE_DEG).unwrap();
        assert_eq!(
            all.iter().filter(|x| edges::matches(&filter, x)).count(),
            1,
            "the picked selector must identify exactly the edge under the click"
        );

        // And it is accepted by the command it is shaped for.
        match e
            .execute(Command::Fillet {
                doc: 1,
                entity: hit.entity,
                radius: num(1.0),
                edges: Some(filter),
                min_angle_deg: None,
            })
            .unwrap()
        {
            CommandResult::EntityModified { edges, .. } => assert_eq!(edges, 1),
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn a_miss_is_an_answer_not_an_error() {
        let (mut e, _) = engine_with_box();
        assert!(pick_cmd(&mut e, [100.0, 100.0, 100.0], [1.0, 0.0, 0.0], None).is_none());
    }

    #[test]
    fn the_nearest_entity_wins_and_sketches_are_skipped() {
        let (mut e, base) = engine_with_box();
        let boss = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "boss".into(),
                center: [num(0.0), num(0.0), num(20.0)],
                size: [num(4.0), num(4.0), num(20.0)],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected {r:?}"),
        };
        let hit = pick_cmd(&mut e, [0.0, 0.0, 100.0], [0.0, 0.0, -1.0], None).unwrap();
        assert_eq!(hit.entity, boss);
        assert!((hit.point[2] - 30.0).abs() < 1e-9);

        // A sketch above the box is display-only; it must not absorb the ray.
        e.execute(Command::AddSketch { doc: 1, name: "draft".into(), plane: Some(wright_protocol::SketchPlane::Datum { plane: wright_protocol::DatumPlane::Xy, offset: Some(num(50.0)) }) })
            .unwrap();
        let hit = pick_cmd(&mut e, [0.0, 0.0, 100.0], [0.0, 0.0, -1.0], None).unwrap();
        assert_eq!(hit.entity, boss);
        assert_ne!(hit.entity, base);
    }

    #[test]
    fn picks_follow_entity_rotation_into_world_space() {
        let (mut e, base) = engine_with_box();
        e.execute(Command::Transform {
            doc: 1,
            entity: base,
            translation: None,
            rotation_deg: Some([num(90.0), num(0.0), num(0.0)]),
            scale: None,
        })
        .unwrap();
        // The box now stands 10 mm wide in Z and 20 mm deep in Y, centred
        // at the origin: its old top face points at -Y.
        let hit = pick_cmd(&mut e, [0.0, -100.0, 0.0], [0.0, 1.0, 0.0], None).unwrap();
        assert!((hit.normal[1] + 1.0).abs() < 1e-6, "{:?}", hit.normal);
        assert!((hit.point[1] + 5.0).abs() < 1e-6, "{:?}", hit.point);
    }

    #[test]
    fn tolerance_controls_edge_snapping() {
        let (mut e, _) = engine_with_box();
        let origin = [9.8, 0.0, 100.0];
        let dir = [0.0, 0.0, -1.0];
        assert!(pick_cmd(&mut e, origin, dir, Some(0.1)).unwrap().edge.is_none());
        assert!(pick_cmd(&mut e, origin, dir, Some(0.5)).unwrap().edge.is_some());
    }

    #[test]
    fn bad_rays_are_rejected_with_hints() {
        let (mut e, _) = engine_with_box();
        let err = e
            .execute(Command::Pick { doc: 1, ray: Ray { origin: [0.0; 3], direction: [0.0; 3] }, tolerance: None })
            .unwrap_err();
        assert!(err.to_string().contains("ray.direction"), "{err}");
        assert!(e
            .execute(Command::Pick { doc: 1, ray: Ray { origin: [0.0; 3], direction: [0.0, 0.0, -1.0] }, tolerance: Some(0.0) })
            .is_err());
    }
}
