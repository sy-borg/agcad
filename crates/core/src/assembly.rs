//! Assemblies: reusable parts, placed instances, connectors and mates.
//!
//! A **part** is a complete `.wcad`-shaped document embedded in the assembly
//! program by [`wright_protocol::Command::AddPart`]. `add_instance` replays that part privately
//! (with optional per-instance parameter overrides) and turns every printable
//! solid into an ordinary entity of the assembly. Nothing references the
//! source file or document afterwards, which keeps the same self-contained
//! promise as `import_mesh`: save the assembly, delete the parts, reopen it —
//! the geometry still replays.
//!
//! An **instance** keeps its placement (translation, extrinsic XYZ rotation,
//! uniform scale) and the part-local placement of each member. Moving the
//! instance rewrites the members' transforms; their meshes stay local, so
//! fillets and booleans keep working on them like any other solid.
//!
//! **Mates** are sequential, not a global constraint solver: each mate is a
//! rigid motion of instance `b` that puts its connector point (and axis) on
//! instance `a`'s. That is enough for the common assembly joints — a screw
//! into a hole, a plate on a spacer — and it is deterministic and replayable.

use std::collections::BTreeMap;

use wright_protocol::{
    Bbox, BomRow, BooleanOp, CommandResult, DocId, DocumentSnapshot, EntityId, InterferenceHit,
    MateKind, MateRef, Mesh, Scalar, Vec3, DOCUMENT_FORMAT, DOCUMENT_VERSION,
};

use crate::mat4;
use crate::{rebuild, Document, Error, Result, Shape};

/// A registered part: its embedded document plus precomputed facts used in
/// results and BOM rows.
#[derive(Debug, Clone)]
pub struct Part {
    pub name: String,
    /// The part document. Its program still addresses doc 1; it is replayed
    /// privately and never inserted into the engine's document list.
    pub snapshot: DocumentSnapshot,
    pub description: String,
    pub solids: usize,
    pub parameters: usize,
}

/// One part-local member placement, kept so an instance can move without
/// rebuilding the part.
#[derive(Debug, Clone, Copy)]
pub struct Member {
    pub entity: EntityId,
    pub source: EntityId,
    pub local_translation: Vec3,
    pub local_rotation_deg: Vec3,
    pub local_scale: Vec3,
}

/// A named point (and optional axis) in instance-local coordinates.
#[derive(Debug, Clone, Copy)]
pub struct Connector {
    pub point: Vec3,
    pub axis: Option<Vec3>,
}

/// One placed part.
#[derive(Debug, Clone)]
pub struct Instance {
    pub id: u64,
    pub name: String,
    pub part: String,
    pub translation: Vec3,
    pub rotation_deg: Vec3,
    pub scale: f64,
    /// Resolved per-instance parameter overrides.
    pub parameters: BTreeMap<String, f64>,
    pub members: Vec<Member>,
    pub connectors: BTreeMap<String, Connector>,
}

impl Instance {
    /// World-space position of an instance-local point.
    pub fn point(&self, p: Vec3) -> Vec3 {
        let r = mat4::rotation_xyz_deg(self.rotation_deg);
        let q = mat4::transform_direction(
            &r,
            [p[0] * self.scale, p[1] * self.scale, p[2] * self.scale],
        );
        [
            self.translation[0] + q[0],
            self.translation[1] + q[1],
            self.translation[2] + q[2],
        ]
    }

    /// World-space direction of an instance-local axis.
    pub fn axis(&self, a: Vec3) -> Vec3 {
        let r = mat4::rotation_xyz_deg(self.rotation_deg);
        mat4::transform_direction(&r, a)
    }
}

/// Validate a part snapshot and compute the facts shown in results.
pub fn register_part(
    d: &mut Document,
    doc: DocId,
    name: &str,
    snapshot: DocumentSnapshot,
) -> Result<CommandResult> {
    let name = name.trim();
    if name.is_empty() {
        return Err(Error::Invalid(
            "part name must not be empty (hint: name it after the file or document)".into(),
        ));
    }
    if d.parts().contains_key(name) {
        return Err(Error::Invalid(format!(
            "part '{name}' is already defined (hint: part names are unique in a document; \
             use a different name, or delete the existing instances first)"
        )));
    }
    if snapshot.format != DOCUMENT_FORMAT {
        return Err(Error::Invalid(format!(
            "part '{name}' is not a wright document (format: '{}')",
            snapshot.format
        )));
    }
    if snapshot.version > DOCUMENT_VERSION {
        return Err(Error::Invalid(format!(
            "part '{name}' is version {} but this build reads up to {DOCUMENT_VERSION} \
             (hint: upgrade the server)",
            snapshot.version
        )));
    }

    // Replay once now: a broken part file should fail at import, not at the
    // first instance. The private document id is 1, which is what the
    // snapshot's commands were normalized to.
    let private = Document::from_snapshot(snapshot.clone(), 1, snapshot.name.clone())?;
    let solids = private
        .entities()
        .values()
        .filter(|e| !matches!(e.shape, Shape::Sketch { .. }))
        .count();
    if solids == 0 {
        return Err(Error::Invalid(format!(
            "part '{name}' contains no printable solids (hint: sketches are display-only; \
             extrude/revolve the part before importing it)"
        )));
    }

    let part = Part {
        name: name.to_string(),
        description: snapshot.name.clone(),
        solids,
        parameters: snapshot.parameters.len(),
        snapshot,
    };
    d.parts_mut().insert(name.to_string(), part);
    Ok(CommandResult::PartDefined {
        doc,
        name: name.to_string(),
        solids,
        parameters: d.parts()[name].parameters,
    })
}

/// Replay a part with overrides into a private document.
fn build_part(part: &Part, overrides: &BTreeMap<String, f64>) -> Result<Document> {
    let mut snapshot = part.snapshot.clone();
    for (name, value) in overrides {
        match snapshot.parameters.get_mut(name) {
            Some(p) => p.value = *value,
            None => {
                let valid: Vec<&str> = snapshot.parameters.keys().map(String::as_str).collect();
                return Err(Error::Invalid(format!(
                    "part '{}' has no parameter '{name}' (valid: {})",
                    part.name,
                    if valid.is_empty() {
                        "none".to_string()
                    } else {
                        valid.join(", ")
                    }
                )));
            }
        }
    }
    Document::from_snapshot(snapshot, 1, part.description.clone())
}

/// Combine an instance placement with a member's part-local placement.
fn combine(instance: &Instance, member: &Member) -> (Vec3, Vec3, Vec3) {
    let r_inst = mat4::rotation_xyz_deg(instance.rotation_deg);
    let scaled = [
        member.local_translation[0] * instance.scale,
        member.local_translation[1] * instance.scale,
        member.local_translation[2] * instance.scale,
    ];
    let rotated = mat4::transform_direction(&r_inst, scaled);
    let translation = [
        instance.translation[0] + rotated[0],
        instance.translation[1] + rotated[1],
        instance.translation[2] + rotated[2],
    ];
    let rotation = mat4::mul(&r_inst, &mat4::rotation_xyz_deg(member.local_rotation_deg));
    let rotation_deg = mat4::euler_xyz_deg(&rotation);
    let scale = [
        member.local_scale[0] * instance.scale,
        member.local_scale[1] * instance.scale,
        member.local_scale[2] * instance.scale,
    ];
    (translation, rotation_deg, scale)
}

/// Place a part in the assembly. Returns the new instance id and member ids.
#[allow(clippy::too_many_arguments)]
pub fn add_instance(
    d: &mut Document,
    doc: DocId,
    part_name: &str,
    name: &str,
    translation: Option<&wright_protocol::SVec3>,
    rotation_deg: Option<&wright_protocol::SVec3>,
    scale: Option<&Scalar>,
    parameters: Option<&BTreeMap<String, Scalar>>,
) -> Result<CommandResult> {
    let name = name.trim();
    if name.is_empty() {
        return Err(Error::Invalid(
            "instance name must not be empty (hint: use a name like 'bolt_1')".into(),
        ));
    }
    if d.instances().values().any(|i| i.name == name) {
        return Err(Error::Invalid(format!(
            "instance '{name}' already exists (hint: instance names are unique in a \
             document; add_instance returns the numeric id mates use)"
        )));
    }
    let part = d.parts().get(part_name).cloned().ok_or_else(|| {
        let valid: Vec<&str> = d.parts().keys().map(String::as_str).collect();
        Error::Invalid(format!(
            "unknown part '{part_name}' (hint: import it first; defined parts: {})",
            if valid.is_empty() {
                "none".to_string()
            } else {
                valid.join(", ")
            }
        ))
    })?;
    let translation = match translation {
        Some(v) => d.resolve3(v)?,
        None => [0.0; 3],
    };
    let rotation_deg = match rotation_deg {
        Some(v) => d.resolve3(v)?,
        None => [0.0; 3],
    };
    let scale = match scale {
        Some(v) => d.resolve(v)?,
        None => 1.0,
    };
    if !scale.is_finite() || scale <= 0.0 {
        return Err(Error::Invalid("instance scale must be > 0".into()));
    }
    let mut overrides = BTreeMap::new();
    for (k, v) in parameters.into_iter().flatten() {
        overrides.insert(k.clone(), d.resolve(v)?);
    }

    let private = build_part(&part, &overrides)?;
    let instance_id = d.next_instance();
    let mut instance = Instance {
        id: instance_id,
        name: name.to_string(),
        part: part.name.clone(),
        translation,
        rotation_deg,
        scale,
        parameters: overrides.clone(),
        members: Vec::new(),
        connectors: BTreeMap::new(),
    };

    let solids: Vec<(
        EntityId,
        Vec3,
        Vec3,
        Vec3,
        String,
        wright_protocol::Rgba,
        Mesh,
    )> = private
        .entities()
        .values()
        .filter(|e| !matches!(e.shape, Shape::Sketch { .. }))
        .map(|e| {
            (
                e.id,
                e.translation,
                e.rotation_deg,
                e.scale,
                e.name.clone(),
                e.color,
                e.mesh.clone(),
            )
        })
        .collect();

    let mut entities = Vec::with_capacity(solids.len());
    for (source, lt, lr, ls, member_name, color, mesh) in solids {
        let member = Member {
            entity: 0, // filled below
            source,
            local_translation: lt,
            local_rotation_deg: lr,
            local_scale: ls,
        };
        let id = d.next_entity();
        let key = rebuild::part_member_key(&part.name, source, &overrides);
        let (t, r, s) = combine(&instance, &member);
        let entity = crate::Entity {
            id,
            name: format!("{}.{}", instance.name, member_name),
            shape: Shape::PartMember {
                instance: instance.id,
                part: part.name.clone(),
            },
            mesh,
            translation: t,
            rotation_deg: r,
            scale: s,
            color,
            geom_key: key,
        };
        d.entities_mut().insert(id, entity);
        instance.members.push(Member {
            entity: id,
            ..member
        });
        entities.push(id);
    }

    d.instances_mut().insert(instance.id, instance);
    Ok(CommandResult::InstanceCreated {
        doc,
        instance: instance_id,
        name: name.to_string(),
        part: part.name,
        entities,
    })
}

/// Rewrite one instance's member transforms from its placement.
pub fn place_instance(d: &mut Document, instance_id: u64) -> Result<()> {
    let instance = d.instances().get(&instance_id).cloned().ok_or_else(|| {
        Error::Invalid(format!(
            "unknown instance {instance_id} (hint: add_instance returns the id; \
             get_scene names every member)"
        ))
    })?;
    for member in &instance.members {
        let (t, r, s) = combine(&instance, member);
        if let Some(e) = d.entities_mut().get_mut(&member.entity) {
            e.translation = t;
            e.rotation_deg = r;
            e.scale = s;
        }
    }
    Ok(())
}

pub fn add_connector(
    d: &mut Document,
    doc: DocId,
    instance: u64,
    name: &str,
    point: &wright_protocol::SVec3,
    axis: Option<&[f64; 3]>,
) -> Result<CommandResult> {
    if name.trim().is_empty() {
        return Err(Error::Invalid("connector name must not be empty".into()));
    }
    let point = d.resolve3(point)?;
    let axis = match axis {
        Some(a) => {
            let len = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            if !len.is_finite() || len < 1e-12 {
                return Err(Error::Invalid(
                    "connector axis must be a non-zero vector".into(),
                ));
            }
            Some([a[0] / len, a[1] / len, a[2] / len])
        }
        None => None,
    };
    let inst = d.instances_mut().get_mut(&instance).ok_or_else(|| {
        Error::Invalid(format!(
            "unknown instance {instance} (hint: add_instance returns the id)"
        ))
    })?;
    if inst.connectors.contains_key(name) {
        return Err(Error::Invalid(format!(
            "instance '{}' already has a connector called '{name}'",
            inst.name
        )));
    }
    inst.connectors
        .insert(name.to_string(), Connector { point, axis });
    Ok(CommandResult::ConnectorAdded {
        doc,
        instance,
        name: name.to_string(),
    })
}

pub fn move_instance(
    d: &mut Document,
    doc: DocId,
    instance: u64,
    translation: Option<&wright_protocol::SVec3>,
    rotation_deg: Option<&wright_protocol::SVec3>,
    scale: Option<&Scalar>,
) -> Result<CommandResult> {
    let translation = translation.map(|v| d.resolve3(v)).transpose()?;
    let rotation_deg = rotation_deg.map(|v| d.resolve3(v)).transpose()?;
    let scale = scale.map(|v| d.resolve(v)).transpose()?;
    if let Some(s) = scale {
        if !s.is_finite() || s <= 0.0 {
            return Err(Error::Invalid("instance scale must be > 0".into()));
        }
    }
    {
        let inst = d.instances_mut().get_mut(&instance).ok_or_else(|| {
            Error::Invalid(format!(
                "unknown instance {instance} (hint: add_instance returns the id)"
            ))
        })?;
        if let Some(t) = translation {
            inst.translation = t;
        }
        if let Some(r) = rotation_deg {
            inst.rotation_deg = r;
        }
        if let Some(s) = scale {
            inst.scale = s;
        }
    }
    place_instance(d, instance)?;
    Ok(CommandResult::InstanceMoved { doc, instance })
}

fn connector_world(inst: &Instance, name: &str) -> Result<(Vec3, Option<Vec3>)> {
    let c = inst.connectors.get(name).ok_or_else(|| {
        let valid: Vec<&str> = inst.connectors.keys().map(String::as_str).collect();
        Error::Invalid(format!(
            "instance '{}' has no connector '{name}' (defined: {})",
            inst.name,
            if valid.is_empty() {
                "none".to_string()
            } else {
                valid.join(", ")
            }
        ))
    })?;
    Ok((inst.point(c.point), c.axis.map(|a| inst.axis(a))))
}

fn normalized(v: Vec3) -> Option<Vec3> {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if !len.is_finite() || len < 1e-12 {
        None
    } else {
        Some([v[0] / len, v[1] / len, v[2] / len])
    }
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn mate(
    d: &mut Document,
    doc: DocId,
    kind: MateKind,
    a: &MateRef,
    b: &MateRef,
    offset: Option<&Scalar>,
) -> Result<CommandResult> {
    if a.instance == b.instance {
        return Err(Error::Invalid(
            "a mate needs two different instances (hint: two connectors on the same \
             instance are a sketch-level relation, not a mate)"
                .into(),
        ));
    }
    let offset = match offset {
        Some(v) => d.resolve(v)?,
        None => 0.0,
    };
    if !offset.is_finite() {
        return Err(Error::Invalid("mate offset must be finite".into()));
    }

    let inst_a = d.instances().get(&a.instance).cloned().ok_or_else(|| {
        Error::Invalid(format!(
            "unknown instance {} (hint: add_instance returns the id)",
            a.instance
        ))
    })?;
    let mut inst_b = d.instances().get(&b.instance).cloned().ok_or_else(|| {
        Error::Invalid(format!(
            "unknown instance {} (hint: add_instance returns the id)",
            b.instance
        ))
    })?;
    let (pa, aa) = connector_world(&inst_a, &a.connector)?;
    let (pb, ab) = connector_world(&inst_b, &b.connector)?;
    let b_local = inst_b
        .connectors
        .get(&b.connector)
        .map(|c| c.point)
        .ok_or_else(|| Error::Invalid("missing connector".into()))?;

    let (target_point, target_axis) = match kind {
        MateKind::Coincident => {
            let axis = match (aa, ab) {
                (Some(axis_a), Some(_)) => Some([-axis_a[0], -axis_a[1], -axis_a[2]]),
                _ => None,
            };
            (pa, axis)
        }
        MateKind::Distance => {
            let axis_a = aa.ok_or_else(|| {
                Error::Invalid(format!(
                    "distance mate needs an axis on connector '{}.{}'",
                    inst_a.name, a.connector
                ))
            })?;
            let target = [
                pa[0] + axis_a[0] * offset,
                pa[1] + axis_a[1] * offset,
                pa[2] + axis_a[2] * offset,
            ];
            (target, Some([-axis_a[0], -axis_a[1], -axis_a[2]]))
        }
        MateKind::Parallel => {
            let axis_a = aa.ok_or_else(|| {
                Error::Invalid(format!(
                    "parallel mate needs an axis on connector '{}.{}'",
                    inst_a.name, a.connector
                ))
            })?;
            (pb, Some(axis_a))
        }
        MateKind::Angle => {
            let axis_a = aa.ok_or_else(|| {
                Error::Invalid(format!(
                    "angle mate needs an axis on connector '{}.{}'",
                    inst_a.name, a.connector
                ))
            })?;
            // A stable perpendicular reference: Z for horizontal axes, X for
            // vertical ones. Offset 0 is parallel, 180 anti-parallel.
            let helper = if axis_a[2].abs() < 0.9 {
                [0.0, 0.0, 1.0]
            } else {
                [1.0, 0.0, 0.0]
            };
            let mut ref_axis = cross(axis_a, helper);
            if normalized(ref_axis).is_none() {
                ref_axis = cross(axis_a, [1.0, 0.0, 0.0]);
            }
            let rot = mat4::rotation_axis_angle(ref_axis, offset);
            let target = mat4::transform_direction(&rot, axis_a);
            (pa, Some(target))
        }
    };

    // New rotation: align B's connector axis with the target (if any).
    let new_rotation = match (ab, target_axis) {
        (Some(current), Some(target)) => {
            let delta = mat4::rotation_between(current, target)
                .ok_or_else(|| Error::Invalid("cannot align degenerate connector axes".into()))?;
            mat4::mul(&delta, &mat4::rotation_xyz_deg(inst_b.rotation_deg))
        }
        _ => mat4::rotation_xyz_deg(inst_b.rotation_deg),
    };
    let r_new = mat4::euler_xyz_deg(&new_rotation);
    let s = inst_b.scale;
    let rotated = mat4::transform_direction(
        &mat4::rotation_xyz_deg(r_new),
        [b_local[0] * s, b_local[1] * s, b_local[2] * s],
    );
    let t_new = [
        target_point[0] - rotated[0],
        target_point[1] - rotated[1],
        target_point[2] - rotated[2],
    ];

    inst_b.translation = t_new;
    inst_b.rotation_deg = r_new;
    d.instances_mut().insert(inst_b.id, inst_b.clone());
    place_instance(d, inst_b.id)?;
    let mut values = Vec::new();
    for v in [pa, pb] {
        values.extend_from_slice(&v);
    }
    for axis in [aa, ab] {
        values.extend_from_slice(&axis.unwrap_or([0.0; 3]));
    }
    values.push(offset);
    let domain = format!(
        "wright/mate/1|{}.{}|{}.{}",
        inst_a.name, a.connector, inst_b.name, b.connector
    );
    let reference = crate::refs::fixed_report(
        &domain,
        &values,
        format!(
            "mate {} between {}.{} and {}.{}",
            kind, inst_a.name, a.connector, inst_b.name, b.connector
        ),
    );
    d.record_reference(&reference);
    Ok(CommandResult::MateSolved {
        doc,
        kind,
        a_instance: a.instance,
        b_instance: b.instance,
        reference: Some(reference),
    })
}

/// Signed local volume of a mesh (positive when outward-wound).
fn mesh_volume(mesh: &Mesh) -> f64 {
    crate::fillet::signed_volume(mesh)
}

/// World-space volume of an entity: local volume times the absolute
/// determinant of its placement scale.
fn world_volume(e: &crate::Entity) -> f64 {
    let det = (e.scale[0] * e.scale[1] * e.scale[2]).abs();
    mesh_volume(&e.mesh) * det
}

pub fn bill_of_materials(d: &Document) -> Vec<BomRow> {
    let mut by_part: BTreeMap<String, Vec<&Instance>> = BTreeMap::new();
    for inst in d.instances().values() {
        by_part.entry(inst.part.clone()).or_default().push(inst);
    }
    let mut rows = Vec::with_capacity(by_part.len());
    for (item, (part_name, instances)) in by_part.iter().enumerate() {
        let (entities, volume) = instances.iter().fold((0usize, 0.0), |(n, v), inst| {
            let mut count = n;
            let mut volume = v;
            for m in &inst.members {
                if let Some(e) = d.entities().get(&m.entity) {
                    count += 1;
                    volume += world_volume(e);
                }
            }
            (count, volume)
        });
        let description = d.parts().get(part_name).map(|p| p.description.clone());
        rows.push(BomRow {
            item: item as u32 + 1,
            part: part_name.clone(),
            description,
            quantity: instances.len() as u32,
            entities,
            volume,
            instances: instances.iter().map(|i| i.name.clone()).collect(),
        });
    }
    rows
}

fn bbox_of(mesh: &Mesh) -> Bbox {
    if mesh.positions.is_empty() {
        return Bbox {
            min: [0.0; 3],
            max: [0.0; 3],
        };
    }
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i] as f64);
            hi[i] = hi[i].max(p[i] as f64);
        }
    }
    Bbox { min: lo, max: hi }
}

fn aabb_overlap(a: &Bbox, b: &Bbox, tol: f64) -> bool {
    (0..3).all(|i| a.min[i] <= b.max[i] + tol && b.min[i] <= a.max[i] + tol)
}

pub fn check_interference(
    d: &Document,
    doc: DocId,
    instances: Option<&[u64]>,
    entities: Option<&[EntityId]>,
    tolerance: Option<f64>,
) -> Result<CommandResult> {
    let tolerance = tolerance.unwrap_or(1e-3);
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(Error::Invalid("interference tolerance must be >= 0".into()));
    }

    // Candidate pairs, already restricted to solids that exist.
    let mut candidates: Vec<(Option<u64>, Option<u64>, EntityId, EntityId)> = Vec::new();
    if let Some(ids) = entities {
        for (i, &a) in ids.iter().enumerate() {
            if !d.entities().contains_key(&a) {
                return Err(Error::UnknownEntity { doc, entity: a });
            }
            for &b in ids.iter().skip(i + 1) {
                if !d.entities().contains_key(&b) {
                    return Err(Error::UnknownEntity { doc, entity: b });
                }
                candidates.push((None, None, a, b));
            }
        }
    } else {
        let chosen: Vec<&Instance> = match instances {
            Some(ids) => {
                let mut out = Vec::new();
                for id in ids {
                    let inst = d.instances().get(id).ok_or_else(|| {
                        Error::Invalid(format!(
                            "unknown instance {id} (hint: add_instance returns the id)"
                        ))
                    })?;
                    out.push(inst);
                }
                out
            }
            None => d.instances().values().collect(),
        };
        for (i, a) in chosen.iter().enumerate() {
            for b in chosen.iter().skip(i + 1) {
                for ma in &a.members {
                    for mb in &b.members {
                        if d.entities().contains_key(&ma.entity)
                            && d.entities().contains_key(&mb.entity)
                        {
                            candidates.push((Some(a.id), Some(b.id), ma.entity, mb.entity));
                        }
                    }
                }
            }
        }
    }

    let mut hits = Vec::new();
    let mut checked = 0usize;
    for (ia, ib, ea, eb) in candidates {
        let a = d.entities().get(&ea).expect("candidate exists");
        let b = d.entities().get(&eb).expect("candidate exists");
        if matches!(a.shape, Shape::Sketch { .. }) || matches!(b.shape, Shape::Sketch { .. }) {
            continue;
        }
        let ba = a.snapshot().world_bbox;
        let bb = b.snapshot().world_bbox;
        if !aabb_overlap(&ba, &bb, 0.0) {
            continue;
        }
        checked += 1;
        let ma = crate::transform_mesh(&a.mesh, &a.transform());
        let mb = crate::transform_mesh(&b.mesh, &b.transform());
        let mesh = match crate::boolean::boolean(BooleanOp::Intersection, &[ma, mb]) {
            Ok(mesh) => mesh,
            Err(_) => continue, // open/non-manifold operand: no reliable overlap
        };
        let volume = mesh_volume(&mesh).abs();
        if volume <= tolerance {
            continue;
        }
        let a_name = d
            .instances()
            .get(&ia.unwrap_or(u64::MAX))
            .map(|i| i.name.clone())
            .unwrap_or_else(|| a.name.clone());
        let b_name = d
            .instances()
            .get(&ib.unwrap_or(u64::MAX))
            .map(|i| i.name.clone())
            .unwrap_or_else(|| b.name.clone());
        hits.push(InterferenceHit {
            a_instance: ia,
            b_instance: ib,
            a_name,
            b_name,
            a_entity: ea,
            b_entity: eb,
            volume,
            bbox: bbox_of(&mesh),
        });
    }

    Ok(CommandResult::Interference {
        doc,
        ok: hits.is_empty(),
        checked,
        tolerance,
        hits,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use wright_protocol::{Bbox, Command, EntitySnapshot, SVec3};

    use crate::Engine;

    static NEXT_FILE: AtomicUsize = AtomicUsize::new(0);

    fn temp_file(tag: &str) -> String {
        let n = NEXT_FILE.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir()
            .join(format!("wright_m19_{tag}_{}_{n}.wcad", std::process::id()))
            .to_string_lossy()
            .into_owned()
    }

    fn num(v: f64) -> Scalar {
        Scalar::Number(v)
    }

    fn vec3(v: [f64; 3]) -> SVec3 {
        [num(v[0]), num(v[1]), num(v[2])]
    }

    fn create(e: &mut Engine, name: &str) -> DocId {
        match e
            .execute(Command::CreateDocument {
                name: name.into(),
                doc: None,
            })
            .unwrap()
        {
            CommandResult::DocumentCreated { doc } => doc,
            r => panic!("unexpected {r:?}"),
        }
    }

    fn box_part(e: &mut Engine, doc: DocId, center: [f64; 3], size: [f64; 3]) -> EntityId {
        match e
            .execute(Command::AddBox {
                doc,
                name: "body".into(),
                center: vec3(center),
                size: vec3(size),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected {r:?}"),
        }
    }

    fn import_from(e: &mut Engine, doc: DocId, from: DocId, name: &str) -> (usize, usize) {
        match e
            .execute(Command::ImportPart {
                doc,
                name: Some(name.into()),
                path: None,
                from: Some(from),
            })
            .unwrap()
        {
            CommandResult::PartImported {
                solids, parameters, ..
            } => (solids, parameters),
            r => panic!("unexpected {r:?}"),
        }
    }

    fn add_instance(
        e: &mut Engine,
        doc: DocId,
        part: &str,
        name: &str,
        t: [f64; 3],
        params: &[(&str, f64)],
    ) -> u64 {
        match e
            .execute(Command::AddInstance {
                doc,
                part: part.into(),
                name: name.into(),
                translation: Some(vec3(t)),
                rotation_deg: None,
                scale: None,
                parameters: Some(
                    params
                        .iter()
                        .map(|(k, v)| (k.to_string(), num(*v)))
                        .collect(),
                ),
            })
            .unwrap()
        {
            CommandResult::InstanceCreated { instance, .. } => instance,
            r => panic!("unexpected {r:?}"),
        }
    }

    fn scene(e: &mut Engine, doc: DocId) -> Vec<EntitySnapshot> {
        match e.execute(Command::GetScene { doc }).unwrap() {
            CommandResult::Scene { entities, .. } => entities,
            r => panic!("unexpected {r:?}"),
        }
    }

    fn bbox_of(e: &mut Engine, doc: DocId, name: &str) -> Bbox {
        scene(e, doc)
            .into_iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("no entity named {name}"))
            .world_bbox
    }

    #[test]
    fn an_assembly_is_self_contained_after_save_and_close() {
        let mut e = Engine::new();
        let part_doc = create(&mut e, "plate part");
        box_part(&mut e, part_doc, [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);

        let asm = create(&mut e, "stack");
        assert_eq!(import_from(&mut e, asm, part_doc, "plate"), (1, 0));
        add_instance(&mut e, asm, "plate", "base", [0.0, 0.0, 0.0], &[]);

        let path = temp_file("assembly");
        e.execute(Command::SaveDocument {
            doc: asm,
            path: path.clone(),
            source: None,
        })
        .unwrap();
        // The source document and the file are no longer needed: the part
        // snapshot lives in the saved assembly program.
        e.execute(Command::CloseDocument { doc: part_doc }).unwrap();
        let reopened = match e
            .execute(Command::OpenDocument {
                path: path.clone(),
                name: None,
            })
            .unwrap()
        {
            CommandResult::DocumentOpened { doc, .. } => doc,
            r => panic!("unexpected {r:?}"),
        };
        let bbox = bbox_of(&mut e, reopened, "base.body");
        assert_eq!(bbox.min, [-10.0, -5.0, 0.0]);
        assert_eq!(bbox.max, [10.0, 5.0, 5.0]);

        let text = std::fs::read_to_string(&path).unwrap();
        assert!(
            text.contains("\"method\": \"add_part\""),
            "part step missing"
        );
        assert!(
            text.contains("\"method\": \"add_box\""),
            "embedded part program missing"
        );
        assert!(
            !text.contains(&path),
            "assembly must not reference its own path"
        );
        // The embedded part program uses doc 1; the outer program is also
        // normalized, so save -> open -> save is stable.
        let path2 = temp_file("assembly2");
        e.execute(Command::SaveDocument {
            doc: reopened,
            path: path2.clone(),
            source: None,
        })
        .unwrap();
        assert_eq!(
            std::fs::read(&path).unwrap(),
            std::fs::read(&path2).unwrap()
        );
        let _ = std::fs::remove_file(path);
        let _ = std::fs::remove_file(path2);
    }

    #[test]
    fn import_part_from_a_file_embeds_the_program() {
        let mut e = Engine::new();
        let part = create(&mut e, "file part");
        box_part(&mut e, part, [0.0, 0.0, 3.0], [8.0, 6.0, 6.0]);
        let path = temp_file("part");
        e.execute(Command::SaveDocument {
            doc: part,
            path: path.clone(),
            source: None,
        })
        .unwrap();
        e.execute(Command::CloseDocument { doc: part }).unwrap();

        let asm = create(&mut e, "from file");
        match e
            .execute(Command::ImportPart {
                doc: asm,
                name: Some("plate".into()),
                path: Some(path.clone()),
                from: None,
            })
            .unwrap()
        {
            CommandResult::PartImported {
                solids,
                parameters,
                bytes,
                ..
            } => {
                assert_eq!(solids, 1);
                assert_eq!(parameters, 0);
                assert!(bytes > 100);
            }
            r => panic!("unexpected {r:?}"),
        }
        add_instance(&mut e, asm, "plate", "one", [0.0, 0.0, 0.0], &[]);
        let b = bbox_of(&mut e, asm, "one.body");
        assert_eq!(b.min, [-4.0, -3.0, 0.0]);
        assert_eq!(b.max, [4.0, 3.0, 6.0]);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn instance_placement_combines_with_the_part_placement() {
        let mut e = Engine::new();
        let part = create(&mut e, "offset part");
        // The entity's translation (0,0,10) is part of its own placement.
        box_part(&mut e, part, [0.0, 0.0, 10.0], [10.0, 4.0, 2.0]);
        let asm = create(&mut e, "placed");
        import_from(&mut e, asm, part, "thing");
        match e
            .execute(Command::AddInstance {
                doc: asm,
                part: "thing".into(),
                name: "one".into(),
                translation: Some(vec3([100.0, 0.0, 0.0])),
                rotation_deg: Some(vec3([0.0, 0.0, 90.0])),
                scale: Some(num(2.0)),
                parameters: None,
            })
            .unwrap()
        {
            CommandResult::InstanceCreated { entities, .. } => assert_eq!(entities.len(), 1),
            r => panic!("unexpected {r:?}"),
        }
        // Rotated +90 about Z and scaled 2: the part's local (0,0,10)
        // translation stays on Z, the 10×4 footprint becomes 4×10 and doubles.
        let snap = scene(&mut e, asm)
            .into_iter()
            .find(|s| s.name == "one.body")
            .unwrap();
        assert_eq!(snap.instance, Some(1));
        assert_eq!(snap.part.as_deref(), Some("thing"));
        let b = bbox_of(&mut e, asm, "one.body");
        assert!((b.min[0] - 96.0).abs() < 1e-4 && (b.max[0] - 104.0).abs() < 1e-4);
        assert!((b.min[1] + 10.0).abs() < 1e-4 && (b.max[1] - 10.0).abs() < 1e-4);
        assert!((b.min[2] - 18.0).abs() < 1e-4 && (b.max[2] - 22.0).abs() < 1e-4);
    }

    #[test]
    fn mates_move_every_member_and_replay() {
        let mut e = Engine::new();
        let part = create(&mut e, "brick");
        box_part(&mut e, part, [0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
        let asm = create(&mut e, "mated");
        import_from(&mut e, asm, part, "brick");
        add_instance(&mut e, asm, "brick", "a", [0.0, 0.0, 0.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 1,
            name: "top".into(),
            point: vec3([0.0, 0.0, 10.0]),
            axis: Some([0.0, 0.0, 1.0]),
        })
        .unwrap();
        add_instance(&mut e, asm, "brick", "b", [50.0, 0.0, 0.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 2,
            name: "bottom".into(),
            point: vec3([0.0, 0.0, 0.0]),
            axis: Some([0.0, 0.0, -1.0]),
        })
        .unwrap();
        e.execute(Command::Mate {
            doc: asm,
            kind: MateKind::Coincident,
            a: MateRef {
                instance: 1,
                connector: "top".into(),
            },
            b: MateRef {
                instance: 2,
                connector: "bottom".into(),
            },
            offset: None,
        })
        .unwrap();
        let b = bbox_of(&mut e, asm, "b.body");
        assert!((b.min[2] - 10.0).abs() < 1e-4 && (b.max[2] - 20.0).abs() < 1e-4);
        assert!((b.min[0] + 5.0).abs() < 1e-4 && (b.max[0] - 5.0).abs() < 1e-4);

        // A later distance mate re-positions the same instance during a
        // full replay, and the saved file reproduces it.
        e.execute(Command::Mate {
            doc: asm,
            kind: MateKind::Distance,
            a: MateRef {
                instance: 1,
                connector: "top".into(),
            },
            b: MateRef {
                instance: 2,
                connector: "bottom".into(),
            },
            offset: Some(num(4.0)),
        })
        .unwrap();
        let b = bbox_of(&mut e, asm, "b.body");
        assert!((b.min[2] - 14.0).abs() < 1e-4 && (b.max[2] - 24.0).abs() < 1e-4);

        let path = temp_file("mated");
        e.execute(Command::SaveDocument {
            doc: asm,
            path: path.clone(),
            source: None,
        })
        .unwrap();
        let reopened = match e
            .execute(Command::OpenDocument {
                path: path.clone(),
                name: None,
            })
            .unwrap()
        {
            CommandResult::DocumentOpened { doc, .. } => doc,
            r => panic!("unexpected {r:?}"),
        };
        let b2 = bbox_of(&mut e, reopened, "b.body");
        assert!((b2.min[2] - 14.0).abs() < 1e-4 && (b2.max[2] - 24.0).abs() < 1e-4);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn mate_offset_expressions_replay_with_assembly_knobs() {
        let mut e = Engine::new();
        let part = create(&mut e, "mate part");
        box_part(&mut e, part, [0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
        let asm = create(&mut e, "mate knobs");
        e.execute(Command::SetParameter {
            doc: asm,
            name: "lift".into(),
            value: num(5.0),
            description: None,
        })
        .unwrap();
        import_from(&mut e, asm, part, "brick");
        add_instance(&mut e, asm, "brick", "base", [0.0, 0.0, 0.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 1,
            name: "top".into(),
            point: vec3([0.0, 0.0, 10.0]),
            axis: Some([0.0, 0.0, 1.0]),
        })
        .unwrap();
        add_instance(&mut e, asm, "brick", "upper", [0.0, 0.0, 100.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 2,
            name: "bottom".into(),
            point: vec3([0.0, 0.0, 0.0]),
            axis: Some([0.0, 0.0, -1.0]),
        })
        .unwrap();
        e.execute(Command::Mate {
            doc: asm,
            kind: MateKind::Distance,
            a: MateRef {
                instance: 1,
                connector: "top".into(),
            },
            b: MateRef {
                instance: 2,
                connector: "bottom".into(),
            },
            offset: Some(Scalar::Expression("lift".into())),
        })
        .unwrap();
        let b = bbox_of(&mut e, asm, "upper.body");
        assert!((b.min[2] - 15.0).abs() < 1e-4);
        e.execute(Command::SetParameter {
            doc: asm,
            name: "lift".into(),
            value: num(20.0),
            description: None,
        })
        .unwrap();
        let b = bbox_of(&mut e, asm, "upper.body");
        assert!((b.min[2] - 30.0).abs() < 1e-4);
    }

    #[test]
    fn angle_mate_rotates_b_about_its_connector() {
        let mut e = Engine::new();
        let part = create(&mut e, "bar");
        box_part(&mut e, part, [0.0, 0.0, 0.0], [20.0, 4.0, 10.0]);
        let asm = create(&mut e, "angled");
        import_from(&mut e, asm, part, "bar");
        add_instance(&mut e, asm, "bar", "a", [0.0, 0.0, 0.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 1,
            name: "up".into(),
            point: vec3([0.0, 0.0, 0.0]),
            axis: Some([0.0, 0.0, 1.0]),
        })
        .unwrap();
        add_instance(&mut e, asm, "bar", "b", [50.0, 0.0, 0.0], &[]);
        e.execute(Command::AddConnector {
            doc: asm,
            instance: 2,
            name: "up".into(),
            point: vec3([0.0, 0.0, 0.0]),
            axis: Some([0.0, 0.0, 1.0]),
        })
        .unwrap();
        e.execute(Command::Mate {
            doc: asm,
            kind: MateKind::Angle,
            a: MateRef {
                instance: 1,
                connector: "up".into(),
            },
            b: MateRef {
                instance: 2,
                connector: "up".into(),
            },
            offset: Some(num(90.0)),
        })
        .unwrap();
        let b = bbox_of(&mut e, asm, "b.body");
        // +Z rotates to +X: the 20 mm X length becomes the Z extent and the
        // 10 mm Z length becomes the X extent; the connector point moved to A.
        assert!((b.min[0] + 5.0).abs() < 1e-3 && (b.max[0] - 5.0).abs() < 1e-3);
        assert!((b.min[1] + 2.0).abs() < 1e-3 && (b.max[1] - 2.0).abs() < 1e-3);
        assert!((b.min[2] + 10.0).abs() < 1e-3 && (b.max[2] - 10.0).abs() < 1e-3);
    }

    #[test]
    fn instance_parameters_override_the_part_and_survive_replay() {
        let mut e = Engine::new();
        let part = create(&mut e, "variable plate");
        e.execute(Command::SetParameter {
            doc: part,
            name: "w".into(),
            value: num(20.0),
            description: None,
        })
        .unwrap();
        e.execute(Command::AddBox {
            doc: part,
            name: "body".into(),
            center: vec3([0.0, 0.0, 0.0]),
            size: [Scalar::Expression("w".into()), num(10.0), num(10.0)],
            color: None,
        })
        .unwrap();
        let asm = create(&mut e, "variants");
        import_from(&mut e, asm, part, "plate");
        add_instance(
            &mut e,
            asm,
            "plate",
            "wide",
            [0.0, 0.0, 0.0],
            &[("w", 50.0)],
        );
        let b = bbox_of(&mut e, asm, "wide.body");
        assert!((b.min[0] + 25.0).abs() < 1e-4 && (b.max[0] - 25.0).abs() < 1e-4);

        let path = temp_file("variant");
        e.execute(Command::SaveDocument {
            doc: asm,
            path: path.clone(),
            source: None,
        })
        .unwrap();
        let reopened = match e
            .execute(Command::OpenDocument {
                path: path.clone(),
                name: None,
            })
            .unwrap()
        {
            CommandResult::DocumentOpened { doc, .. } => doc,
            r => panic!("unexpected {r:?}"),
        };
        let b2 = bbox_of(&mut e, reopened, "wide.body");
        assert!((b2.min[0] + 25.0).abs() < 1e-4 && (b2.max[0] - 25.0).abs() < 1e-4);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn undo_and_redo_restore_parts_and_instances() {
        let mut e = Engine::new();
        let part = create(&mut e, "undo part");
        box_part(&mut e, part, [0.0, 0.0, 2.0], [4.0, 4.0, 4.0]);
        let asm = create(&mut e, "undo assembly");
        import_from(&mut e, asm, part, "cube");
        add_instance(&mut e, asm, "cube", "one", [0.0, 0.0, 0.0], &[]);
        assert_eq!(scene(&mut e, asm).len(), 1);
        e.execute(Command::Undo { doc: asm }).unwrap();
        assert!(scene(&mut e, asm).is_empty());
        match e.execute(Command::BillOfMaterials { doc: asm }).unwrap() {
            CommandResult::Bom { rows, .. } => assert!(rows.is_empty()),
            r => panic!("unexpected {r:?}"),
        }
        e.execute(Command::Redo { doc: asm }).unwrap();
        assert_eq!(scene(&mut e, asm).len(), 1);
        e.execute(Command::Undo { doc: asm }).unwrap(); // instance
        e.execute(Command::Undo { doc: asm }).unwrap(); // part definition
        assert!(scene(&mut e, asm).is_empty());
        // With the part gone, placing it is a clear error rather than a
        // half-restored instance.
        let err = e
            .execute(Command::AddInstance {
                doc: asm,
                part: "cube".into(),
                name: "two".into(),
                translation: None,
                rotation_deg: None,
                scale: None,
                parameters: None,
            })
            .unwrap_err()
            .to_string();
        assert!(err.contains("unknown part 'cube'"), "{err}");
    }

    #[test]
    fn bom_groups_instances_by_part() {
        let mut e = Engine::new();
        let part = create(&mut e, "washer");
        box_part(&mut e, part, [0.0, 0.0, 1.0], [10.0, 10.0, 2.0]);
        let asm = create(&mut e, "washers");
        import_from(&mut e, asm, part, "washer");
        add_instance(&mut e, asm, "washer", "w1", [0.0, 0.0, 0.0], &[]);
        add_instance(&mut e, asm, "washer", "w2", [20.0, 0.0, 0.0], &[]);
        match e.execute(Command::BillOfMaterials { doc: asm }).unwrap() {
            CommandResult::Bom { rows, .. } => {
                assert_eq!(rows.len(), 1);
                assert_eq!(rows[0].item, 1);
                assert_eq!(rows[0].part, "washer");
                assert_eq!(rows[0].quantity, 2);
                assert_eq!(rows[0].entities, 2);
                assert!((rows[0].volume - 400.0).abs() < 1e-6);
                assert_eq!(rows[0].instances, vec!["w1", "w2"]);
            }
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn interference_reports_overlap_and_clears_separation() {
        let mut e = Engine::new();
        let doc = create(&mut e, "overlap");
        box_part(&mut e, doc, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        box_part(&mut e, doc, [5.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        match e
            .execute(Command::CheckInterference {
                doc,
                instances: None,
                entities: Some(vec![1, 2]),
                tolerance: None,
            })
            .unwrap()
        {
            CommandResult::Interference {
                ok, checked, hits, ..
            } => {
                assert!(!ok);
                assert_eq!(checked, 1);
                assert_eq!(hits.len(), 1);
                assert!((hits[0].volume - 500.0).abs() < 1e-6);
                assert_eq!(hits[0].bbox.min, [0.0, -5.0, -5.0]);
            }
            r => panic!("unexpected {r:?}"),
        }
        e.execute(Command::Transform {
            doc,
            entity: 2,
            translation: Some(vec3([50.0, 0.0, 0.0])),
            rotation_deg: None,
            scale: None,
        })
        .unwrap();
        match e
            .execute(Command::CheckInterference {
                doc,
                instances: None,
                entities: Some(vec![1, 2]),
                tolerance: None,
            })
            .unwrap()
        {
            CommandResult::Interference {
                ok, checked, hits, ..
            } => {
                assert!(ok);
                assert_eq!(checked, 0, "AABB pruning should skip separated solids");
                assert!(hits.is_empty());
            }
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn default_interference_checks_instance_pairs() {
        let mut e = Engine::new();
        let part = create(&mut e, "block");
        box_part(&mut e, part, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        let asm = create(&mut e, "overlapping assembly");
        import_from(&mut e, asm, part, "block");
        add_instance(&mut e, asm, "block", "left", [0.0, 0.0, 0.0], &[]);
        add_instance(&mut e, asm, "block", "right", [6.0, 0.0, 0.0], &[]);
        match e
            .execute(Command::CheckInterference {
                doc: asm,
                instances: None,
                entities: None,
                tolerance: None,
            })
            .unwrap()
        {
            CommandResult::Interference { ok, hits, .. } => {
                assert!(!ok);
                assert_eq!(hits.len(), 1);
                assert_eq!(hits[0].a_instance, Some(1));
                assert_eq!(hits[0].b_instance, Some(2));
                assert_eq!(hits[0].a_name, "left");
                assert_eq!(hits[0].b_name, "right");
                assert!((hits[0].volume - 400.0).abs() < 1e-6);
            }
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn instance_parameter_expressions_follow_assembly_knobs() {
        let mut e = Engine::new();
        let part = create(&mut e, "cube");
        box_part(&mut e, part, [0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
        let asm = create(&mut e, "driven");
        e.execute(Command::SetParameter {
            doc: asm,
            name: "gap".into(),
            value: num(10.0),
            description: None,
        })
        .unwrap();
        import_from(&mut e, asm, part, "cube");
        match e
            .execute(Command::AddInstance {
                doc: asm,
                part: "cube".into(),
                name: "c".into(),
                translation: Some([Scalar::Expression("gap*2".into()), num(0.0), num(0.0)]),
                rotation_deg: None,
                scale: None,
                parameters: None,
            })
            .unwrap()
        {
            CommandResult::InstanceCreated { .. } => {}
            r => panic!("unexpected {r:?}"),
        }
        let before = bbox_of(&mut e, asm, "c.body");
        assert!((before.min[0] - 18.0).abs() < 1e-4);
        e.execute(Command::SetParameter {
            doc: asm,
            name: "gap".into(),
            value: num(30.0),
            description: None,
        })
        .unwrap();
        let after = bbox_of(&mut e, asm, "c.body");
        assert!((after.min[0] - 58.0).abs() < 1e-4);
    }

    #[test]
    fn nested_assemblies_flatten_into_members() {
        let mut e = Engine::new();
        let part = create(&mut e, "leg");
        box_part(&mut e, part, [0.0, 0.0, 5.0], [4.0, 4.0, 10.0]);

        let sub = create(&mut e, "pair");
        import_from(&mut e, sub, part, "leg");
        add_instance(&mut e, sub, "leg", "left", [0.0, 0.0, 0.0], &[]);
        add_instance(&mut e, sub, "leg", "right", [20.0, 0.0, 0.0], &[]);

        let top = create(&mut e, "stacked");
        import_from(&mut e, top, sub, "pair");
        add_instance(&mut e, top, "pair", "top", [0.0, 0.0, 100.0], &[]);
        let snapshots = scene(&mut e, top);
        assert_eq!(snapshots.len(), 2);
        assert!(snapshots.iter().any(|s| s.name == "top.left.body"));
        assert!(snapshots.iter().any(|s| s.name == "top.right.body"));
        match e.execute(Command::BillOfMaterials { doc: top }).unwrap() {
            CommandResult::Bom { rows, .. } => {
                assert_eq!(rows.len(), 1);
                assert_eq!(rows[0].part, "pair");
                assert_eq!(rows[0].entities, 2);
            }
            r => panic!("unexpected {r:?}"),
        }
    }

    #[test]
    fn unknown_parts_and_overrides_are_rejected_with_hints() {
        let mut e = Engine::new();
        let doc = create(&mut e, "empty assembly");
        let err = e
            .execute(Command::AddInstance {
                doc,
                part: "missing".into(),
                name: "x".into(),
                translation: None,
                rotation_deg: None,
                scale: None,
                parameters: None,
            })
            .unwrap_err()
            .to_string();
        assert!(err.contains("unknown part 'missing'"), "{err}");
        let part = create(&mut e, "part");
        box_part(&mut e, part, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        import_from(&mut e, doc, part, "ok");
        let err = e
            .execute(Command::AddInstance {
                doc,
                part: "ok".into(),
                name: "x".into(),
                translation: None,
                rotation_deg: None,
                scale: None,
                parameters: Some([("nope".to_string(), num(1.0))].into_iter().collect()),
            })
            .unwrap_err()
            .to_string();
        assert!(err.contains("has no parameter 'nope'"), "{err}");
    }
}
