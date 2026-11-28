//! Measurement and mass properties (M24): the answers an agent should ask
//! instead of estimating dimensions from a screenshot.
//!
//! * **Distance** is exact for two points and the true minimum
//!   surface-to-surface distance for planar faces and solids. A deterministic
//!   triangle BVH keeps the query fast; the closest points travel with the
//!   number, so a fit can be steered instead of guessed.
//! * **Angle** uses face normals: the acute plane angle and the normal angle
//!   are both reported. For two points the segment direction's angles to the
//!   world axes are reported.
//! * **Section area** integrates the oriented plane cut of a closed solid.
//!   Holes subtract because inner surfaces wind the other way.
//! * **Mass properties** integrate volume, surface area, centroid and inertia
//!   from the world-space triangles (signed tetrahedra + the standard
//!   covariance formula). Density comes from `set_material`, a report
//!   override or an explicit g/cm³ value.
//! * **Clearance** combines the closest approach with the M23 fit classes so
//!   "does this pin fit?" is one number plus a class table.
//!
//! Everything here is deterministic: mesh order drives the BVH, sorts break
//! ties by triangle index, and there is no randomness, clocks or hash-order
//! dependency.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::{
    AssemblyMassProperties, Bbox, ClearanceClassInfo, ClearanceReport, ClosestApproach,
    CommandResult, DocId, EntityId, FitClass, MassPropertiesReport, MeasureReport,
    MeasureTarget, MeasureTargetInfo, Mesh, SectionReport, SolidMassProperties, Vec3,
};

use crate::edges;
use crate::plane::{self, Frame};
use crate::{Document, Error, Result, Shape};

/// Triangles per BVH leaf. Small enough for cheap exact triangle pairs, large
/// enough that the tree stays shallow on ordinary parts.
const LEAF_TRIS: usize = 8;
/// Absolute floor for treating a distance as zero.
const ZERO_MM: f64 = 1e-9;

// --- public entry points -------------------------------------------------------

/// Run [`wright_protocol::Command::Measure`].
pub fn measure(
    d: &Document,
    doc: DocId,
    a: &MeasureTarget,
    b: Option<&MeasureTarget>,
    section_plane: Option<&Frame>,
) -> Result<CommandResult> {
    if b.is_none() && section_plane.is_none() {
        return Err(Error::Invalid(
            "measure needs a second target (`b`) or a `plane` for a cross-section (hint: pass \
             two points, faces or solids, or one solid plus plane=\"xy\")"
                .into(),
        ));
    }

    let a = resolve(d, doc, a)?;
    let b = b.map(|target| resolve(d, doc, target)).transpose()?;
    let mut warnings = Vec::new();

    let mut report = MeasureReport {
        doc,
        a: a.info.clone(),
        b: b.as_ref().map(|b| b.info.clone()),
        distance: None,
        delta: None,
        closest: None,
        plane_distance: None,
        angle_deg: None,
        normal_angle_deg: None,
        direction: None,
        angles_to_axes: None,
        section: None,
        warnings: Vec::new(),
    };

    if let Some(b) = &b {
        report.delta = Some(edges::sub(b.info.point, a.info.point));
        let (distance, closest) = distance_between(&a, b)?;
        report.distance = Some(distance);
        if let Some((pa, pb)) = closest {
            let direction = unit(edges::sub(pb, pa));
            report.closest = Some(ClosestApproach {
                a: pa,
                b: pb,
                distance,
                direction: if direction[0].is_nan() { None } else { Some(direction) },
            });
            if distance > ZERO_MM {
                report.direction = Some(edges::normalize(edges::sub(pb, pa)));
            }
        }

        if a.kind == TargetKind::Point && b.kind == TargetKind::Point && distance > ZERO_MM {
            let dir = edges::normalize(edges::sub(b.point, a.point));
            report.direction = Some(dir);
            report.angles_to_axes = Some([
                angle_deg(dir, [1.0, 0.0, 0.0]),
                angle_deg(dir, [0.0, 1.0, 0.0]),
                angle_deg(dir, [0.0, 0.0, 1.0]),
            ]);
        }

        if let (Some(na), Some(nb)) = (a.normal, b.normal) {
            let normal_angle = angle_deg(na, nb);
            report.normal_angle_deg = Some(normal_angle);
            report.angle_deg = Some(normal_angle.min(180.0 - normal_angle));
            if edges::dot(na, nb).abs() >= 1.0 - 1e-9 {
                let offset = edges::sub(b.info.point, a.info.point);
                report.plane_distance = Some(edges::dot(na, offset).abs());
            }
        }
    }

    if let Some(frame) = section_plane {
        if a.kind == TargetKind::Point {
            return Err(Error::Invalid(
                "a cross-section needs a face or solid target (a point has no area; hint: pass \
                 the solid entity id)"
                    .into(),
            ));
        }
        let mesh = a.mesh.as_ref().ok_or_else(|| {
            Error::Invalid("the section target has no surface geometry".into())
        })?;
        let report_section = section_report(mesh, frame)?;
        if report_section.coplanar_tris > 0 && report_section.area <= ZERO_MM {
            warnings.push(format!(
                "the section plane is coplanar with {} triangle(s); a plane coincident with a \
                 face is not a cut, so the area is 0",
                report_section.coplanar_tris
            ));
        }
        report.section = Some(report_section);
    }

    report.warnings = warnings;
    Ok(CommandResult::Measured(report))
}

/// Run [`wright_protocol::Command::MassProperties`].
pub fn mass_properties(
    d: &Document,
    doc: DocId,
    entities: Option<&[EntityId]>,
    material: Option<&str>,
    density_g_cm3: Option<f64>,
) -> Result<CommandResult> {
    if let Some(density) = density_g_cm3 {
        validate_density(density)?;
    }

    let override_spec = match material {
        Some(name) => Some(crate::print::resolve_material(name, density_g_cm3, None)?),
        None => None,
    };
    let override_density = override_spec
        .as_ref()
        .map(|m| m.density_g_cm3)
        .or(density_g_cm3);

    let ids: Vec<EntityId> = match entities {
        Some(ids) => {
            let set: BTreeSet<EntityId> = ids.iter().copied().collect();
            set.into_iter().collect()
        }
        None => d.entities().keys().copied().collect(),
    };
    if ids.is_empty() {
        return Err(Error::Invalid(
            "mass_properties has no solids to measure (hint: the document has no entities yet)"
                .into(),
        ));
    }

    let mut solids = Vec::with_capacity(ids.len());
    let mut warnings = Vec::new();
    for id in ids {
        let e = d
            .entities()
            .get(&id)
            .ok_or(Error::UnknownEntity { doc, entity: id })?;
        if matches!(e.shape, Shape::Sketch { .. }) {
            return Err(Error::Invalid(format!(
                "entity {id} is a sketch; mass_properties measures printable solids (hint: \
                 extrude the sketch first, or pass entities=[...] with solids only)"
            )));
        }
        if e.mesh.indices.is_empty() {
            return Err(Error::Invalid(format!(
                "entity {id} has no geometry to measure"
            )));
        }
        let world = crate::transform_mesh(&e.mesh, &e.transform());
        let props = mesh_mass_properties(&world);
        let assigned = d.materials().get(&id);
        let (material_name, density) = match &override_spec {
            Some(spec) => (Some(spec.name.clone()), override_density),
            None => (
                assigned.map(|m| m.name.clone()),
                override_density.or_else(|| assigned.map(|m| m.density_g_cm3)),
            ),
        };
        let mass_g = density.map(|rho| props.volume / 1000.0 * rho);
        let inertia = density.map(|rho| scale_tensor(props.inertia_unit, rho / 1000.0));
        if density.is_none() {
            warnings.push(format!(
                "entity {id} ('{}') has no material density; mass and inertia are omitted",
                e.name
            ));
        }
        solids.push(SolidMassProperties {
            entity: id,
            name: e.name.clone(),
            kind: e.shape.kind().to_string(),
            volume: props.volume,
            surface_area: props.surface_area,
            centroid: props.centroid,
            bbox: props.bbox,
            tris: props.tris,
            material: material_name,
            density_g_cm3: density,
            mass_g,
            inertia,
        });
    }

    let assembly = assemble(&solids);
    if assembly.unassigned > 0 {
        warnings.push(format!(
            "{} of {} solids have no density; assembly mass and inertia are omitted (hint: \
             set_material, or pass material='pla' or density_g_cm3)",
            assembly.unassigned,
            assembly.solids
        ));
    }

    Ok(CommandResult::MassPropertiesMeasured(MassPropertiesReport {
        doc,
        solids,
        assembly,
        material: override_spec.as_ref().map(|m| m.name.clone()),
        density_g_cm3: override_density,
        warnings,
    }))
}

/// Run [`wright_protocol::Command::Clearance`].
#[allow(clippy::too_many_arguments)]
pub fn clearance(
    d: &Document,
    doc: DocId,
    a: EntityId,
    b: EntityId,
    class: Option<FitClass>,
    material: Option<&str>,
    printer_tolerance_mm: Option<f64>,
    hole_shrink_mm: Option<f64>,
    tolerance: Option<f64>,
) -> Result<CommandResult> {
    if a == b {
        return Err(Error::Invalid(
            "clearance needs two different solids (hint: measure a part against another part, \
             not against itself)"
                .into(),
        ));
    }
    let tolerance = tolerance.unwrap_or(1e-6);
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(Error::Invalid("clearance tolerance must be finite and >= 0".into()));
    }
    let printer_tolerance = printer_tolerance_mm.unwrap_or(0.0);
    if !printer_tolerance.is_finite() || printer_tolerance < 0.0 {
        return Err(Error::Invalid(
            "printer_tolerance_mm must be finite and >= 0".into(),
        ));
    }
    if let Some(v) = hole_shrink_mm {
        if !v.is_finite() {
            return Err(Error::Invalid("hole_shrink_mm must be finite".into()));
        }
    }
    let (material_name, default_shrink, elephant) =
        crate::print::print_profile(material)?;
    let shrink = hole_shrink_mm.unwrap_or(default_shrink);

    let ea = d
        .entities()
        .get(&a)
        .ok_or(Error::UnknownEntity { doc, entity: a })?;
    let eb = d
        .entities()
        .get(&b)
        .ok_or(Error::UnknownEntity { doc, entity: b })?;
    for (entity, e) in [(a, ea), (b, eb)] {
        if matches!(e.shape, Shape::Sketch { .. }) {
            return Err(Error::Invalid(format!(
                "entity {entity} is a sketch; clearance needs printable solids (hint: extrude \
                 it first)"
            )));
        }
        if e.mesh.indices.is_empty() {
            return Err(Error::Invalid(format!(
                "entity {entity} has no geometry to measure"
            )));
        }
    }

    let mesh_a = crate::transform_mesh(&ea.mesh, &ea.transform());
    let mesh_b = crate::transform_mesh(&eb.mesh, &eb.transform());
    let surface_a = Surface::new(&mesh_a)?;
    let surface_b = Surface::new(&mesh_b)?;
    let hit = surface_a.closest_to(&surface_b).ok_or_else(|| {
        Error::Invalid("the solids have no triangles to measure (hint: check their geometry)".into())
    })?;
    let distance = hit.0;
    let direction = unit(edges::sub(hit.2, hit.1));

    let mut warnings = Vec::new();
    let mut overlap_volume = 0.0;
    let ba = bbox_of(&mesh_a);
    let bb = bbox_of(&mesh_b);
    if aabb_overlap(&ba, &bb) {
        match crate::boolean::boolean(
            wright_protocol::BooleanOp::Intersection,
            &[mesh_a.clone(), mesh_b.clone()],
        ) {
            Ok(intersection) => {
                overlap_volume = crate::fillet::signed_volume(&intersection).abs();
            }
            Err(_) => warnings.push(
                "the solids' bounding boxes overlap but the boolean kernel could not intersect \
                 them (an open or non-manifold mesh); interference is not confirmed"
                    .into(),
            ),
        }
    }
    let interference = overlap_volume > 1e-6;

    let mut classes = Vec::with_capacity(4);
    for c in FitClass::all() {
        let required = c.radial_clearance_mm() + printer_tolerance + shrink;
        let delta = distance - required;
        classes.push(ClearanceClassInfo {
            class: c,
            required_radial_mm: required,
            measured_radial_mm: distance,
            delta_mm: delta,
            fits: delta >= -tolerance,
            description: c.description().to_string(),
        });
    }
    let nearest_class = classes
        .iter()
        .min_by(|x, y| {
            x.delta_mm
                .abs()
                .total_cmp(&y.delta_mm.abs())
                .then_with(|| class_rank(x.class).cmp(&class_rank(y.class)))
        })
        .map(|c| c.class)
        .unwrap_or(FitClass::Snug);
    let fit = class.and_then(|requested| {
        classes
            .iter()
            .find(|c| c.class == requested)
            .cloned()
    });
    if interference {
        warnings.push(
            "the solids overlap volumetrically; the measured gap is 0 and a press class is \
             approximate because penetration depth is not measured"
                .into(),
        );
    }

    Ok(CommandResult::ClearanceMeasured(ClearanceReport {
        doc,
        a,
        a_name: ea.name.clone(),
        b,
        b_name: eb.name.clone(),
        distance,
        closest: Some(ClosestApproach {
            a: hit.1,
            b: hit.2,
            distance,
            direction: if direction[0].is_nan() { None } else { Some(direction) },
        }),
        contact: distance <= tolerance,
        interference,
        overlap_volume,
        fit,
        nearest_class,
        classes,
        material: Some(material_name),
        printer_tolerance_mm: printer_tolerance,
        hole_shrink_mm: shrink,
        elephant_foot_mm: elephant,
        warnings,
    }))
}

// --- target resolution ----------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TargetKind {
    Point,
    Face,
    Solid,
}

struct Resolved {
    kind: TargetKind,
    info: MeasureTargetInfo,
    point: Vec3,
    normal: Option<Vec3>,
    surface: Option<Surface>,
    /// World mesh to cut for a section. A face target cuts only the patch.
    mesh: Option<Mesh>,
}

fn resolve(d: &Document, doc: DocId, target: &MeasureTarget) -> Result<Resolved> {
    match target {
        MeasureTarget::Point { point } => {
            validate_point(*point, "point")?;
            Ok(Resolved {
                kind: TargetKind::Point,
                info: MeasureTargetInfo {
                    kind: "point".into(),
                    entity: None,
                    name: None,
                    point: *point,
                    normal: None,
                    area: None,
                    volume: None,
                    bbox: None,
                },
                point: *point,
                normal: None,
                surface: None,
                mesh: None,
            })
        }
        MeasureTarget::Face {
            entity,
            normal,
            near,
            angle_tol_deg,
        } => resolve_face(d, doc, *entity, *normal, *near, *angle_tol_deg),
        MeasureTarget::Solid { entity } => resolve_solid(d, doc, *entity),
    }
}

fn resolve_solid(d: &Document, doc: DocId, entity: EntityId) -> Result<Resolved> {
    let e = d
        .entities()
        .get(&entity)
        .ok_or(Error::UnknownEntity { doc, entity })?;
    ensure_printable(e, entity, "measure")?;
    let world = crate::transform_mesh(&e.mesh, &e.transform());
    let props = mesh_mass_properties(&world);
    Ok(Resolved {
        kind: TargetKind::Solid,
        info: MeasureTargetInfo {
            kind: "solid".into(),
            entity: Some(entity),
            name: Some(e.name.clone()),
            point: props.centroid,
            normal: None,
            area: None,
            volume: Some(props.volume),
            bbox: Some(props.bbox),
        },
        point: props.centroid,
        normal: None,
        surface: Some(Surface::new(&world)?),
        mesh: Some(world),
    })
}

fn resolve_face(
    d: &Document,
    doc: DocId,
    entity: EntityId,
    normal: Option<Vec3>,
    near: Option<Vec3>,
    angle_tol_deg: Option<f64>,
) -> Result<Resolved> {
    let e = d
        .entities()
        .get(&entity)
        .ok_or(Error::UnknownEntity { doc, entity })?;
    ensure_printable(e, entity, "measure")?;
    if let Some(p) = near {
        validate_point(p, "face near")?;
    }
    let want = normal
        .map(|n| {
            validate_normal(n, "face normal")?;
            Ok(edges::normalize(n))
        })
        .transpose()?;
    let tol = angle_tol_deg.unwrap_or(1.0);
    if !(tol > 0.0 && tol <= 90.0) {
        return Err(Error::Invalid(
            "face angle_tol_deg must be in (0, 90] (hint: omit it for 1°)".into(),
        ));
    }
    let min_cos = tol.to_radians().cos();

    let world = crate::transform_mesh(&e.mesh, &e.transform());
    let faces = plane::planar_faces(&world)?;
    if faces.is_empty() {
        return Err(Error::Invalid(format!(
            "entity {entity} has no planar face to measure (hint: measure the solid itself, or \
             pick a flat face)"
        )));
    }
    let candidates: Vec<&plane::PlanarFace> = faces
        .iter()
        .filter(|f| want.is_none_or(|w| edges::dot(f.normal, w) >= min_cos))
        .collect();
    if candidates.is_empty() {
        let mut normals: Vec<String> = faces
            .iter()
            .map(|f| {
                format!(
                    "[{:.2}, {:.2}, {:.2}]",
                    f.normal[0], f.normal[1], f.normal[2]
                )
            })
            .collect();
        normals.sort();
        normals.dedup();
        let wanted = want.expect("candidates are only empty when a normal was given");
        return Err(Error::Invalid(format!(
            "entity {entity} has no face with outward normal [{:.2}, {:.2}, {:.2}] (hint: \
             available face normals: {})",
            wanted[0],
            wanted[1],
            wanted[2],
            normals.join(", ")
        )));
    }
    let chosen = match near {
        Some(p) => candidates
            .iter()
            .copied()
            .min_by(|a, b| {
                let da = (edges::dot(a.normal, p) - a.offset).abs();
                let db = (edges::dot(b.normal, p) - b.offset).abs();
                da.total_cmp(&db)
                    .then_with(|| {
                        let ca = edges::sub(a.centroid, p);
                        let cb = edges::sub(b.centroid, p);
                        edges::dot(ca, ca).total_cmp(&edges::dot(cb, cb))
                    })
                    .then_with(|| b.area.total_cmp(&a.area))
            })
            .expect("candidates is non-empty"),
        None => candidates
            .iter()
            .copied()
            .max_by(|a, b| a.area.total_cmp(&b.area))
            .expect("candidates is non-empty"),
    };

    let mut indices = Vec::with_capacity(chosen.triangles.len() * 3);
    for &t in &chosen.triangles {
        if let Some(tri) = world.indices.get(t * 3..t * 3 + 3) {
            indices.extend_from_slice(tri);
        }
    }
    let patch = Mesh {
        positions: world.positions.clone(),
        normals: world.normals.clone(),
        indices,
    };
    let patch_bbox = bbox_of_indices(&patch);
    Ok(Resolved {
        kind: TargetKind::Face,
        info: MeasureTargetInfo {
            kind: "face".into(),
            entity: Some(entity),
            name: Some(e.name.clone()),
            point: chosen.centroid,
            normal: Some(chosen.normal),
            area: Some(chosen.area),
            volume: None,
            bbox: Some(patch_bbox),
        },
        point: chosen.centroid,
        normal: Some(chosen.normal),
        surface: Some(Surface::new(&patch)?),
        mesh: Some(patch),
    })
}

fn ensure_printable(e: &crate::Entity, entity: EntityId, what: &str) -> Result<()> {
    if matches!(e.shape, Shape::Sketch { .. }) {
        return Err(Error::Invalid(format!(
            "entity {entity} is a sketch; {what} needs a printable solid (hint: extrude the \
             sketch first)"
        )));
    }
    if e.mesh.indices.is_empty() {
        return Err(Error::Invalid(format!(
            "entity {entity} has no geometry to {what}"
        )));
    }
    Ok(())
}

/// Return the closest points `(distance, on_a, on_b)` for the target pair.
fn distance_between(a: &Resolved, b: &Resolved) -> Result<(f64, Option<(Vec3, Vec3)>)> {
    match (&a.surface, &b.surface) {
        (None, None) => Ok((
            length(edges::sub(b.point, a.point)),
            Some((a.point, b.point)),
        )),
        (None, Some(surface)) => {
            let (point, distance) = surface.closest_to_point(a.point).ok_or_else(|| {
                Error::Invalid("the surface has no triangles to measure".into())
            })?;
            Ok((distance, Some((a.point, point))))
        }
        (Some(surface), None) => {
            let (point, distance) = surface.closest_to_point(b.point).ok_or_else(|| {
                Error::Invalid("the surface has no triangles to measure".into())
            })?;
            Ok((distance, Some((point, b.point))))
        }
        (Some(sa), Some(sb)) => {
            let (distance, pa, pb) = sa.closest_to(sb).ok_or_else(|| {
                Error::Invalid("the surfaces have no triangles to measure".into())
            })?;
            Ok((distance, Some((pa, pb))))
        }
    }
}

fn validate_point(p: Vec3, what: &str) -> Result<()> {
    if !p.iter().all(|v| v.is_finite()) {
        return Err(Error::Invalid(format!("{what} must be a finite point")));
    }
    Ok(())
}

fn validate_normal(n: Vec3, what: &str) -> Result<()> {
    validate_point(n, what)?;
    if length(n) < 1e-12 {
        return Err(Error::Invalid(format!(
            "{what} must be non-zero (e.g. [0,0,1])"
        )));
    }
    Ok(())
}

fn validate_density(density: f64) -> Result<()> {
    if !density.is_finite() || density <= 0.0 {
        return Err(Error::Invalid(
            "density_g_cm3 must be a finite value > 0 (e.g. 1.24 for PLA)".into(),
        ));
    }
    Ok(())
}

// --- mass properties ------------------------------------------------------------

struct MeshProperties {
    volume: f64,
    surface_area: f64,
    centroid: Vec3,
    bbox: Bbox,
    tris: usize,
    /// Inertia about the centroid for density 1 g/mm³, in mm⁵ (multiply by
    /// density/1000 to get g·mm²).
    inertia_unit: [[f64; 3]; 3],
}

fn mesh_mass_properties(mesh: &Mesh) -> MeshProperties {
    let tris = mesh.indices.len() / 3;
    let bbox = bbox_of(mesh);
    let mut volume = 0.0;
    let mut surface_area = 0.0;
    let mut centroid = [0.0; 3];
    let mut covariance = [[0.0f64; 3]; 3];

    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = match (
            mesh.positions.get(tri[0] as usize),
            mesh.positions.get(tri[1] as usize),
            mesh.positions.get(tri[2] as usize),
        ) {
            (Some(a), Some(b), Some(c)) => ([a[0] as f64, a[1] as f64, a[2] as f64],
                [b[0] as f64, b[1] as f64, b[2] as f64],
                [c[0] as f64, c[1] as f64, c[2] as f64]),
            _ => continue,
        };
        let ab = edges::sub(b, a);
        let ac = edges::sub(c, a);
        surface_area += 0.5 * length(edges::cross(ab, ac));

        let v = edges::dot(a, edges::cross(b, c)) / 6.0;
        volume += v;
        let sum = [a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]];
        for k in 0..3 {
            centroid[k] += v * sum[k] / 4.0;
        }
        // Covariance of the tetrahedron (0, a, b, c): V/20 * (aa + bb + cc + ss).
        for i in 0..3 {
            for j in 0..3 {
                covariance[i][j] += v / 20.0
                    * (a[i] * a[j]
                        + b[i] * b[j]
                        + c[i] * c[j]
                        + sum[i] * sum[j]);
            }
        }
    }

    if volume.abs() > 1e-12 {
        for c in &mut centroid {
            *c /= volume;
        }
    } else {
        centroid = [
            (bbox.min[0] + bbox.max[0]) / 2.0,
            (bbox.min[1] + bbox.max[1]) / 2.0,
            (bbox.min[2] + bbox.max[2]) / 2.0,
        ];
    }

    // Inertia about the origin: trace(C) * I - C.
    let trace = covariance[0][0] + covariance[1][1] + covariance[2][2];
    let mut inertia = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            inertia[i][j] = (if i == j { trace } else { 0.0 }) - covariance[i][j];
        }
    }
    // Shift to the centroid and make the sign follow the (positive) volume.
    let c = centroid;
    let c2 = edges::dot(c, c);
    for i in 0..3 {
        for j in 0..3 {
            let shift = (if i == j { c2 } else { 0.0 }) - c[i] * c[j];
            inertia[i][j] -= volume * shift;
        }
    }
    let sign = if volume < 0.0 { -1.0 } else { 1.0 };
    for row in &mut inertia {
        for value in row {
            *value *= sign;
        }
    }

    MeshProperties {
        volume: volume.abs(),
        surface_area,
        centroid,
        bbox,
        tris,
        inertia_unit: inertia,
    }
}

fn assemble(solids: &[SolidMassProperties]) -> AssemblyMassProperties {
    let mut volume = 0.0;
    let mut surface_area = 0.0;
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    let mut centroid_sum = [0.0; 3];
    let mut materials = BTreeSet::new();
    let mut unassigned = 0usize;
    let mut mass_sum = 0.0;
    let mut com_sum = [0.0; 3];
    let mut inertia_sum = [[0.0f64; 3]; 3];

    for s in solids {
        volume += s.volume;
        surface_area += s.surface_area;
        for k in 0..3 {
            lo[k] = lo[k].min(s.bbox.min[k]);
            hi[k] = hi[k].max(s.bbox.max[k]);
            centroid_sum[k] += s.volume * s.centroid[k];
        }
        if let Some(material) = &s.material {
            materials.insert(material.clone());
        }
        match (s.mass_g, s.density_g_cm3, s.inertia) {
            (Some(mass), Some(_density), Some(inertia)) => {
                mass_sum += mass;
                for k in 0..3 {
                    com_sum[k] += mass * s.centroid[k];
                }
                for i in 0..3 {
                    for j in 0..3 {
                        inertia_sum[i][j] += inertia[i][j];
                    }
                }
            }
            _ => unassigned += 1,
        }
    }

    let bbox = if solids.is_empty() {
        Bbox { min: [0.0; 3], max: [0.0; 3] }
    } else {
        Bbox { min: lo, max: hi }
    };
    let centroid = if volume.abs() > 1e-12 {
        Some([centroid_sum[0] / volume, centroid_sum[1] / volume, centroid_sum[2] / volume])
    } else {
        None
    };

    let density_complete = unassigned == 0 && !solids.is_empty();
    let (mass_g, center_of_mass, inertia) = if density_complete && mass_sum > 1e-12 {
        let com = [com_sum[0] / mass_sum, com_sum[1] / mass_sum, com_sum[2] / mass_sum];
        // Parallel axis: I_total = Σ I_i + m_i * (|d|² I - d dᵀ).
        for s in solids {
            if let (Some(mass), Some(_)) = (s.mass_g, s.density_g_cm3) {
                let d = edges::sub(s.centroid, com);
                let d2 = edges::dot(d, d);
                for i in 0..3 {
                    for j in 0..3 {
                        let shift = (if i == j { d2 } else { 0.0 }) - d[i] * d[j];
                        inertia_sum[i][j] += mass * shift;
                    }
                }
            }
        }
        (Some(mass_sum), Some(com), Some(inertia_sum))
    } else if density_complete {
        (Some(mass_sum), centroid, None)
    } else {
        (None, None, None)
    };

    AssemblyMassProperties {
        solids: solids.len(),
        volume,
        surface_area,
        bbox,
        centroid,
        center_of_mass,
        mass_g,
        inertia,
        materials: materials.into_iter().collect(),
        unassigned,
    }
}

fn scale_tensor(t: [[f64; 3]; 3], factor: f64) -> [[f64; 3]; 3] {
    let mut out = t;
    for row in &mut out {
        for value in row {
            *value *= factor;
        }
    }
    out
}

// --- cross-section --------------------------------------------------------------

struct Segment {
    a: [f64; 2],
    b: [f64; 2],
    wa: Vec3,
    wb: Vec3,
}

fn section_report(mesh: &Mesh, frame: &Frame) -> Result<SectionReport> {
    let (positions, tris) = edges::weld(mesh)?;
    let bbox = bbox_of(mesh);
    let diag = length(edges::sub(bbox.max, bbox.min));
    let weld = (diag * 1e-6).max(1e-6);
    let eps = weld.max(1e-9);

    let to2d = |p: Vec3| -> [f64; 2] {
        let d = edges::sub(p, frame.origin);
        [edges::dot(d, frame.x_axis), edges::dot(d, frame.y_axis())]
    };

    let mut segments: Vec<Segment> = Vec::new();
    let mut coplanar_tris = 0usize;
    for tri in tris.chunks_exact(3) {
        let p = [
            positions[tri[0] as usize],
            positions[tri[1] as usize],
            positions[tri[2] as usize],
        ];
        let d = p.map(|q| edges::dot(edges::sub(q, frame.origin), frame.normal));
        if d.iter().all(|v| v.abs() <= eps) {
            coplanar_tris += 1;
            continue;
        }
        if d.iter().all(|v| *v > eps) || d.iter().all(|v| *v < -eps) {
            continue;
        }

        let mut points: Vec<Vec3> = Vec::with_capacity(3);
        for k in 0..3 {
            let j = (k + 1) % 3;
            if d[k].abs() <= eps {
                push_unique3(&mut points, p[k], eps);
            }
            if (d[k] > eps && d[j] < -eps) || (d[k] < -eps && d[j] > eps) {
                let t = d[k] / (d[k] - d[j]);
                let q = [
                    p[k][0] + t * (p[j][0] - p[k][0]),
                    p[k][1] + t * (p[j][1] - p[k][1]),
                    p[k][2] + t * (p[j][2] - p[k][2]),
                ];
                push_unique3(&mut points, q, eps);
            }
        }
        if points.len() != 2 {
            continue;
        }
        if length(edges::sub(points[0], points[1])) <= eps {
            continue;
        }
        // Orient the segment so the cross-section is counter-clockwise about
        // the plane normal: d ∝ n × m, where n is the triangle's outward
        // normal. This is what makes holes (inward-wound surfaces) subtract.
        let normal = edges::cross(edges::sub(p[1], p[0]), edges::sub(p[2], p[0]));
        let guide = edges::cross(normal, frame.normal);
        let mut wa = points[0];
        let mut wb = points[1];
        if length(guide) > 1e-12 && edges::dot(edges::sub(wb, wa), guide) < 0.0 {
            std::mem::swap(&mut wa, &mut wb);
        }
        segments.push(Segment {
            a: to2d(wa),
            b: to2d(wb),
            wa,
            wb,
        });
    }

    // A plane through an existing mesh edge produces the same cut from both
    // adjacent triangles. If both traversals agree, the edge is on the
    // cross-section boundary and counts once. If they oppose, the plane is
    // tangent to the solid there and the pair cancels.
    let key = |p: [f64; 2]| -> [i64; 2] {
        [(p[0] / weld).round() as i64, (p[1] / weld).round() as i64]
    };
    let mut groups: BTreeMap<([i64; 2], [i64; 2]), Vec<usize>> = BTreeMap::new();
    for (i, seg) in segments.iter().enumerate() {
        let (ka, kb) = (key(seg.a), key(seg.b));
        let pair = if ka < kb { (ka, kb) } else { (kb, ka) };
        groups.entry(pair).or_default().push(i);
    }
    let mut keep = vec![false; segments.len()];
    for ((ka, kb), ids) in &groups {
        if ids.len() == 1 {
            keep[ids[0]] = true;
            continue;
        }
        let mut sign = 0i32;
        let mut agree = true;
        for &i in ids {
            let (sa, sb) = (key(segments[i].a), key(segments[i].b));
            let s = if sa == *ka && sb == *kb { 1 } else { -1 };
            if sign == 0 {
                sign = s;
            } else if s != sign {
                agree = false;
            }
        }
        if agree {
            keep[ids[0]] = true;
        }
    }
    let unique: Vec<&Segment> = segments
        .iter()
        .enumerate()
        .filter_map(|(i, seg)| if keep[i] { Some(seg) } else { None })
        .collect();

    let mut area2 = 0.0;
    let mut perimeter = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for seg in &unique {
        let cross = seg.a[0] * seg.b[1] - seg.b[0] * seg.a[1];
        area2 += cross;
        cx += (seg.a[0] + seg.b[0]) * cross;
        cy += (seg.a[1] + seg.b[1]) * cross;
        perimeter += distance2(seg.a, seg.b);
        for q in [seg.wa, seg.wb] {
            for k in 0..3 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
    }
    let signed_area = area2 * 0.5;
    let area = signed_area.abs();
    let centroid = if area.abs() > 1e-12 {
        let c2 = [cx / (6.0 * signed_area), cy / (6.0 * signed_area)];
        Some([
            frame.origin[0] + frame.x_axis[0] * c2[0] + frame.y_axis()[0] * c2[1],
            frame.origin[1] + frame.x_axis[1] * c2[0] + frame.y_axis()[1] * c2[1],
            frame.origin[2] + frame.x_axis[2] * c2[0] + frame.y_axis()[2] * c2[1],
        ])
    } else {
        None
    };

    Ok(SectionReport {
        origin: frame.origin,
        normal: frame.normal,
        x_axis: frame.x_axis,
        area,
        perimeter,
        loops: count_loops(&unique, weld),
        segments: unique.len(),
        centroid,
        bbox: if unique.is_empty() {
            None
        } else {
            Some(Bbox { min: lo, max: hi })
        },
        coplanar_tris,
    })
}

fn count_loops(segments: &[&Segment], weld: f64) -> usize {
    if segments.is_empty() {
        return 0;
    }
    let key = |p: [f64; 2]| -> [i64; 2] {
        [(p[0] / weld).round() as i64, (p[1] / weld).round() as i64]
    };
    let mut ids: BTreeMap<[i64; 2], usize> = BTreeMap::new();
    let mut parent: Vec<usize> = Vec::new();
    let mut degree: Vec<usize> = Vec::new();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let id = |p: [f64; 2], ids: &mut BTreeMap<[i64; 2], usize>, parent: &mut Vec<usize>, degree: &mut Vec<usize>| {
        let k = key(p);
        if let Some(&i) = ids.get(&k) {
            return i;
        }
        let i = parent.len();
        ids.insert(k, i);
        parent.push(i);
        degree.push(0);
        i
    };
    for seg in segments {
        let a = id(seg.a, &mut ids, &mut parent, &mut degree);
        let b = id(seg.b, &mut ids, &mut parent, &mut degree);
        degree[a] += 1;
        degree[b] += 1;
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            let (lo, hi) = (ra.min(rb), ra.max(rb));
            parent[hi] = lo;
        }
    }
    let mut components: BTreeSet<usize> = BTreeSet::new();
    let mut closed: BTreeSet<usize> = BTreeSet::new();
    for i in 0..parent.len() {
        let root = find(&mut parent, i);
        components.insert(root);
        if degree[i] != 2 {
            closed.insert(root);
        }
    }
    components.len().saturating_sub(closed.len())
}

// --- distance engine ------------------------------------------------------------

#[derive(Clone, Copy)]
struct Tri {
    p: [Vec3; 3],
    lo: Vec3,
    hi: Vec3,
}

impl Tri {
    fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for q in [a, b, c] {
            for k in 0..3 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
        Self { p: [a, b, c], lo, hi }
    }

    fn centroid(&self) -> Vec3 {
        [
            (self.p[0][0] + self.p[1][0] + self.p[2][0]) / 3.0,
            (self.p[0][1] + self.p[1][1] + self.p[2][1]) / 3.0,
            (self.p[0][2] + self.p[1][2] + self.p[2][2]) / 3.0,
        ]
    }
}

struct BvhNode {
    lo: Vec3,
    hi: Vec3,
    left: usize,
    right: usize,
    start: usize,
    end: usize,
}

const NO_CHILD: usize = usize::MAX;

struct Bvh {
    nodes: Vec<BvhNode>,
    order: Vec<usize>,
}

struct Surface {
    tris: Vec<Tri>,
    bvh: Bvh,
}

impl Surface {
    fn new(mesh: &Mesh) -> Result<Self> {
        let mut tris = Vec::with_capacity(mesh.indices.len() / 3);
        for tri in mesh.indices.chunks_exact(3) {
            if let (Some(a), Some(b), Some(c)) = (
                mesh.positions.get(tri[0] as usize),
                mesh.positions.get(tri[1] as usize),
                mesh.positions.get(tri[2] as usize),
            ) {
                let t = Tri::new(
                    [a[0] as f64, a[1] as f64, a[2] as f64],
                    [b[0] as f64, b[1] as f64, b[2] as f64],
                    [c[0] as f64, c[1] as f64, c[2] as f64],
                );
                if length(edges::cross(edges::sub(t.p[1], t.p[0]), edges::sub(t.p[2], t.p[0])))
                    > 1e-14
                {
                    tris.push(t);
                }
            }
        }
        if tris.is_empty() {
            return Err(Error::Invalid(
                "a measurement target has no non-degenerate triangles (hint: repair the mesh \
                 or measure a valid solid)"
                    .into(),
            ));
        }
        let bvh = Bvh::build(&tris);
        Ok(Self { tris, bvh })
    }

    /// `(closest point, distance)` from `p` to this surface.
    fn closest_to_point(&self, p: Vec3) -> Option<(Vec3, f64)> {
        let mut best = PointBest { d2: f64::INFINITY, point: [0.0; 3] };
        self.bvh.point_query(0, &self.tris, p, &mut best);
        if best.d2.is_finite() {
            Some((best.point, best.d2.sqrt()))
        } else {
            None
        }
    }

    /// `(distance, point on self, point on other)`.
    fn closest_to(&self, other: &Surface) -> Option<(f64, Vec3, Vec3)> {
        let mut best = PairBest {
            d2: f64::INFINITY,
            a: [0.0; 3],
            b: [0.0; 3],
        };
        self.bvh.pair_query(
            0,
            &self.tris,
            0,
            &other.bvh,
            &other.tris,
            &mut best,
        );
        if best.d2.is_finite() {
            Some((best.d2.sqrt(), best.a, best.b))
        } else {
            None
        }
    }
}

impl Bvh {
    fn build(tris: &[Tri]) -> Self {
        let mut order: Vec<usize> = (0..tris.len()).collect();
        let mut bvh = Bvh { nodes: Vec::new(), order: Vec::new() };
        bvh.build_node(&mut order, tris);
        bvh
    }

    fn build_node(&mut self, ids: &mut [usize], tris: &[Tri]) -> usize {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for &i in ids.iter() {
            for k in 0..3 {
                lo[k] = lo[k].min(tris[i].lo[k]);
                hi[k] = hi[k].max(tris[i].hi[k]);
            }
        }
        let index = self.nodes.len();
        self.nodes.push(BvhNode {
            lo,
            hi,
            left: NO_CHILD,
            right: NO_CHILD,
            start: 0,
            end: 0,
        });
        if ids.len() <= LEAF_TRIS {
            let start = self.order.len();
            self.order.extend_from_slice(ids);
            let end = self.order.len();
            self.nodes[index].start = start;
            self.nodes[index].end = end;
            return index;
        }

        let ext = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] {
            0
        } else if ext[1] >= ext[2] {
            1
        } else {
            2
        };
        ids.sort_by(|&a, &b| {
            tris[a].centroid()[axis]
                .total_cmp(&tris[b].centroid()[axis])
                .then_with(|| a.cmp(&b))
        });
        let mid = ids.len() / 2;
        let (left, right) = ids.split_at_mut(mid);
        let l = self.build_node(left, tris);
        let r = self.build_node(right, tris);
        self.nodes[index].left = l;
        self.nodes[index].right = r;
        index
    }

    fn point_query(&self, node: usize, tris: &[Tri], p: Vec3, best: &mut PointBest) {
        let n = &self.nodes[node];
        if point_aabb_dist2(p, n.lo, n.hi) >= best.d2 {
            return;
        }
        if n.left == NO_CHILD {
            for &i in &self.order[n.start..n.end] {
                let (q, distance) = closest_on_triangle(p, &tris[i]);
                let d2 = distance * distance;
                if d2 < best.d2 {
                    best.d2 = d2;
                    best.point = q;
                }
            }
            return;
        }
        let dl = point_aabb_dist2(p, self.nodes[n.left].lo, self.nodes[n.left].hi);
        let dr = point_aabb_dist2(p, self.nodes[n.right].lo, self.nodes[n.right].hi);
        if dl <= dr {
            self.point_query(n.left, tris, p, best);
            self.point_query(n.right, tris, p, best);
        } else {
            self.point_query(n.right, tris, p, best);
            self.point_query(n.left, tris, p, best);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn pair_query(
        &self,
        na: usize,
        tris_a: &[Tri],
        nb: usize,
        other: &Bvh,
        tris_b: &[Tri],
        best: &mut PairBest,
    ) {
        let a = &self.nodes[na];
        let b = &other.nodes[nb];
        if aabb_dist2(a.lo, a.hi, b.lo, b.hi) >= best.d2 {
            return;
        }
        let a_leaf = a.left == NO_CHILD;
        let b_leaf = b.left == NO_CHILD;
        if a_leaf && b_leaf {
            for &i in &self.order[a.start..a.end] {
                for &j in &other.order[b.start..b.end] {
                    if let Some((pa, pb, d2)) = triangle_distance(&tris_a[i], &tris_b[j]) {
                        if d2 < best.d2 {
                            best.d2 = d2;
                            best.a = pa;
                            best.b = pb;
                        }
                    }
                }
            }
            return;
        }
        if a_leaf {
            let dl = aabb_dist2(a.lo, a.hi, other.nodes[b.left].lo, other.nodes[b.left].hi);
            let dr = aabb_dist2(a.lo, a.hi, other.nodes[b.right].lo, other.nodes[b.right].hi);
            if dl <= dr {
                self.pair_query(na, tris_a, b.left, other, tris_b, best);
                self.pair_query(na, tris_a, b.right, other, tris_b, best);
            } else {
                self.pair_query(na, tris_a, b.right, other, tris_b, best);
                self.pair_query(na, tris_a, b.left, other, tris_b, best);
            }
            return;
        }
        if b_leaf {
            let dl = aabb_dist2(self.nodes[a.left].lo, self.nodes[a.left].hi, b.lo, b.hi);
            let dr = aabb_dist2(self.nodes[a.right].lo, self.nodes[a.right].hi, b.lo, b.hi);
            if dl <= dr {
                self.pair_query(a.left, tris_a, nb, other, tris_b, best);
                self.pair_query(a.right, tris_a, nb, other, tris_b, best);
            } else {
                self.pair_query(a.right, tris_a, nb, other, tris_b, best);
                self.pair_query(a.left, tris_a, nb, other, tris_b, best);
            }
            return;
        }
        // Split whichever node is larger, which keeps the recursion balanced
        // when the two trees have different depths.
        let ea = extent(a);
        let eb = extent(b);
        if ea >= eb {
            let dl = aabb_dist2(self.nodes[a.left].lo, self.nodes[a.left].hi, b.lo, b.hi);
            let dr = aabb_dist2(self.nodes[a.right].lo, self.nodes[a.right].hi, b.lo, b.hi);
            if dl <= dr {
                self.pair_query(a.left, tris_a, nb, other, tris_b, best);
                self.pair_query(a.right, tris_a, nb, other, tris_b, best);
            } else {
                self.pair_query(a.right, tris_a, nb, other, tris_b, best);
                self.pair_query(a.left, tris_a, nb, other, tris_b, best);
            }
        } else {
            let dl = aabb_dist2(a.lo, a.hi, other.nodes[b.left].lo, other.nodes[b.left].hi);
            let dr = aabb_dist2(a.lo, a.hi, other.nodes[b.right].lo, other.nodes[b.right].hi);
            if dl <= dr {
                self.pair_query(na, tris_a, b.left, other, tris_b, best);
                self.pair_query(na, tris_a, b.right, other, tris_b, best);
            } else {
                self.pair_query(na, tris_a, b.right, other, tris_b, best);
                self.pair_query(na, tris_a, b.left, other, tris_b, best);
            }
        }
    }
}

struct PointBest {
    d2: f64,
    point: Vec3,
}

struct PairBest {
    d2: f64,
    a: Vec3,
    b: Vec3,
}

fn extent(n: &BvhNode) -> f64 {
    (n.hi[0] - n.lo[0])
        .max(n.hi[1] - n.lo[1])
        .max(n.hi[2] - n.lo[2])
}

fn point_aabb_dist2(p: Vec3, lo: Vec3, hi: Vec3) -> f64 {
    let mut d2 = 0.0;
    for k in 0..3 {
        let d = if p[k] < lo[k] {
            lo[k] - p[k]
        } else if p[k] > hi[k] {
            p[k] - hi[k]
        } else {
            0.0
        };
        d2 += d * d;
    }
    d2
}

fn aabb_dist2(alo: Vec3, ahi: Vec3, blo: Vec3, bhi: Vec3) -> f64 {
    let mut d2 = 0.0;
    for k in 0..3 {
        let d = if ahi[k] < blo[k] {
            blo[k] - ahi[k]
        } else if bhi[k] < alo[k] {
            alo[k] - bhi[k]
        } else {
            0.0
        };
        d2 += d * d;
    }
    d2
}

fn closest_on_triangle(p: Vec3, t: &Tri) -> (Vec3, f64) {
    let [a, b, c] = t.p;
    let ab = edges::sub(b, a);
    let ac = edges::sub(c, a);
    let ap = edges::sub(p, a);
    let d1 = edges::dot(ab, ap);
    let d2 = edges::dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, length(edges::sub(p, a)));
    }

    let bp = edges::sub(p, b);
    let d3 = edges::dot(ab, bp);
    let d4 = edges::dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, length(edges::sub(p, b)));
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let q = add(a, edges::scale(ab, v));
        return (q, length(edges::sub(p, q)));
    }

    let cp = edges::sub(p, c);
    let d5 = edges::dot(ab, cp);
    let d6 = edges::dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, length(edges::sub(p, c)));
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let q = add(a, edges::scale(ac, w));
        return (q, length(edges::sub(p, q)));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let q = add(b, edges::scale(edges::sub(c, b), w));
        return (q, length(edges::sub(p, q)));
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let q = add(a, add(edges::scale(ab, v), edges::scale(ac, w)));
    (q, length(edges::sub(p, q)))
}

fn triangle_distance(a: &Tri, b: &Tri) -> Option<(Vec3, Vec3, f64)> {
    // Crossing triangles are at distance 0.
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        if let Some(q) = segment_triangle(a.p[i], a.p[j], b) {
            return Some((q, q, 0.0));
        }
    }
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        if let Some(q) = segment_triangle(b.p[i], b.p[j], a) {
            return Some((q, q, 0.0));
        }
    }

    let mut best = PairBest {
        d2: f64::INFINITY,
        a: [0.0; 3],
        b: [0.0; 3],
    };
    for q in a.p {
        let (p, d2) = closest_on_triangle(q, b);
        if d2 * d2 < best.d2 {
            best.d2 = d2 * d2;
            best.a = q;
            best.b = p;
        }
    }
    for q in b.p {
        let (p, d2) = closest_on_triangle(q, a);
        if d2 * d2 < best.d2 {
            best.d2 = d2 * d2;
            best.a = p;
            best.b = q;
        }
    }
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        for (k, l) in [(0, 1), (1, 2), (2, 0)] {
            let (pa, pb, d2) = segment_segment(a.p[i], a.p[j], b.p[k], b.p[l]);
            if d2 < best.d2 {
                best.d2 = d2;
                best.a = pa;
                best.b = pb;
            }
        }
    }
    if best.d2.is_finite() {
        Some((best.a, best.b, best.d2))
    } else {
        None
    }
}

fn segment_triangle(p: Vec3, q: Vec3, t: &Tri) -> Option<Vec3> {
    let dir = edges::sub(q, p);
    let [a, b, c] = t.p;
    let e1 = edges::sub(b, a);
    let e2 = edges::sub(c, a);
    let h = edges::cross(dir, e2);
    let det = edges::dot(e1, h);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = edges::sub(p, a);
    let u = inv * edges::dot(s, h);
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let qv = edges::cross(s, e1);
    let v = inv * edges::dot(dir, qv);
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    let time = inv * edges::dot(e2, qv);
    if !(-1e-9..=1.0 + 1e-9).contains(&time) {
        return None;
    }
    Some(add(p, edges::scale(dir, time)))
}

fn segment_segment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3, f64) {
    let d1 = edges::sub(q1, p1);
    let d2 = edges::sub(q2, p2);
    let r = edges::sub(p1, p2);
    let a = edges::dot(d1, d1);
    let e = edges::dot(d2, d2);
    let f = edges::dot(d2, r);
    let eps = 1e-14;
    let (mut s, mut t);
    if a <= eps && e <= eps {
        s = 0.0;
        t = 0.0;
    } else if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = edges::dot(d1, r);
        if e <= eps {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = edges::dot(d1, d2);
            let denom = a * e - b * b;
            s = if denom > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    let pa = add(p1, edges::scale(d1, s));
    let pb = add(p2, edges::scale(d2, t));
    (pa, pb, length2(edges::sub(pa, pb)))
}

// --- small helpers --------------------------------------------------------------

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn length(v: Vec3) -> f64 {
    edges::dot(v, v).sqrt()
}

fn length2(v: Vec3) -> f64 {
    edges::dot(v, v)
}

fn unit(v: Vec3) -> Vec3 {
    let l = length(v);
    if l < 1e-12 {
        [f64::NAN; 3]
    } else {
        [v[0] / l, v[1] / l, v[2] / l]
    }
}

fn angle_deg(a: Vec3, b: Vec3) -> f64 {
    let denom = length(a) * length(b);
    if denom < 1e-12 {
        return 0.0;
    }
    (edges::dot(a, b) / denom).clamp(-1.0, 1.0).acos().to_degrees()
}

fn distance2(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

fn push_unique3(points: &mut Vec<Vec3>, p: Vec3, tol: f64) {
    if points.iter().all(|q| length(edges::sub(*q, p)) > tol) {
        points.push(p);
    }
}

fn bbox_of(mesh: &Mesh) -> Bbox {
    if mesh.positions.is_empty() {
        return Bbox { min: [0.0; 3], max: [0.0; 3] };
    }
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k] as f64);
            hi[k] = hi[k].max(p[k] as f64);
        }
    }
    Bbox { min: lo, max: hi }
}

fn bbox_of_indices(mesh: &Mesh) -> Bbox {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for &i in &mesh.indices {
        if let Some(p) = mesh.positions.get(i as usize) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k] as f64);
                hi[k] = hi[k].max(p[k] as f64);
            }
        }
    }
    if lo[0] > hi[0] {
        Bbox { min: [0.0; 3], max: [0.0; 3] }
    } else {
        Bbox { min: lo, max: hi }
    }
}

fn aabb_overlap(a: &Bbox, b: &Bbox) -> bool {
    (0..3).all(|i| a.min[i] <= b.max[i] && b.min[i] <= a.max[i])
}

fn class_rank(class: FitClass) -> usize {
    FitClass::all().iter().position(|c| *c == class).unwrap_or(4)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_mesh(size: [f64; 3], center: [f64; 3]) -> Mesh {
        let [sx, sy, sz] = size;
        let [cx, cy, cz] = center;
        let hx = sx / 2.0;
        let hy = sy / 2.0;
        let hz = sz / 2.0;
        let p = |x: f64, y: f64, z: f64| [cx + x, cy + y, cz + z];
        let v = vec![
            p(-hx, -hy, -hz),
            p(hx, -hy, -hz),
            p(hx, hy, -hz),
            p(-hx, hy, -hz),
            p(-hx, -hy, hz),
            p(hx, -hy, hz),
            p(hx, hy, hz),
            p(-hx, hy, hz),
        ];
        let idx = vec![
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3,
            7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
        ];
        Mesh { positions: v.into_iter().map(|q| [q[0] as f32, q[1] as f32, q[2] as f32]).collect(), normals: Vec::new(), indices: idx }
    }

    fn frame_xy() -> Frame {
        Frame { origin: [0.0; 3], normal: [0.0, 0.0, 1.0], x_axis: [1.0, 0.0, 0.0] }
    }

    #[test]
    fn closest_point_on_triangle_handles_vertices_edges_and_face() {
        let t = Tri::new([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]);
        let (q, d) = closest_on_triangle([-1.0, -1.0, 3.0], &t);
        assert_eq!(q, [0.0, 0.0, 0.0]);
        assert!((d - 11f64.sqrt()).abs() < 1e-9, "{d}");
        let (q, d) = closest_on_triangle([2.0, 2.0, 5.0], &t);
        assert_eq!(q, [2.0, 2.0, 0.0]);
        assert!((d - 5.0).abs() < 1e-9, "{d}");
    }

    #[test]
    fn box_mass_properties_match_the_analytic_values() {
        let mesh = box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0]);
        let p = mesh_mass_properties(&mesh);
        assert!((p.volume - 1000.0).abs() < 1e-9, "{}", p.volume);
        assert!((p.surface_area - 600.0).abs() < 1e-9, "{}", p.surface_area);
        assert_eq!(p.centroid, [0.0, 0.0, 0.0]);
        // I = m a² / 6 for a cube about its centroid, density 1 g/mm³.
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1000.0 * 100.0 / 6.0 } else { 0.0 };
                assert!(
                    (p.inertia_unit[i][j] - want).abs() < 1e-6,
                    "I[{i}][{j}] = {} want {want}",
                    p.inertia_unit[i][j]
                );
            }
        }
    }

    #[test]
    fn section_area_of_a_box_is_its_cross_section() {
        let mesh = box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0]);
        let report = section_report(&mesh, &frame_xy()).unwrap();
        assert!((report.area - 100.0).abs() < 1e-9, "{}", report.area);
        assert!((report.perimeter - 40.0).abs() < 1e-9, "{}", report.perimeter);
        assert_eq!(report.loops, 1);
        assert_eq!(report.centroid, Some([0.0, 0.0, 0.0]));
        assert!(report.coplanar_tris == 0);
    }

    #[test]
    fn section_area_works_on_non_xy_planes() {
        let mesh = box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0]);
        let yz = Frame { origin: [0.0; 3], normal: [1.0, 0.0, 0.0], x_axis: [0.0, 1.0, 0.0] };
        let report = section_report(&mesh, &yz).unwrap();
        assert!((report.area - 100.0).abs() < 1e-9, "{}", report.area);

        let inv = 1.0 / 2f64.sqrt();
        let diagonal = Frame {
            origin: [0.0; 3],
            normal: [inv, inv, 0.0],
            x_axis: [0.0, 0.0, 1.0],
        };
        let report = section_report(&mesh, &diagonal).unwrap();
        assert!(
            (report.area - 100.0 * 2f64.sqrt()).abs() < 1e-6,
            "{}",
            report.area
        );
    }

    #[test]
    fn section_holes_subtract_from_the_area() {
        let outer = box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0]);
        let inner = box_mesh([4.0, 4.0, 10.0], [0.0, 0.0, 0.0]);
        let cut = crate::boolean::boolean(
            wright_protocol::BooleanOp::Difference,
            &[outer, inner],
        )
        .unwrap();
        let report = section_report(&cut, &frame_xy()).unwrap();
        assert!((report.area - 84.0).abs() < 1e-6, "{}", report.area);
    }

    #[test]
    fn two_boxes_report_the_gap_and_closest_points() {
        let a = Surface::new(&box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0])).unwrap();
        let b = Surface::new(&box_mesh([10.0, 10.0, 10.0], [13.0, 0.0, 0.0])).unwrap();
        let (d, pa, pb) = a.closest_to(&b).unwrap();
        assert!((d - 3.0).abs() < 1e-9, "{d}");
        assert!((pa[0] - 5.0).abs() < 1e-9, "{pa:?}");
        assert!((pb[0] - 8.0).abs() < 1e-9, "{pb:?}");
    }

    #[test]
    fn point_to_surface_distance_is_exact() {
        let a = Surface::new(&box_mesh([10.0, 10.0, 10.0], [0.0, 0.0, 0.0])).unwrap();
        let (q, d) = a.closest_to_point([0.0, 0.0, 20.0]).unwrap();
        assert!((d - 15.0).abs() < 1e-9, "{d}");
        assert_eq!(q, [0.0, 0.0, 5.0]);
    }
}
