//! STEP export: an AP214 (`AUTOMOTIVE_DESIGN`) **faceted B-rep**.
//!
//! wright's kernel is tessellated. It does not remember that a cylinder was
//! a cylinder once the mesh is built, so a STEP writer cannot honestly emit
//! `CYLINDRICAL_SURFACE`s — it would have to invent them from triangles. This
//! module makes that decision explicitly instead of faking analytic geometry:
//!
//! * every triangle becomes a planar `FACE_SURFACE` bounded by a three-point
//!   `POLY_LOOP`, with a real `PLANE` and `AXIS2_PLACEMENT_3D`;
//! * coincident vertices are welded by exact `f32` position bits, so adjacent
//!   faces reference the same `CARTESIAN_POINT`s (the mesh carries duplicate
//!   corners for crease normals — those are not STEP topology);
//! * each connected, edge-manifold, consistently wound piece becomes one
//!   `FACETED_BREP` with a `CLOSED_SHELL`. The result is a real solid B-rep a
//!   CAD kernel can measure and boolean, just faceted;
//! * a piece that is open, non-manifold or inconsistently wound becomes an
//!   `OPEN_SHELL` in a `SHELL_BASED_SURFACE_MODEL` instead. Writing it as a
//!   `FACETED_BREP` would be a lie, and silently dropping it would lose data.
//!
//! The import direction is deliberately not implemented. Real STEP files are
//! mostly analytic B-reps with trimmed surfaces; turning those into a mesh is
//! a different kernel problem (surface intersection, trimming, tolerance
//! healing). Incoming geometry comes through STL/OBJ/3MF, which are already
//! tessellated. See the README for the full trade-off.
//!
//! Everything is deterministic: welded-vertex ids come from first use, a
//! `BTreeMap` drives every lookup, and the header timestamp is fixed (a real
//! clock would make identical parts export different bytes).

use std::collections::BTreeMap;
use std::fmt::Write as _;

use wright_protocol::{Mat4, Mesh};

use crate::mat4;
use crate::Error;

/// Fixed ISO-8601 timestamp for `FILE_NAME`. STEP requires a timestamp, but a
/// real one carries no model information and would break byte-stable exports.
const FIXED_TIMESTAMP: &str = "1970-01-01T00:00:00";

/// One part to write: an entity-local mesh, its world placement and its name.
pub struct Part<'a> {
    pub name: &'a str,
    pub mesh: &'a Mesh,
    pub transform: Mat4,
}

/// Counts and byte size of a written STEP file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Export {
    /// Closed pieces written as `FACETED_BREP` solids.
    pub solids: usize,
    /// Open / non-manifold pieces written as `SHELL_BASED_SURFACE_MODEL`s.
    pub surfaces: usize,
    /// Triangles written as faces (both solids and surfaces).
    pub faces: usize,
    pub bytes: usize,
}

/// Serialize the parts into an AP214 STEP file, returning `(bytes, export)`.
pub fn step_bytes(parts: &[Part<'_>], title: &str) -> (Vec<u8>, Export) {
    let mut w = Writer::new();
    w.header(title);

    let mut solids = Vec::new();
    let mut surfaces = Vec::new();
    let mut faces = 0usize;

    for part in parts {
        let prepared = Prepared::new(part);
        if prepared.triangles.is_empty() {
            continue;
        }
        let groups = connected_components(&prepared.triangles);
        let point_ids = prepared
            .points
            .iter()
            .map(|&p| w.point(p))
            .collect::<Vec<_>>();

        let has_multiple = groups.len() > 1;
        for (index, group) in groups.iter().enumerate() {
            let name = if has_multiple {
                format!("{} {}", part.name, index + 1)
            } else {
                part.name.to_string()
            };
            let closed = is_closed_manifold(&prepared.triangles, group);
            let mut shell_faces = Vec::with_capacity(group.len());
            for &tri_index in group {
                let tri = prepared.triangles[tri_index];
                shell_faces.push(w.face(&prepared.points, &point_ids, tri));
                faces += 1;
            }
            let shell = w.add(if closed {
                format!("CLOSED_SHELL('',({}))", ids(&shell_faces))
            } else {
                format!("OPEN_SHELL('',({}))", ids(&shell_faces))
            });
            if closed {
                solids.push(w.add(format!("FACETED_BREP({},#{shell})", quote(&name))));
            } else {
                surfaces.push(w.add(format!(
                    "SHELL_BASED_SURFACE_MODEL({},(#{shell}))",
                    quote(&name)
                )));
            }
        }
    }

    if !solids.is_empty() {
        let rep = w.add(format!(
            "FACETED_BREP_SHAPE_REPRESENTATION({},({}),#{})",
            quote(title),
            ids(&solids),
            w.context
        ));
        w.add(format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{},#{rep})",
            w.product_shape
        ));
    }
    if !surfaces.is_empty() {
        let rep = w.add(format!(
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION({},({}),#{})",
            quote(&format!("{title} surfaces")),
            ids(&surfaces),
            w.context
        ));
        w.add(format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{},#{rep})",
            w.product_shape
        ));
    }

    w.footer();
    let bytes = w.out.into_bytes();
    let export = Export {
        solids: solids.len(),
        surfaces: surfaces.len(),
        faces,
        bytes: bytes.len(),
    };
    (bytes, export)
}

/// Write a STEP file, returning `(resolved path, export)`.
pub fn write_step(path: &str, parts: &[Part<'_>], title: &str) -> Result<(String, Export), Error> {
    let (bytes, export) = step_bytes(parts, title);
    std::fs::write(path, &bytes).map_err(|e| {
        Error::Invalid(format!(
            "cannot write STEP to '{path}': {e} (hint: use an absolute path the server can write)"
        ))
    })?;
    let resolved = std::fs::canonicalize(path)
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    Ok((resolved, export))
}

// --- topology ----------------------------------------------------------------

/// A part after world-space transform and exact-position welding.
struct Prepared {
    points: Vec<[f32; 3]>,
    triangles: Vec<[u32; 3]>,
}

impl Prepared {
    fn new(part: &Part<'_>) -> Self {
        let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
        let mut points = Vec::new();
        let mut remap = Vec::with_capacity(part.mesh.positions.len());
        for &p in &part.mesh.positions {
            let world = mat4::transform_point(&part.transform, p);
            let key = [world[0].to_bits(), world[1].to_bits(), world[2].to_bits()];
            let id = match ids.get(&key) {
                Some(&id) => id,
                None => {
                    let id = points.len() as u32;
                    ids.insert(key, id);
                    points.push(world);
                    id
                }
            };
            remap.push(id);
        }
        let mut triangles = Vec::with_capacity(part.mesh.indices.len() / 3);
        for tri in part.mesh.indices.chunks_exact(3) {
            let t = [
                remap[tri[0] as usize],
                remap[tri[1] as usize],
                remap[tri[2] as usize],
            ];
            // A face needs three distinct points. The manifold kernel may keep
            // zero-area triangles with distinct corners (tangent seams); those
            // stay, because dropping them would open the shell.
            if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
                continue;
            }
            triangles.push(t);
        }
        Self { points, triangles }
    }
}

/// Split triangles into edge-connected components, ordered by their first
/// triangle. A boolean can leave disjoint pieces in one entity; each piece is
/// its own shell (a single `CLOSED_SHELL` may not be spatially disjoint).
fn connected_components(triangles: &[[u32; 3]]) -> Vec<Vec<usize>> {
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..triangles.len()).collect();
    let mut first_edge: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for (index, tri) in triangles.iter().enumerate() {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            match first_edge.get(&key) {
                Some(&other) => {
                    let ra = find(&mut parent, index);
                    let rb = find(&mut parent, other);
                    if ra != rb {
                        // Keep the earlier triangle as the root, so groups and
                        // therefore entity ids are deterministic.
                        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                        parent[hi] = lo;
                    }
                }
                None => {
                    first_edge.insert(key, index);
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for index in 0..triangles.len() {
        let root = find(&mut parent, index);
        groups.entry(root).or_default().push(index);
    }
    groups.into_values().collect()
}

/// True when every directed edge appears exactly once and its reverse appears
/// exactly once: a closed, edge-manifold, consistently wound piece.
fn is_closed_manifold(triangles: &[[u32; 3]], group: &[usize]) -> bool {
    let mut uses: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for &index in group {
        let tri = triangles[index];
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *uses.entry((a, b)).or_default() += 1;
        }
    }
    uses.iter()
        .all(|(&(a, b), &count)| count == 1 && uses.get(&(b, a)) == Some(&1))
}

// --- writing -----------------------------------------------------------------

struct Writer {
    out: String,
    next: usize,
    context: usize,
    product_shape: usize,
    points: BTreeMap<[u32; 3], usize>,
    directions: BTreeMap<[u32; 3], usize>,
}

impl Writer {
    fn new() -> Self {
        Self {
            out: String::new(),
            next: 1,
            context: 0,
            product_shape: 0,
            points: BTreeMap::new(),
            directions: BTreeMap::new(),
        }
    }

    fn add(&mut self, body: String) -> usize {
        let id = self.next;
        self.next += 1;
        writeln!(self.out, "#{id}={body};").unwrap();
        id
    }

    fn header(&mut self, title: &str) {
        let name = if title.trim().is_empty() {
            "wright part"
        } else {
            title
        };
        self.out.push_str("ISO-10303-21;\nHEADER;\n");
        self.out
            .push_str("FILE_DESCRIPTION(('wright faceted B-rep; units millimeters'),'2;1');\n");
        writeln!(
            self.out,
            "FILE_NAME({},'{}',('wright'),('wright'),'wright','wright','');",
            quote(&format!("{name}.step")),
            FIXED_TIMESTAMP
        )
        .unwrap();
        self.out
            .push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
        self.out.push_str("ENDSEC;\nDATA;\n");

        let app = self.add("APPLICATION_CONTEXT('automotive design')".into());
        self.add(format!(
            "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})"
        ));
        let product_context = self.add(format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
        let definition_context = self.add(format!(
            "PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"
        ));
        let product = self.add(format!(
            "PRODUCT({name},{name},'',(#{product_context}))",
            name = quote(&name.to_string())
        ));
        let formation = self.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
        let definition = self.add(format!(
            "PRODUCT_DEFINITION('design','',#{formation},#{definition_context})"
        ));
        self.product_shape = self.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));

        let length = self.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))".into());
        let angle = self.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))".into());
        let solid_angle =
            self.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())".into());
        let uncertainty = self.add(format!(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-7),#{length},'distance_accuracy_value','confusion accuracy')"
        ));
        self.context = self.add(format!(
            "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solid_angle})) \
             REPRESENTATION_CONTEXT('','3D'))"
        ));
    }

    fn footer(&mut self) {
        self.out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    }

    fn point(&mut self, p: [f32; 3]) -> usize {
        let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
        if let Some(&id) = self.points.get(&key) {
            return id;
        }
        let id = self.add(format!("CARTESIAN_POINT('',{})", vec3(p)));
        self.points.insert(key, id);
        id
    }

    fn direction(&mut self, d: [f32; 3]) -> usize {
        let key = [d[0].to_bits(), d[1].to_bits(), d[2].to_bits()];
        if let Some(&id) = self.directions.get(&key) {
            return id;
        }
        let id = self.add(format!("DIRECTION('',{})", vec3(d)));
        self.directions.insert(key, id);
        id
    }

    /// One planar face for a triangle: `POLY_LOOP` over the already-written
    /// points, a `FACE_OUTER_BOUND`, a plane with the face normal, and the
    /// `FACE_SURFACE` that `FACETED_BREP` expects.
    fn face(&mut self, points: &[[f32; 3]], point_ids: &[usize], tri: [u32; 3]) -> usize {
        let p = [
            points[tri[0] as usize],
            points[tri[1] as usize],
            points[tri[2] as usize],
        ];
        let (normal, reference) = plane_axes(p);
        let n = self.direction(normal);
        let r = self.direction(reference);
        let origin = point_ids[tri[0] as usize];
        let axis = self.add(format!("AXIS2_PLACEMENT_3D('',#{origin},#{n},#{r})"));
        let plane = self.add(format!("PLANE('',#{axis})"));
        let loop_id = self.add(format!(
            "POLY_LOOP('',(#{},#{},#{}))",
            point_ids[tri[0] as usize], point_ids[tri[1] as usize], point_ids[tri[2] as usize]
        ));
        let bound = self.add(format!("FACE_OUTER_BOUND('',#{loop_id},.T.)"));
        self.add(format!("FACE_SURFACE('',(#{bound}),#{plane},.T.)"))
    }
}

fn ids(values: &[usize]) -> String {
    values
        .iter()
        .map(|id| format!("#{id}"))
        .collect::<Vec<_>>()
        .join(",")
}

/// Unit normal and in-plane reference direction for a triangle. The normal is
/// the cross product; a zero-area triangle (tangent seam) falls back to a
/// deterministic plane containing its line, so topology stays closed. The
/// reference is the first edge made perpendicular to the normal.
fn plane_axes(p: [[f32; 3]; 3]) -> ([f32; 3], [f32; 3]) {
    fn sub(a: [f32; 3], b: [f32; 3]) -> [f64; 3] {
        [
            a[0] as f64 - b[0] as f64,
            a[1] as f64 - b[1] as f64,
            a[2] as f64 - b[2] as f64,
        ]
    }
    fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }
    fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
    fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
        let len = dot(v, v).sqrt();
        (len > 1e-20).then(|| [v[0] / len, v[1] / len, v[2] / len])
    }

    let e1 = sub(p[1], p[0]);
    let e2 = sub(p[2], p[0]);
    let mut normal = unit(cross(e1, e2));
    if normal.is_none() {
        // Collinear or coincident corners: any plane through the line works.
        // Pick the coordinate axis least parallel to the longest edge.
        let edge = unit(e1).or_else(|| unit(e2)).unwrap_or([1.0, 0.0, 0.0]);
        let pick = if edge[0].abs() <= edge[1].abs() && edge[0].abs() <= edge[2].abs() {
            [1.0, 0.0, 0.0]
        } else if edge[1].abs() <= edge[2].abs() {
            [0.0, 1.0, 0.0]
        } else {
            [0.0, 0.0, 1.0]
        };
        normal = unit(cross(edge, pick));
    }
    let normal = normal.unwrap_or([0.0, 0.0, 1.0]);

    let mut reference = e1;
    let along = dot(reference, normal);
    reference = [
        reference[0] - along * normal[0],
        reference[1] - along * normal[1],
        reference[2] - along * normal[2],
    ];
    let reference = unit(reference)
        .or_else(|| {
            // e1 was parallel to the normal (or zero); use e2, then any axis.
            let along = dot(e2, normal);
            unit([
                e2[0] - along * normal[0],
                e2[1] - along * normal[1],
                e2[2] - along * normal[2],
            ])
        })
        .unwrap_or_else(|| {
            let pick = if normal[0].abs() <= normal[1].abs() && normal[0].abs() <= normal[2].abs() {
                [1.0, 0.0, 0.0]
            } else if normal[1].abs() <= normal[2].abs() {
                [0.0, 1.0, 0.0]
            } else {
                [0.0, 0.0, 1.0]
            };
            unit(cross(normal, pick)).unwrap_or([1.0, 0.0, 0.0])
        });

    (
        [normal[0] as f32, normal[1] as f32, normal[2] as f32],
        [
            reference[0] as f32,
            reference[1] as f32,
            reference[2] as f32,
        ],
    )
}

// --- text encoding -----------------------------------------------------------

/// STEP Part 21 string literal: single quotes are doubled, a backslash is
/// escaped, and non-ASCII/control characters are replaced (names are labels,
/// not payloads). The result includes the surrounding quotes.
fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('\'');
    for ch in text.chars() {
        match ch {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            c if c == ' ' || c.is_ascii_graphic() => out.push(c),
            _ => out.push('_'),
        }
    }
    out.push('\'');
    out
}

/// STEP real literal for an `f32`. `Display` never uses an exponent, which
/// keeps ordinary millimeter values short; very small/large magnitudes switch
/// to the `1.E-7` spelling STEP parsers expect.
fn real(v: f32) -> String {
    if !v.is_finite() || v == 0.0 {
        return "0.".into();
    }
    let abs = v.abs();
    if (1e-6..1e9).contains(&abs) {
        let mut s = format!("{v}");
        if !s.contains('.') {
            s.push('.');
        }
        s
    } else {
        let s = format!("{v:e}");
        match s.split_once('e') {
            Some((mantissa, exponent)) => {
                let mantissa = if mantissa.contains('.') {
                    mantissa.to_string()
                } else {
                    format!("{mantissa}.")
                };
                format!("{mantissa}E{exponent}")
            }
            None => s,
        }
    }
}

fn vec3(p: [f32; 3]) -> String {
    format!("({},{},{})", real(p[0]), real(p[1]), real(p[2]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn box_part(name: &str, center: [f64; 3]) -> (Mesh, Mat4, String) {
        let mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        let transform = mat4::translation(center);
        (mesh, transform, name.to_string())
    }

    fn count(haystack: &str, needle: &str) -> usize {
        haystack.matches(needle).count()
    }

    #[test]
    fn a_box_is_one_closed_faceted_brep() {
        let (mesh, transform, name) = box_part("cube", [0.0; 3]);
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let (bytes, export) = step_bytes(&parts, "test");
        assert_eq!(export.solids, 1);
        assert_eq!(export.surfaces, 0);
        assert_eq!(export.faces, 12);
        assert_eq!(export.bytes, bytes.len());

        let text = String::from_utf8(bytes).unwrap();
        assert!(text.starts_with("ISO-10303-21;\nHEADER;\n"));
        assert!(text.ends_with("ENDSEC;\nEND-ISO-10303-21;\n"));
        assert!(text.contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN"));
        assert!(text.contains("FACETED_BREP('cube',#"));
        assert!(text.contains("FACETED_BREP_SHAPE_REPRESENTATION('test'"));
        assert!(!text.contains("SHELL_BASED_SURFACE_MODEL"));
        assert_eq!(count(&text, "CLOSED_SHELL("), 1);
        assert_eq!(count(&text, "FACE_SURFACE("), 12);
        assert_eq!(count(&text, "POLY_LOOP("), 12);
        assert_eq!(
            count(&text, "CARTESIAN_POINT("),
            8,
            "box corners are welded"
        );
        assert_eq!(count(&text, "VERTEX_POINT("), 0, "poly loops use points");
        assert!(text.contains("'1970-01-01T00:00:00'"));
    }

    #[test]
    fn world_space_and_plane_directions_are_written() {
        let (mesh, transform, name) = box_part("moved", [100.0, 0.0, 0.0]);
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let text = String::from_utf8(step_bytes(&parts, "moved").0).unwrap();
        // A 10 mm box centred at x=100 spans 95..105 after the transform.
        assert!(text.contains("(95.,-5.,-5.)"));
        assert!(text.contains("(105.,5.,5.)"));
        assert!(text.contains("DIRECTION('',(0.,0.,1.))"));
        assert!(text.contains("DIRECTION('',(0.,0.,-1.))"));
    }

    #[test]
    fn an_open_mesh_becomes_a_surface_model() {
        let mut mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        mesh.indices.truncate(33); // drop the last (fourth) triangle
        let transform = mat4::identity();
        let name = "open".to_string();
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let (bytes, export) = step_bytes(&parts, "open");
        assert_eq!(export.solids, 0);
        assert_eq!(export.surfaces, 1);
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.contains("SHELL_BASED_SURFACE_MODEL('open'"));
        assert!(text.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('open surfaces'"));
        assert!(text.contains("OPEN_SHELL("));
        assert!(!text.contains("FACETED_BREP("));
    }

    #[test]
    fn disconnected_pieces_become_separate_breps() {
        let (a, _, _) = box_part("a", [0.0; 3]);
        let (b, b_transform, _) = box_part("b", [50.0, 0.0, 0.0]);
        let mut mesh = Mesh {
            positions: a.positions.clone(),
            normals: vec![],
            indices: a.indices.clone(),
        };
        let offset = mesh.positions.len() as u32;
        mesh.positions.extend(
            b.positions
                .iter()
                .map(|&p| mat4::transform_point(&b_transform, p)),
        );
        mesh.indices.extend(b.indices.iter().map(|i| i + offset));
        let transform = mat4::identity();
        let name = "pair".to_string();
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let (bytes, export) = step_bytes(&parts, "pair");
        assert_eq!(export.solids, 2);
        assert_eq!(export.faces, 24);
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.contains("FACETED_BREP('pair 1'"));
        assert!(text.contains("FACETED_BREP('pair 2'"));
        assert_eq!(count(&text, "CLOSED_SHELL("), 2);
    }

    #[test]
    fn inconsistent_winding_is_one_surface_piece() {
        // Flip one triangle in a closed box: every edge is still used twice,
        // but one directed edge is duplicated, so it is not a solid shell.
        let mut mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        mesh.indices.swap(0, 1);
        let transform = mat4::identity();
        let name = "flipped".to_string();
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let (_, export) = step_bytes(&parts, "flipped");
        assert_eq!(export.solids, 0);
        assert_eq!(export.surfaces, 1);
    }

    #[test]
    fn identical_parts_export_identical_bytes() {
        let (mesh, transform, name) = box_part("same", [1.0, 2.0, 3.0]);
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        assert_eq!(step_bytes(&parts, "same").0, step_bytes(&parts, "same").0);
    }

    #[test]
    fn every_reference_resolves_to_one_definition() {
        let (mesh, transform, name) = box_part("cube", [3.0, -2.0, 1.0]);
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let text = String::from_utf8(step_bytes(&parts, "refs").0).unwrap();
        let data = text.split("DATA;\n").nth(1).unwrap();
        let data = data.split("ENDSEC;").next().unwrap();

        let mut defined = std::collections::BTreeSet::new();
        for line in data.lines() {
            let line = line.trim();
            if let Some(rest) = line.strip_prefix('#') {
                let id: usize = rest
                    .split('=')
                    .next()
                    .unwrap()
                    .parse()
                    .expect("entity id is a number");
                assert!(defined.insert(id), "duplicate entity #{id}");
                assert!(line.ends_with(';'), "unterminated entity: {line}");
            }
        }
        let mut used = std::collections::BTreeSet::new();
        let bytes = data.as_bytes();
        let mut i = 0;
        while i < bytes.len() {
            if bytes[i] == b'#' && i + 1 < bytes.len() && bytes[i + 1].is_ascii_digit() {
                let start = i + 1;
                let mut end = start;
                while end < bytes.len() && bytes[end].is_ascii_digit() {
                    end += 1;
                }
                used.insert(
                    std::str::from_utf8(&bytes[start..end])
                        .unwrap()
                        .parse::<usize>()
                        .unwrap(),
                );
                i = end;
            } else {
                i += 1;
            }
        }
        let missing: Vec<_> = used.difference(&defined).collect();
        assert!(
            missing.is_empty(),
            "references without definitions: {missing:?}"
        );
    }

    #[test]
    fn names_are_step_string_literals() {
        let (mesh, transform, _) = box_part("ignored", [0.0; 3]);
        let name = "O'Brien & \"plate\"".to_string();
        let parts = [Part {
            name: &name,
            mesh: &mesh,
            transform,
        }];
        let text = String::from_utf8(step_bytes(&parts, "t'1").0).unwrap();
        assert!(text.contains("FACETED_BREP('O''Brien & \"plate\"'"));
        assert!(text.contains("FACETED_BREP_SHAPE_REPRESENTATION('t''1'"));
    }
}
