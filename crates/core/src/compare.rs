//! Visual verification (M25): turn a render into a number an agent can act on.
//!
//! `compare_render` renders the candidate's silhouette with the same camera
//! math as [`crate::render`] and compares it with a reference image (PNG) or
//! mesh (STL/OBJ/3MF). The silhouette is cropped to its own bbox before both
//! sides are resampled onto a fixed grid, so a different reference framing or
//! output resolution does not dominate the score. What remains — shape, the
//! bbox aspect ratio and which regions have missing/extra material — is the
//! part a designer actually cares about.
//!
//! Everything here is deterministic: PNG decoding, border-color background
//! detection, grid resampling and the comparison order are all fixed.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use wright_protocol::{
    CompareRegion, CompareView, Mesh, RenderViewScore, SilhouetteBox, Size3Mm, SizeMm,
    VolumeComparison,
};

use crate::render::{self, Camera, RenderItem};
use crate::{Error, Result};

/// The aligned comparison grid. Both silhouettes are cropped to their bbox
/// and resampled to this many cells on each side, which makes IoU independent
/// of framing and output resolution.
const ALIGN_GRID: usize = 256;

const PNG_SIGNATURE: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

/// A reference-image scale bound to one document.
#[derive(Debug, Clone)]
pub struct Calibration {
    pub mm_per_px: f64,
    /// Canonical path the scale belongs to; `None` applies to any image.
    pub reference: Option<String>,
    /// `two_points`, `known_length` or `mm_per_px`.
    pub source: String,
}

/// One reference silhouette, held at its native resolution.
#[derive(Debug, Clone)]
pub struct Bitmap {
    pub width: usize,
    pub height: usize,
    pub mask: Vec<bool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PixelBounds {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

impl Bitmap {
    pub fn from_mask(width: usize, height: usize, mask: Vec<bool>) -> Result<Self> {
        if width == 0 || height == 0 || mask.len() != width * height {
            return Err(Error::Invalid(format!(
                "bitmap shape {width}x{height} does not match {} mask pixels",
                mask.len()
            )));
        }
        Ok(Self { width, height, mask })
    }

    pub fn get(&self, x: usize, y: usize) -> bool {
        x < self.width && y < self.height && self.mask[y * self.width + x]
    }

    pub fn bounds(&self) -> Option<PixelBounds> {
        let mut min_x = self.width;
        let mut min_y = self.height;
        let mut max_x = 0usize;
        let mut max_y = 0usize;
        let mut any = false;
        for y in 0..self.height {
            for x in 0..self.width {
                if self.mask[y * self.width + x] {
                    any = true;
                    min_x = min_x.min(x);
                    min_y = min_y.min(y);
                    max_x = max_x.max(x);
                    max_y = max_y.max(y);
                }
            }
        }
        any.then(|| PixelBounds {
            x: min_x,
            y: min_y,
            width: max_x - min_x + 1,
            height: max_y - min_y + 1,
        })
    }

    pub fn count(&self) -> usize {
        self.mask.iter().filter(|&&v| v).count()
    }
}

/// Options collected from the `compare_render` command.
#[derive(Debug, Clone, Default)]
pub struct CompareOptions {
    pub reference: Option<String>,
    pub view: Option<String>,
    pub views: Option<Vec<CompareView>>,
    pub size: Option<u32>,
    pub regions: Option<u32>,
    pub tolerance: Option<f64>,
    pub min_iou: Option<f64>,
    pub volumetric: Option<bool>,
}

/// The engine-facing result. `CommandResult::RenderCompared` flattens these
/// fields onto the wire.
#[derive(Debug, Clone)]
pub struct Comparison {
    pub views: Vec<RenderViewScore>,
    pub iou: f64,
    pub min_iou: f64,
    pub worst_view: String,
    pub aspect_delta: f64,
    pub passed: bool,
    pub threshold: f64,
    pub needs_calibration: bool,
    pub message: Option<String>,
    pub volumes: Vec<VolumeComparison>,
}

/// Resolve `a`/`b`/`length_mm`, `pixels`/`length_mm`, or `mm_per_px` into a
/// millimeters-per-pixel scale and the source name for the report.
pub fn compute_calibration(
    a: Option<[f64; 2]>,
    b: Option<[f64; 2]>,
    length_mm: Option<f64>,
    pixels: Option<f64>,
    mm_per_px: Option<f64>,
) -> Result<(f64, &'static str)> {
    let two_points = a.is_some() || b.is_some();
    if mm_per_px.is_some() {
        if two_points || length_mm.is_some() || pixels.is_some() {
            return Err(Error::Invalid(
                "set_calibration takes either mm_per_px or a known length, not both (hint: drop \
                 mm_per_px when giving a and b + length_mm)"
                    .into(),
            ));
        }
        let v = mm_per_px.unwrap();
        if !v.is_finite() || v <= 0.0 {
            return Err(Error::Invalid(format!(
                "mm_per_px must be a finite value > 0 (got {v}; hint: 0.2 means one reference pixel \
                 is 0.2 mm)"
            )));
        }
        return Ok((v, "mm_per_px"));
    }

    if two_points {
        let (a, b) = match (a, b) {
            (Some(a), Some(b)) => (a, b),
            _ => {
                return Err(Error::Invalid(
                    "set_calibration needs both `a` and `b` pixel points (hint: [x, y] from the \
                     reference image, 0,0 at the top-left)"
                        .into(),
                ))
            },
        };
        let length = positive("length_mm", length_mm)?;
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let span = (dx * dx + dy * dy).sqrt();
        if !span.is_finite() || span <= 0.0 {
            return Err(Error::Invalid(
                "the calibration points must be different (hint: pick two points on a known edge)"
                    .into(),
            ));
        }
        return Ok((length / span, "two_points"));
    }

    if length_mm.is_some() || pixels.is_some() {
        let length = positive("length_mm", length_mm)?;
        let span = positive("pixels", pixels)?;
        return Ok((length / span, "known_length"));
    }

    Err(Error::Invalid(
        "set_calibration needs a scale: give a + b + length_mm, pixels + length_mm, or mm_per_px \
         (hint: ask the user for one known dimension in the reference image)"
            .into(),
    ))
}

fn positive(name: &str, value: Option<f64>) -> Result<f64> {
    let v = value.ok_or_else(|| {
        Error::Invalid(format!(
            "set_calibration needs {name} with that scale (hint: lengths are millimeters)"
        ))
    })?;
    if !v.is_finite() || v <= 0.0 {
        return Err(Error::Invalid(format!(
            "{name} must be a finite value > 0 (got {v})"
        )));
    }
    Ok(v)
}

/// Read the pixel dimensions of a PNG without decoding its full frame.
pub fn image_dimensions(path: &str) -> Result<(u32, u32)> {
    let file = File::open(path).map_err(|e| {
        Error::Document(format!(
            "cannot read reference image '{path}': {e} (hint: use an absolute path the server can read)"
        ))
    })?;
    let decoder = png::Decoder::new(BufReader::new(file));
    let reader = decoder.read_info().map_err(|e| {
        Error::Document(format!(
            "cannot decode reference image '{path}': {e} (hint: PNG is the supported reference \
             image format)"
        ))
    })?;
    let info = reader.info();
    Ok((info.width, info.height))
}

/// Decode a PNG and turn everything that differs from its border color into a
/// foreground mask. Transparent PNGs use their alpha channel directly.
pub fn load_png_mask(path: &str, tolerance: f64) -> Result<Bitmap> {
    let file = File::open(path).map_err(|e| {
        Error::Document(format!(
            "cannot read reference image '{path}': {e} (hint: use an absolute path the server can read)"
        ))
    })?;
    let mut decoder = png::Decoder::new(BufReader::new(file));
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().map_err(|e| {
        Error::Document(format!(
            "cannot decode reference image '{path}': {e} (hint: PNG is the supported reference \
             image format)"
        ))
    })?;
    let (width, height) = {
        let info = reader.info();
        (info.width, info.height)
    };
    let need = reader.output_buffer_size().ok_or_else(|| {
        Error::Document(format!(
            "reference image '{path}' is too large to decode (hint: {width}x{height} exceeds the \
             decoder's pixel limit)"
        ))
    })?;
    let mut buf = vec![0u8; need];
    let out = reader.next_frame(&mut buf).map_err(|e| {
        Error::Document(format!(
            "cannot decode reference image '{path}': {e} (hint: PNG is the supported reference \
             image format)"
        ))
    })?;
    let data = &buf[..out.buffer_size()];
    let channels = match out.color_type {
        png::ColorType::Grayscale => 1,
        png::ColorType::GrayscaleAlpha => 2,
        png::ColorType::Rgb => 3,
        png::ColorType::Rgba => 4,
        png::ColorType::Indexed => {
            return Err(Error::Document(format!(
                "reference image '{path}' stayed paletted after decoding (hint: re-save it as RGB \
                 or RGBA PNG)"
            )))
        },
    };
    let mask = mask_from_pixels(data, out.width as usize, out.height as usize, channels, tolerance)?;
    Bitmap::from_mask(out.width as usize, out.height as usize, mask)
}

/// Extract a foreground mask from raw 8-bit pixels. `channels` is 1, 2, 3 or
/// 4. Alpha, when any pixel is not fully opaque, wins; otherwise the dominant
/// border color is the background and pixels farther than `tolerance` from it
/// are foreground.
pub fn mask_from_pixels(
    pixels: &[u8],
    width: usize,
    height: usize,
    channels: usize,
    tolerance: f64,
) -> Result<Vec<bool>> {
    if !(1..=4).contains(&channels) {
        return Err(Error::Invalid(format!(
            "unsupported reference image channel count {channels} (hint: RGB or RGBA PNG)"
        )));
    }
    if width == 0 || height == 0 || pixels.len() < width * height * channels {
        return Err(Error::Document(
            "reference image has no pixels (hint: check the PNG is not empty)".into(),
        ));
    }
    let alpha = if channels == 2 || channels == 4 {
        let ai = channels - 1;
        (0..width * height).any(|i| pixels[i * channels + ai] < 250)
    } else {
        false
    };
    if alpha {
        let ai = channels - 1;
        return Ok((0..width * height)
            .map(|i| pixels[i * channels + ai] >= 128)
            .collect());
    }

    let bg = dominant_border_color(pixels, width, height, channels);
    let diff = (tolerance.clamp(0.0, 1.0) * 255.0).round() as i32;
    Ok((0..width * height)
        .map(|i| {
            let (r, g, b) = rgb_at(pixels, i, channels);
            (r - bg[0]).abs().max((g - bg[1]).abs()).max((b - bg[2]).abs()) > diff
        })
        .collect())
}

fn rgb_at(pixels: &[u8], i: usize, channels: usize) -> (i32, i32, i32) {
    match channels {
        1 | 2 => {
            let v = pixels[i * channels] as i32;
            (v, v, v)
        },
        _ => {
            let p = i * channels;
            (pixels[p] as i32, pixels[p + 1] as i32, pixels[p + 2] as i32)
        },
    }
}

/// The most common RGB color on the image border. Ties resolve to the
/// lexicographically smallest color, so the choice is deterministic.
fn dominant_border_color(pixels: &[u8], width: usize, height: usize, channels: usize) -> [i32; 3] {
    let mut counts: BTreeMap<[u8; 3], usize> = BTreeMap::new();
    let mut add = |x: usize, y: usize| {
        let (r, g, b) = rgb_at(pixels, y * width + x, channels);
        *counts.entry([r as u8, g as u8, b as u8]).or_insert(0) += 1;
    };
    for x in 0..width {
        add(x, 0);
        if height > 1 {
            add(x, height - 1);
        }
    }
    for y in 1..height.saturating_sub(1) {
        add(0, y);
        if width > 1 {
            add(width - 1, y);
        }
    }
    let mut best = [255u8, 255, 255];
    let mut best_count = 0usize;
    for (color, count) in counts {
        if count > best_count || (count == best_count && color < best) {
            best = color;
            best_count = count;
        }
    }
    [best[0] as i32, best[1] as i32, best[2] as i32]
}

/// Compare the scene's silhouette against one reference image or mesh.
pub fn compare(
    items: &[RenderItem<'_>],
    options: &CompareOptions,
    calibration: Option<&Calibration>,
) -> Result<Comparison> {
    let size = options.size.unwrap_or(512).clamp(64, 2048);
    let regions_per_axis = options.regions.unwrap_or(3).clamp(1, 8);
    let tolerance = options.tolerance.unwrap_or(0.08);
    if !tolerance.is_finite() || !(0.0..=1.0).contains(&tolerance) {
        return Err(Error::Invalid(format!(
            "tolerance must be between 0 and 1 (got {tolerance}; hint: 0.08 is a good default)"
        )));
    }
    let threshold = options.min_iou.unwrap_or(0.85);
    if !threshold.is_finite() || !(0.0..=1.0).contains(&threshold) {
        return Err(Error::Invalid(format!(
            "min_iou must be between 0 and 1 (got {threshold}; hint: 0.85 is a good default)"
        )));
    }

    let specs = view_specs(options)?;
    let mut loaded: BTreeMap<String, RefData> = BTreeMap::new();
    for spec in &specs {
        let path = spec
            .reference
            .as_deref()
            .or(options.reference.as_deref())
            .ok_or_else(|| {
                Error::Invalid(format!(
                    "view '{}' has no reference (hint: pass `reference`, or a `reference` inside \
                     that view)",
                    spec.view
                ))
            })?;
        let key = canonical_path(path);
        if !loaded.contains_key(&key) {
            loaded.insert(key, load_reference(path, tolerance)?);
        }
    }

    let want_volumes = options
        .volumetric
        .unwrap_or_else(|| loaded.values().any(|d| matches!(d, RefData::Mesh { .. })));
    if want_volumes && !loaded.values().any(|d| matches!(d, RefData::Mesh { .. })) {
        return Err(Error::Invalid(
            "volumetric comparison needs a mesh reference (STL/OBJ/3MF); this comparison only used \
             images (hint: set volumetric:false for a 2D-only score, or pass a mesh)"
                .into(),
        ));
    }
    let mut views = Vec::with_capacity(specs.len());
    let mut volumes = Vec::new();
    let mut needs_calibration = false;

    for spec in &specs {
        let view = canonical_view(&spec.view)?;
        let path = spec.reference.as_deref().or(options.reference.as_deref()).unwrap();
        let key = canonical_path(path);
        let data = loaded.get(&key).expect("reference loaded above");

        let camera = named_view_camera(items, &view, size)?;
        let model_mask =
            Bitmap::from_mask(size as usize, size as usize, render::render_mask(
                items,
                camera,
                size,
                size,
            ))?;
        let model_bbox = model_mask.bounds().ok_or_else(|| {
            Error::Invalid(format!(
                "the model renders to an empty silhouette in view '{view}' (hint: add or move a \
                 solid into the camera's view)"
            ))
        })?;
        let model_scale = render::ortho_mm_per_px(camera, items, size);

        // A mesh reference is rendered on demand; the bitmap has to outlive
        // the score call, so build it into a local slot for this iteration.
        let mesh_render = if let RefData::Mesh { mesh } = data {
            let ref_items = [RenderItem {
                mesh,
                model: crate::mat4::identity(),
                color: [1.0, 1.0, 1.0, 1.0],
            }];
            let ref_camera = named_view_camera(&ref_items, &view, size)?;
            let bitmap = Bitmap::from_mask(
                size as usize,
                size as usize,
                render::render_mask(&ref_items, ref_camera, size, size),
            )?;
            Some((bitmap, render::ortho_mm_per_px(ref_camera, &ref_items, size)))
        } else {
            None
        };
        let (reference_bitmap, reference_width, reference_height, reference_kind) = match data {
            RefData::Image { bitmap } => {
                let (w, h) = (bitmap.width as u32, bitmap.height as u32);
                (bitmap, Some(w), Some(h), "image")
            },
            RefData::Mesh { .. } => (
                &mesh_render.as_ref().expect("mesh reference rendered above").0,
                None,
                None,
                "mesh",
            ),
        };
        let reference_bbox = reference_bitmap.bounds().ok_or_else(|| {
            Error::Invalid(format!(
                "reference '{path}' has no foreground pixels (hint: use a plain background or a \
                 transparent PNG; the silhouette needs something to compare)"
            ))
        })?;

        let cal_applies = calibration.filter(|cal| cal.applies_to(&key));
        let ref_scale = match reference_kind {
            "image" => cal_applies.map(|c| c.mm_per_px),
            _ => mesh_render.as_ref().and_then(|(_, scale)| *scale),
        };
        let view_needs_calibration = reference_kind == "image" && ref_scale.is_none();
        needs_calibration |= view_needs_calibration;

        let score = score_view(
            &view,
            &key,
            reference_kind,
            reference_width,
            reference_height,
            &model_mask,
            reference_bitmap,
            model_bbox,
            reference_bbox,
            model_scale,
            ref_scale,
            regions_per_axis,
            view_needs_calibration,
        );

        if want_volumes {
            if let RefData::Mesh { mesh } = data {
                let (model_volume, model_size) = volume_and_size(items);
                let (reference_volume, reference_size) = mesh_volume_and_size(mesh);
                volumes.push(VolumeComparison {
                    view: view.clone(),
                    reference: key.clone(),
                    model_mm3: model_volume,
                    reference_mm3: reference_volume,
                    delta_mm3: model_volume - reference_volume,
                    ratio: (reference_volume.abs() > 1e-9)
                        .then_some(model_volume / reference_volume),
                    model_size_mm: model_size,
                    reference_size_mm: reference_size,
                });
            }
        }
        views.push(score);
    }

    let iou = views.iter().map(|v| v.iou).sum::<f64>() / views.len() as f64;
    let min_iou = views.iter().map(|v| v.iou).fold(1.0, f64::min);
    let worst_view = views
        .iter()
        .min_by(|a, b| a.iou.total_cmp(&b.iou))
        .map(|v| v.view.clone())
        .unwrap_or_default();
    let aspect_delta =
        views.iter().map(|v| v.aspect_delta).sum::<f64>() / views.len() as f64;
    let passed = views.iter().all(|v| v.iou >= threshold);
    let message = if needs_calibration {
        Some(
            "no reference scale: silhouette IoU and aspect are scale-free, but physical sizes need \
             calibration. Ask the user for one known dimension, then call set_calibration with two \
             pixel points + length_mm, pixels + length_mm, or mm_per_px."
                .to_string(),
        )
    } else if !passed {
        Some(format!(
            "silhouette IoU {min_iou:.3} is below the {threshold:.2} threshold in view \
             '{worst_view}'; the region report shows where material is missing or extra"
        ))
    } else {
        None
    };

    Ok(Comparison {
        views,
        iou,
        min_iou,
        worst_view,
        aspect_delta,
        passed,
        threshold,
        needs_calibration,
        message,
        volumes,
    })
}

impl Calibration {
    fn applies_to(&self, canonical_reference: &str) -> bool {
        self.reference.as_deref().map_or(true, |p| p == canonical_reference)
    }
}

enum RefData {
    Image { bitmap: Bitmap },
    Mesh { mesh: Mesh },
}

fn load_reference(path: &str, tolerance: f64) -> Result<RefData> {
    let bytes = std::fs::read(path).map_err(|e| {
        Error::Document(format!(
            "cannot read reference '{path}': {e} (hint: use an absolute path the server can read)"
        ))
    })?;
    if bytes.starts_with(&PNG_SIGNATURE) {
        return Ok(RefData::Image {
            bitmap: load_png_mask(path, tolerance)?,
        });
    }
    let imported = crate::mesh_import::import_file(path, None, 1.0)?;
    let (mesh, _) = crate::mesh_import::build(&imported.positions, &imported.indices, 30.0)?;
    Ok(RefData::Mesh { mesh })
}

fn view_specs(options: &CompareOptions) -> Result<Vec<CompareView>> {
    match (&options.view, &options.views) {
        (Some(_), Some(_)) => Err(Error::Invalid(
            "compare_render takes `view` or `views`, not both (hint: use `views` for multi-view)"
                .into(),
        )),
        (_, Some(views)) if views.is_empty() => Err(Error::Invalid(
            "compare_render needs at least one view (hint: views: [{\"view\": \"front\"}])".into(),
        )),
        (_, Some(views)) => Ok(views.clone()),
        (Some(view), None) => Ok(vec![CompareView {
            view: view.clone(),
            reference: None,
        }]),
        (None, None) => Ok(vec![CompareView { view: "iso".into(), reference: None }]),
    }
}

/// Canonicalize a path for calibration matching, keeping the caller's text
/// when the file does not exist (the later read will report the real error).
pub fn canonical_path(path: &str) -> String {
    std::fs::canonicalize(Path::new(path))
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string())
}

fn canonical_view(view: &str) -> Result<String> {
    let key = view.trim().to_ascii_lowercase();
    let canonical = match key.as_str() {
        "iso" | "auto" | "default" | "3/4" | "three_quarter" | "three-quarter" => "iso",
        "front" | "f" | "xz" => "front",
        "back" | "rear" | "b" => "back",
        "left" | "l" => "left",
        "right" | "side" | "r" | "yz" => "right",
        "top" | "plan" | "above" | "xy" => "top",
        "bottom" | "under" | "below" => "bottom",
        _ => {
            return Err(Error::Invalid(format!(
                "unknown view '{view}' (hint: front, back, left, right, top, bottom or iso)"
            )))
        },
    };
    Ok(canonical.to_string())
}

fn named_view_camera(items: &[RenderItem<'_>], view: &str, size: u32) -> Result<Camera> {
    let mut camera = Camera::auto(items, size, size);
    if view == "iso" {
        camera.ortho = true;
        return Ok(camera);
    }
    let (center, _) = render::scene_bounds(items);
    let distance = norm([
        camera.eye[0] - center[0],
        camera.eye[1] - center[1],
        camera.eye[2] - center[2],
    ])
    .max(1e-3);
    let (dir, up) = match view {
        "front" => ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
        "back" => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        "left" => ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        "right" => ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        "top" => ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
        "bottom" => ([0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),
        _ => unreachable!("view names are canonicalized first"),
    };
    camera.eye = [
        center[0] + dir[0] * distance,
        center[1] + dir[1] * distance,
        center[2] + dir[2] * distance,
    ];
    camera.target = center;
    camera.up = up;
    camera.ortho = true;
    Ok(camera)
}

#[allow(clippy::too_many_arguments)]
fn score_view(
    view: &str,
    reference: &str,
    reference_kind: &str,
    reference_width: Option<u32>,
    reference_height: Option<u32>,
    model: &Bitmap,
    reference_mask: &Bitmap,
    model_bbox: PixelBounds,
    reference_bbox: PixelBounds,
    model_scale: Option<f64>,
    reference_scale: Option<f64>,
    regions_per_axis: u32,
    needs_calibration: bool,
) -> RenderViewScore {
    let model_aspect = model_bbox.width as f64 / model_bbox.height.max(1) as f64;
    let reference_aspect =
        reference_bbox.width as f64 / reference_bbox.height.max(1) as f64;
    let aspect_delta = if reference_aspect > 0.0 {
        ((model_aspect - reference_aspect) / reference_aspect).abs()
    } else {
        0.0
    };

    let model_grid = resample(model, model_bbox, ALIGN_GRID);
    let reference_grid = resample(reference_mask, reference_bbox, ALIGN_GRID);
    let (iou, _) = iou_counts(&model_grid, &reference_grid);
    let regions = region_scores(
        &model_grid,
        &reference_grid,
        regions_per_axis as usize,
    );

    let model_size_mm = model_scale.map(|s| SizeMm {
        width: model_bbox.width as f64 * s,
        height: model_bbox.height as f64 * s,
    });
    let reference_size_mm = reference_scale.map(|s| SizeMm {
        width: reference_bbox.width as f64 * s,
        height: reference_bbox.height as f64 * s,
    });
    let size_delta_mm = match (model_size_mm, reference_size_mm) {
        (Some(m), Some(r)) => Some(SizeMm {
            width: m.width - r.width,
            height: m.height - r.height,
        }),
        _ => None,
    };

    RenderViewScore {
        view: view.to_string(),
        reference: reference.to_string(),
        reference_kind: reference_kind.to_string(),
        reference_width,
        reference_height,
        iou,
        aspect_delta,
        model_bbox: pixel_box(model_bbox),
        reference_bbox: pixel_box(reference_bbox),
        regions,
        model_size_mm,
        reference_size_mm,
        size_delta_mm,
        needs_calibration,
    }
}

fn pixel_box(b: PixelBounds) -> SilhouetteBox {
    SilhouetteBox {
        x: b.x as u32,
        y: b.y as u32,
        width: b.width as u32,
        height: b.height as u32,
        aspect: b.width as f64 / b.height.max(1) as f64,
    }
}

fn integral(mask: &Bitmap) -> Vec<u32> {
    let stride = mask.width + 1;
    let mut out = vec![0u32; stride * (mask.height + 1)];
    for y in 0..mask.height {
        let mut row = 0u32;
        for x in 0..mask.width {
            row += mask.mask[y * mask.width + x] as u32;
            out[(y + 1) * stride + x + 1] = out[y * stride + x + 1] + row;
        }
    }
    out
}

fn rect_sum(integral: &[u32], width: usize, x0: usize, y0: usize, x1: usize, y1: usize) -> u32 {
    let stride = width + 1;
    let s = |x: usize, y: usize| integral[y * stride + x] as i64;
    (s(x1, y1) - s(x0, y1) - s(x1, y0) + s(x0, y0)) as u32
}

/// Crop one silhouette to `bounds` and box-filter it onto a square grid.
fn resample(mask: &Bitmap, bounds: PixelBounds, grid: usize) -> Vec<bool> {
    let integ = integral(mask);
    let mut out = vec![false; grid * grid];
    for gy in 0..grid {
        let y0 = bounds.y + gy * bounds.height / grid;
        let y1 = (bounds.y + (gy + 1) * bounds.height / grid)
            .max(y0 + 1)
            .min(bounds.y + bounds.height);
        for gx in 0..grid {
            let x0 = bounds.x + gx * bounds.width / grid;
            let x1 = (bounds.x + (gx + 1) * bounds.width / grid)
                .max(x0 + 1)
                .min(bounds.x + bounds.width);
            let sum = rect_sum(&integ, mask.width, x0, y0, x1, y1) as u64;
            let area = ((x1 - x0) * (y1 - y0)) as u64;
            out[gy * grid + gx] = sum * 2 >= area;
        }
    }
    out
}

fn iou_counts(a: &[bool], b: &[bool]) -> (f64, usize) {
    let mut intersection = 0usize;
    let mut union = 0usize;
    for (x, y) in a.iter().zip(b) {
        if *x || *y {
            union += 1;
            if *x && *y {
                intersection += 1;
            }
        }
    }
    let iou = if union == 0 { 1.0 } else { intersection as f64 / union as f64 };
    (iou, union)
}

fn region_scores(model: &[bool], reference: &[bool], n: usize) -> Vec<CompareRegion> {
    let mut out = Vec::with_capacity(n * n);
    for row in 0..n {
        let r0 = row * ALIGN_GRID / n;
        let r1 = (row + 1) * ALIGN_GRID / n;
        for col in 0..n {
            let c0 = col * ALIGN_GRID / n;
            let c1 = (col + 1) * ALIGN_GRID / n;
            let (mut inter, mut union, mut missing, mut extra) = (0usize, 0usize, 0usize, 0usize);
            for y in r0..r1 {
                for x in c0..c1 {
                    let i = y * ALIGN_GRID + x;
                    let (m, r) = (model[i], reference[i]);
                    if m || r {
                        union += 1;
                        if m && r {
                            inter += 1;
                        } else if r {
                            missing += 1;
                        } else {
                            extra += 1;
                        }
                    }
                }
            }
            let iou = if union == 0 { 1.0 } else { inter as f64 / union as f64 };
            out.push(CompareRegion {
                name: region_name(row, col, n),
                row: row as u32,
                col: col as u32,
                iou,
                mismatch: 1.0 - iou,
                missing: if union == 0 { 0.0 } else { missing as f64 / union as f64 },
                extra: if union == 0 { 0.0 } else { extra as f64 / union as f64 },
                union_pixels: union as u32,
            });
        }
    }
    out
}

fn region_name(row: usize, col: usize, n: usize) -> String {
    if n != 3 {
        return format!("r{row}c{col}");
    }
    let r = ["top", "middle", "bottom"][row];
    let c = ["left", "center", "right"][col];
    match (r, c) {
        ("middle", "center") => "center".to_string(),
        ("middle", c) => c.to_string(),
        (r, "center") => r.to_string(),
        (r, c) => format!("{r}-{c}"),
    }
}

fn volume_and_size(items: &[RenderItem<'_>]) -> (f64, Size3Mm) {
    let mut volume = 0.0;
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    let mut any = false;
    for item in items {
        let world = crate::transform_mesh(item.mesh, &item.model);
        volume += crate::fillet::signed_volume(&world).abs();
        for p in &world.positions {
            any = true;
            for i in 0..3 {
                lo[i] = lo[i].min(p[i] as f64);
                hi[i] = hi[i].max(p[i] as f64);
            }
        }
    }
    let size = if any {
        Size3Mm {
            x: hi[0] - lo[0],
            y: hi[1] - lo[1],
            z: hi[2] - lo[2],
        }
    } else {
        Size3Mm { x: 0.0, y: 0.0, z: 0.0 }
    };
    (volume, size)
}

fn mesh_volume_and_size(mesh: &Mesh) -> (f64, Size3Mm) {
    let volume = crate::fillet::signed_volume(mesh).abs();
    if mesh.positions.is_empty() {
        return (volume, Size3Mm { x: 0.0, y: 0.0, z: 0.0 });
    }
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i] as f64);
            hi[i] = hi[i].max(p[i] as f64);
        }
    }
    (
        volume,
        Size3Mm {
            x: hi[0] - lo[0],
            y: hi[1] - lo[1],
            z: hi[2] - lo[2],
        },
    )
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect_bitmap(width: usize, height: usize, x0: usize, y0: usize, x1: usize, y1: usize) -> Bitmap {
        let mut mask = vec![false; width * height];
        for y in y0..y1.min(height) {
            for x in x0..x1.min(width) {
                mask[y * width + x] = true;
            }
        }
        Bitmap::from_mask(width, height, mask).unwrap()
    }

    #[test]
    fn calibration_from_two_points_and_known_length() {
        let (scale, source) =
            compute_calibration(Some([10.0, 10.0]), Some([110.0, 10.0]), Some(50.0), None, None)
                .unwrap();
        assert_eq!(source, "two_points");
        assert!((scale - 0.5).abs() < 1e-12);

        let (scale, source) = compute_calibration(None, None, Some(200.0), Some(1000.0), None).unwrap();
        assert_eq!(source, "known_length");
        assert!((scale - 0.2).abs() < 1e-12);

        let (scale, source) = compute_calibration(None, None, None, None, Some(0.25)).unwrap();
        assert_eq!(source, "mm_per_px");
        assert!((scale - 0.25).abs() < 1e-12);
    }

    #[test]
    fn calibration_rejects_missing_or_impossible_inputs() {
        assert!(compute_calibration(Some([0.0, 0.0]), None, Some(10.0), None, None).is_err());
        assert!(compute_calibration(Some([5.0, 5.0]), Some([5.0, 5.0]), Some(10.0), None, None).is_err());
        assert!(compute_calibration(None, None, None, None, None).is_err());
        assert!(compute_calibration(None, None, Some(-1.0), Some(10.0), None).is_err());
        assert!(compute_calibration(None, None, None, None, Some(0.0)).is_err());
    }

    #[test]
    fn mask_uses_the_border_color_as_background() {
        // 5x4 white canvas with a 2x2 black block at (2,1).
        let (w, h) = (5usize, 4usize);
        let mut px = vec![255u8; w * h * 3];
        for y in 1..3 {
            for x in 2..4 {
                let i = (y * w + x) * 3;
                px[i..i + 3].fill(0);
            }
        }
        let mask = mask_from_pixels(&px, w, h, 3, 0.08).unwrap();
        assert!(mask[1 * w + 2] && mask[2 * w + 3]);
        assert!(!mask[0] && !mask[3 * w + 4]);
        let bitmap = Bitmap::from_mask(w, h, mask).unwrap();
        assert_eq!(
            bitmap.bounds(),
            Some(PixelBounds { x: 2, y: 1, width: 2, height: 2 })
        );
    }

    #[test]
    fn transparent_png_uses_alpha() {
        let (w, h) = (2usize, 2usize);
        let px = vec![
            0u8, 0, 0, 0, // transparent black
            255, 0, 0, 255, // opaque red
            255, 0, 0, 255, 255, 0, 0, 255,
        ];
        let mask = mask_from_pixels(&px, w, h, 4, 0.08).unwrap();
        assert_eq!(mask, vec![false, true, true, true]);
    }

    #[test]
    fn aligned_iou_ignores_framing_and_scale() {
        let a = rect_bitmap(100, 100, 10, 20, 50, 70);
        let b = rect_bitmap(200, 200, 20, 40, 100, 140);
        let (ba, bb) = (a.bounds().unwrap(), b.bounds().unwrap());
        let ga = resample(&a, ba, ALIGN_GRID);
        let gb = resample(&b, bb, ALIGN_GRID);
        let (iou, _) = iou_counts(&ga, &gb);
        assert!(iou > 0.99, "same shape at a different scale should match: {iou}");
        let aspect_a = ba.width as f64 / ba.height as f64;
        let aspect_b = bb.width as f64 / bb.height as f64;
        assert!((aspect_a - aspect_b).abs() < 1e-12);
    }

    #[test]
    fn per_region_mismatch_finds_a_missing_hole() {
        let solid = rect_bitmap(120, 120, 10, 10, 110, 110);
        let mut holed = solid.clone();
        for y in 50..70 {
            for x in 50..70 {
                holed.mask[y * holed.width + x] = false;
            }
        }
        let b = solid.bounds().unwrap();
        let gm = resample(&solid, b, ALIGN_GRID);
        let gr = resample(&holed, b, ALIGN_GRID);
        let (iou, _) = iou_counts(&gm, &gr);
        assert!(iou > 0.9, "a small hole keeps a good silhouette: {iou}");
        let regions = region_scores(&gm, &gr, 3);
        let center = regions.iter().find(|r| r.name == "center").unwrap();
        assert!(center.mismatch > 0.1, "center should see the hole: {center:?}");
        assert!(regions.iter().find(|r| r.name == "top-left").unwrap().mismatch < 0.01);
    }

    #[test]
    fn aspect_delta_is_relative() {
        let square = rect_bitmap(100, 100, 10, 10, 90, 90);
        let wide = rect_bitmap(100, 100, 10, 30, 90, 70);
        let b = square.bounds().unwrap();
        let m = wide.bounds().unwrap();
        let delta = ((b.width as f64 / b.height as f64)
            - (m.width as f64 / m.height as f64))
            .abs();
        assert!(delta > 0.5);
    }

    #[test]
    fn region_names_are_readable_for_3x3_and_generic_otherwise() {
        assert_eq!(region_name(0, 0, 3), "top-left");
        assert_eq!(region_name(1, 1, 3), "center");
        assert_eq!(region_name(2, 2, 3), "bottom-right");
        assert_eq!(region_name(0, 1, 2), "r0c1");
    }
}
