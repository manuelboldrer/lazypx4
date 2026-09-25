//! Geometry for the mission map: local-frame projection, great-circle
//! distance / bearing, the minimal KML parser (`kml.py`) and the lawnmower
//! coverage generator (`coverage.py`).

/// WGS-84 equatorial radius, metres.
pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Metres per degree of latitude (WGS-84 mean); good to ~0.5% anywhere.
pub const METRES_PER_DEG: f64 = 111_320.0;

/// Equirectangular projection of a lat/lon onto the local NED tangent plane
/// at the origin: (north, east) metres. Well under a metre of error over the
/// ranges the UI shows, and in the same frame as LOCAL_POSITION_NED.
pub fn global_to_local(lat: f64, lon: f64, origin_lat: f64, origin_lon: f64) -> (f64, f64) {
    let north = (lat - origin_lat).to_radians() * EARTH_RADIUS_M;
    let east = (lon - origin_lon).to_radians() * EARTH_RADIUS_M * origin_lat.to_radians().cos();
    (north, east)
}

/// Same projection, but `None` when the result isn't finite.
pub fn to_local(lat: f64, lon: f64, origin: (f64, f64)) -> Option<(f64, f64)> {
    let (n, e) = global_to_local(lat, lon, origin.0, origin.1);
    (n.is_finite() && e.is_finite()).then_some((n, e))
}

/// Great-circle distance, metres.
pub fn distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2) + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().min(1.0).asin()
}

/// Initial great-circle bearing 1 -> 2, compass degrees 0..360.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dlon = (lon2 - lon1).to_radians();
    let x = dlon.sin() * p2.cos();
    let y = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dlon.cos();
    x.atan2(y).to_degrees().rem_euclid(360.0)
}

/// Offset a lat/lon by (north, east) metres - the goto / jog conversion.
pub fn offset(lat: f64, lon: f64, north: f64, east: f64) -> (f64, f64) {
    (
        lat + north / METRES_PER_DEG,
        lon + east / (METRES_PER_DEG * lat.to_radians().cos().max(0.05)),
    )
}

// ---------------------------------------------------------------------------
// KML
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub name: String,
}

#[derive(Debug, Clone, Default)]
pub struct Kml {
    /// Every Point placemark plus every LineString vertex, document order.
    pub waypoints: Vec<Waypoint>,
    /// One (lat, lon) ring per Polygon outer boundary.
    pub fence_rings: Vec<Vec<(f64, f64)>>,
    /// Site altitude: mean of the file's non-zero altitudes (KML writes 0 for
    /// "no altitude"), used only to sanity-check the EKF altitude.
    pub ground_alt: Option<f64>,
}

fn coords(text: &str) -> Vec<(f64, f64, f64)> {
    text.split_whitespace()
        .filter_map(|chunk| {
            let mut parts = chunk.split(',');
            let lon: f64 = parts.next()?.parse().ok()?;
            let lat: f64 = parts.next()?.parse().ok()?;
            let alt: f64 = parts.next().and_then(|a| a.parse().ok()).unwrap_or(0.0);
            Some((lat, lon, alt))
        })
        .collect()
}

/// Minimal KML: Point (waypoint), LineString (ordered waypoints) and Polygon
/// (fence, outer ring). Styles, folders, extended data are ignored. A
/// visualization overlay only - never uploaded by itself.
pub fn parse_kml(path: &str) -> Result<Kml, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("could not read {path}: {e}"))?;
    let doc = roxmltree::Document::parse(&text).map_err(|e| format!("could not read {path}: {e}"))?;

    fn first<'a, 'i>(node: roxmltree::Node<'a, 'i>, tag: &str) -> Option<roxmltree::Node<'a, 'i>> {
        node.descendants().find(|n| n.is_element() && n.tag_name().name() == tag)
    }
    fn coords_of(node: roxmltree::Node<'_, '_>) -> Vec<(f64, f64, f64)> {
        first(node, "coordinates").and_then(|c| c.text()).map(coords).unwrap_or_default()
    }

    let mut kml = Kml::default();
    let mut altitudes = Vec::new();

    for placemark in doc.descendants().filter(|n| n.is_element() && n.tag_name().name() == "Placemark") {
        let name = first(placemark, "name").and_then(|n| n.text()).unwrap_or("").trim().to_string();

        if let Some(point) = first(placemark, "Point")
            && let Some(&(lat, lon, alt)) = coords_of(point).first() {
                kml.waypoints.push(Waypoint { lat, lon, alt, name: name.clone() });
                altitudes.push(alt);
            }
        if let Some(line) = first(placemark, "LineString") {
            for (i, (lat, lon, alt)) in coords_of(line).into_iter().enumerate() {
                let label = format!("{name} {}", i + 1).trim().to_string();
                kml.waypoints.push(Waypoint { lat, lon, alt, name: label });
                altitudes.push(alt);
            }
        }
        for polygon in placemark.descendants().filter(|n| n.is_element() && n.tag_name().name() == "Polygon") {
            let Some(outer) = first(polygon, "outerBoundaryIs") else { continue };
            let Some(ring) = first(outer, "LinearRing") else { continue };
            let points = coords_of(ring);
            altitudes.extend(points.iter().map(|p| p.2));
            if points.len() >= 3 {
                kml.fence_rings.push(points.into_iter().map(|(lat, lon, _)| (lat, lon)).collect());
            }
        }
    }

    if kml.waypoints.is_empty() && kml.fence_rings.is_empty() {
        return Err("no Point/LineString waypoints or Polygon fence found in this file".into());
    }
    let nonzero: Vec<f64> = altitudes.into_iter().filter(|a| *a != 0.0).collect();
    kml.ground_alt = (!nonzero.is_empty()).then(|| nonzero.iter().sum::<f64>() / nonzero.len() as f64);
    Ok(kml)
}

/// A ring without KML's repeated closing vertex.
pub fn open_ring(ring: &[(f64, f64)]) -> &[(f64, f64)] {
    if ring.len() >= 2 && ring.first() == ring.last() { &ring[..ring.len() - 1] } else { ring }
}

// ---------------------------------------------------------------------------
// Lawnmower coverage
// ---------------------------------------------------------------------------

/// Refuse more waypoints than this - almost certainly a spacing typo.
pub const MAX_COVERAGE_WAYPOINTS: usize = 500;

/// Boustrophedon waypoints covering `ring`: parallel lines `spacing_m`
/// apart at compass heading `angle_deg` (None = along the longest edge,
/// fewest turns), each shortened by `end_inset_m` at both ends so the
/// vehicle doesn't fly exactly along the fence. Two points per segment.
pub fn coverage_path(ring: &[(f64, f64)], spacing_m: f64, angle_deg: Option<f64>, end_inset_m: f64) -> Result<Vec<(f64, f64)>, String> {
    if spacing_m.is_nan() || spacing_m <= 0.0 {
        return Err("spacing must be greater than 0 m".into());
    }
    let ring = open_ring(ring);
    if ring.len() < 3 {
        return Err("polygon needs at least 3 vertices".into());
    }
    let lat0 = ring.iter().map(|p| p.0).sum::<f64>() / ring.len() as f64;
    let lon0 = ring.iter().map(|p| p.1).sum::<f64>() / ring.len() as f64;
    let k_lon = lat0.to_radians().cos();
    // (east, north) metres around the centroid.
    let points: Vec<(f64, f64)> = ring
        .iter()
        .map(|&(lat, lon)| ((lon - lon0).to_radians() * EARTH_RADIUS_M * k_lon, (lat - lat0).to_radians() * EARTH_RADIUS_M))
        .collect();
    let edges = || points.iter().zip(points.iter().cycle().skip(1)).take(points.len());

    let angle = angle_deg.unwrap_or_else(|| {
        let (&(e1, n1), &(e2, n2)) = edges()
            .max_by(|a, b| {
                let la = (a.1.0 - a.0.0).hypot(a.1.1 - a.0.1);
                let lb = (b.1.0 - b.0.0).hypot(b.1.1 - b.0.1);
                la.total_cmp(&lb)
            })
            .unwrap();
        (e2 - e1).atan2(n2 - n1).to_degrees().rem_euclid(180.0)
    });

    // u runs along the sweep lines, lines are stacked along v.
    let a = angle.to_radians();
    let (ue, un) = (a.sin(), a.cos());
    let (ve, vn) = (un, -ue);
    let rot: Vec<(f64, f64)> = points.iter().map(|&(e, n)| (e * ue + n * un, e * ve + n * vn)).collect();
    let v_min = rot.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let v_max = rot.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    if v_max - v_min <= 0.0 {
        return Err("polygon has no area".into());
    }

    let mut segments: Vec<(f64, Vec<(f64, f64)>)> = Vec::new();
    let mut v = v_min + spacing_m / 2.0;
    while v < v_max {
        let mut crossings: Vec<f64> = rot
            .iter()
            .zip(rot.iter().cycle().skip(1))
            .take(rot.len())
            .filter(|((_, v1), (_, v2))| (*v1 <= v && v < *v2) || (*v2 <= v && v < *v1))
            .map(|(&(u1, v1), &(u2, v2))| u1 + (v - v1) / (v2 - v1) * (u2 - u1))
            .collect();
        crossings.sort_by(f64::total_cmp);
        let line: Vec<(f64, f64)> = crossings
            .chunks_exact(2)
            .map(|c| (c[0] + end_inset_m, c[1] - end_inset_m))
            .filter(|(s, e)| e > s)
            .collect();
        if !line.is_empty() {
            segments.push((v, line));
        }
        v += spacing_m;
    }
    if segments.is_empty() {
        return Err("no sweep line fits inside the polygon - try a smaller spacing".into());
    }

    let mut path = Vec::new();
    for (i, (v, line)) in segments.into_iter().enumerate() {
        // Every other pass runs back the way it came.
        let line: Vec<(f64, f64)> = if i % 2 == 1 { line.into_iter().rev().map(|(s, e)| (e, s)).collect() } else { line };
        for (from, to) in line {
            for u in [from, to] {
                let (east, north) = (u * ue + v * ve, u * un + v * vn);
                path.push((
                    lat0 + (north / EARTH_RADIUS_M).to_degrees(),
                    lon0 + (east / (EARTH_RADIUS_M * k_lon)).to_degrees(),
                ));
            }
        }
    }
    if path.len() > MAX_COVERAGE_WAYPOINTS {
        return Err(format!("{} waypoints (max {MAX_COVERAGE_WAYPOINTS}) - use a larger spacing", path.len()));
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kml_points_lines_and_polygons() {
        let dir = std::env::temp_dir().join(format!("lazypx4-kml-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.kml");
        std::fs::write(
            &path,
            r#"<?xml version="1.0"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document>
<Placemark><name>A</name><Point><coordinates>6.88,52.21,40</coordinates></Point></Placemark>
<Placemark><name>L</name><LineString><coordinates>6.881,52.211,0 6.882,52.212,0</coordinates></LineString></Placemark>
<Placemark><Polygon><outerBoundaryIs><LinearRing><coordinates>
6.88,52.21,42 6.89,52.21,42 6.89,52.22,42 6.88,52.21,42</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>
</Document></kml>"#,
        )
        .unwrap();
        let kml = parse_kml(path.to_str().unwrap()).unwrap();
        assert_eq!(kml.waypoints.len(), 3);
        assert_eq!(kml.waypoints[1].name, "L 1");
        assert_eq!(kml.fence_rings.len(), 1);
        assert_eq!(open_ring(&kml.fence_rings[0]).len(), 3);
        assert!((kml.ground_alt.unwrap() - 41.6).abs() < 1e-9);
    }

    #[test]
    fn coverage_of_a_square() {
        // ~111 m x ~68 m box; 10 m spacing, N-S lines.
        let ring = [(52.0, 6.0), (52.001, 6.0), (52.001, 6.001), (52.0, 6.001)];
        let path = coverage_path(&ring, 10.0, Some(0.0), 1.0).unwrap();
        assert_eq!(path.len() % 2, 0);
        assert!(path.len() >= 12);
        // Alternating direction: first pass goes north, second south.
        assert!(path[1].0 > path[0].0 && path[3].0 < path[2].0);
        assert!(coverage_path(&ring, 0.0, None, 1.0).is_err());
    }

    #[test]
    fn distance_and_bearing() {
        let d = distance_m(52.0, 6.0, 52.001, 6.0);
        assert!((d - 111.3).abs() < 0.5);
        assert!((bearing_deg(52.0, 6.0, 52.001, 6.0) - 0.0).abs() < 1e-6);
        assert!((bearing_deg(52.0, 6.0, 52.0, 6.001) - 90.0).abs() < 0.01);
    }
}
