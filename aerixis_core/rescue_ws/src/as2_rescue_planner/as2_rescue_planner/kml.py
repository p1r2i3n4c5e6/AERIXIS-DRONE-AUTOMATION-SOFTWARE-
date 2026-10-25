"""KML ingestion. A KML file is an input boundary, never an executable mission.

Parsing is deliberately strict: unknown geometry is reported rather than
silently skipped, coordinates must be finite, and rings that fail validation are
rejected here so no later stage has to trust unvalidated geometry.
"""
import xml.etree.ElementTree as ET

from .geometry import PlannerRegion, Projection, finite

# KML documents in the wild use several namespaces; match on the local tag name.
MAX_DOCUMENT_BYTES = 8 * 1024 * 1024
MAX_PLACEMARKS = 512


def _local(tag):
    return tag.rsplit('}', 1)[-1] if '}' in tag else tag


def _text(element, name):
    for child in element:
        if _local(child.tag) == name:
            return (child.text or '').strip()
    return ''


def _elements(element, name):
    return [child for child in element.iter() if _local(child.tag) == name]


def _parse_coordinates(text, minimum=3):
    """KML tuples are longitude,latitude[,altitude], whitespace separated.

    ``minimum`` is 3 for a ring and 1 for a single Point placemark; enforcing
    the ring minimum on a Point would reject ordinary waypoint markers.
    """
    points = []
    for token in text.replace('\n', ' ').replace('\t', ' ').split():
        fields = token.split(',')
        if len(fields) < 2:
            raise ValueError('KML coordinate needs at least longitude,latitude')
        try:
            longitude = float(fields[0])
            latitude = float(fields[1])
            altitude = float(fields[2]) if len(fields) > 2 and fields[2].strip() else 0.0
        except ValueError:
            raise ValueError('KML coordinate is not numeric')
        if not finite(latitude) or not finite(longitude) or not finite(altitude):
            raise ValueError('KML coordinate is not finite')
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError('KML coordinate outside geographic bounds')
        points.append((latitude, longitude, altitude))
    if len(points) < minimum:
        raise ValueError('A KML element needs at least %d coordinate(s)' % minimum)
    # KML rings repeat the first point to close; the planner closes implicitly.
    while len(points) > 1 and points[0][:2] == points[-1][:2]:
        points.pop()
    return points


class KmlArea:
    """One validated polygon or exclusion, still in geographic coordinates."""

    def __init__(self, name, shell, holes=None):
        self.name = name
        self.shell = shell
        self.holes = holes or []
        altitudes = [point[2] for point in shell]
        self.altitude_range = (min(altitudes), max(altitudes))


class KmlPlan:
    """Validated KML content, ready to be projected and partitioned.

    ``points`` holds operator waypoint markers found in the document. They are
    reference information only: they never become coverage or an approval.
    ``routes`` holds LineStrings by placemark name: a route drawn for the aircraft of the same
    name (e.g. "drone0") is flown instead of its coverage lanes (mission.py).
    """

    def __init__(self, areas, exclusions, projection, points=None, routes=None):
        self.areas = areas
        self.exclusions = exclusions
        self.projection = projection
        self.points = points or []
        self.routes = routes or {}

    def route(self, name):
        """A drawn route projected into the planar frame, or None."""
        coordinates = self.routes.get(name)
        return None if coordinates is None else [self.projection.to_enu(lat, lon) for lat, lon, _ in coordinates]

    def region(self, area, extra_exclusions=None):
        """Project one KML area into a validated planar PlannerRegion."""
        exclusions = [[self.projection.to_enu(lat, lon) for lat, lon, _ in exclusion.shell]
                      for exclusion in self.exclusions] + list(extra_exclusions or [])
        shell = [self.projection.to_enu(lat, lon) for lat, lon, _ in area.shell]
        holes = [[self.projection.to_enu(lat, lon) for lat, lon, _ in ring] for ring in area.holes]
        return PlannerRegion(shell, holes, exclusions, name=area.name)


def parse_kml(data):
    """Parse and validate KML text into a KmlPlan. Raises ValueError on any problem."""
    if isinstance(data, bytes):
        if len(data) > MAX_DOCUMENT_BYTES:
            raise ValueError('KML document exceeds the size limit')
        try:
            root = ET.fromstring(data)
        except ET.ParseError as error:
            raise ValueError('KML is not well-formed XML: %s' % error)
    elif isinstance(data, str):
        if len(data.encode('utf-8')) > MAX_DOCUMENT_BYTES:
            raise ValueError('KML document exceeds the size limit')
        try:
            root = ET.fromstring(data)
        except ET.ParseError as error:
            raise ValueError('KML is not well-formed XML: %s' % error)
    else:
        raise ValueError('KML input must be text or bytes')

    placemarks = [element for element in root.iter() if _local(element.tag) == 'Placemark']
    if not placemarks:
        raise ValueError('KML contains no Placemark elements')
    if len(placemarks) > MAX_PLACEMARKS:
        raise ValueError('KML contains more than %d Placemarks' % MAX_PLACEMARKS)

    areas = []
    exclusions = []
    points = []
    routes = {}
    unsupported = []
    for placemark in placemarks:
        name = _text(placemark, 'name') or 'unnamed'
        is_exclusion = False
        for style in _elements(placemark, 'description'):
            if 'exclusion' in (style.text or '').lower():
                is_exclusion = True
        polygons = [element for element in placemark.iter() if _local(element.tag) == 'Polygon']
        if not polygons:
            # A Point placemark is an ordinary navigation marker and is common in
            # operator-authored KML (for example the supplied sample field
            # mission). It carries no area, so it is collected for reference and
            # never planned as coverage. Other geometry is still refused, because
            # silently ignoring a LineString or Model could hide a real intent.
            point_elements = [element for element in placemark.iter()
                              if _local(element.tag) == 'Point']
            if point_elements:
                for element in point_elements:
                    coordinates = _parse_coordinates(_text(element, 'coordinates'), minimum=1)
                    if len(coordinates) != 1:
                        raise ValueError('Point "%s" must contain exactly one coordinate' % name)
                    latitude, longitude, altitude = coordinates[0]
                    points.append(dict(name=name, latitude=latitude, longitude=longitude,
                                       altitude=altitude))
                continue
            lines = [element for element in placemark.iter() if _local(element.tag) == 'LineString']
            if len(lines) == 1:
                # A drawn route for the aircraft named like the placemark (mission.py decides).
                if name in routes:
                    raise ValueError('Two routes are named "%s"' % name)
                routes[name] = _parse_coordinates(_text(lines[0], 'coordinates'), minimum=2)
                continue
            other = sorted({_local(element.tag) for element in placemark.iter()}
                           & {'LineString', 'LinearRing', 'Model', 'Track'})
            if other:
                unsupported.append('%s (%s)' % (name, ', '.join(other)))
            continue
        for polygon in polygons:
            outer = None
            holes = []
            for child in polygon:
                tag = _local(child.tag)
                if tag == 'outerBoundaryIs':
                    rings = _elements(child, 'LinearRing')
                    if not rings:
                        raise ValueError('Polygon "%s" has an empty outer boundary' % name)
                    outer = _parse_coordinates(_text(rings[0], 'coordinates'))
                elif tag == 'innerBoundaryIs':
                    for ring in _elements(child, 'LinearRing'):
                        holes.append(_parse_coordinates(_text(ring, 'coordinates')))
            if outer is None:
                raise ValueError('Polygon "%s" has no outer boundary' % name)
            area = KmlArea(name, outer, holes)
            (exclusions if is_exclusion else areas).append(area)

    if not areas:
        detail = '; unsupported: ' + ', '.join(unsupported) if unsupported else ''
        raise ValueError('KML contains no usable Polygon area' + detail)
    if unsupported:
        raise ValueError('KML contains unsupported non-polygon placemarks: ' + ', '.join(unsupported))

    latitudes = [lat for area in areas + exclusions for lat, _, _ in area.shell]
    longitudes = [lon for area in areas + exclusions for _, lon, _ in area.shell]
    projection = Projection(sum(latitudes) / len(latitudes), sum(longitudes) / len(longitudes))
    return KmlPlan(areas, exclusions, projection, points, routes)


def altitude_semantics(area, reference_altitude=None):
    """Interpret a KML area's altitude, refusing to guess between AGL and AMSL.

    Returns a dict describing the requested flight altitude and how it was
    derived. A caller that cannot supply ground elevation must keep the result
    explicitly relative rather than silently treating it as absolute.
    """
    low, high = area.altitude_range
    if abs(high - low) > 1.0:
        raise ValueError('Area "%s" spans %.1f m of altitude; a single flight '
                         'altitude cannot represent it' % (area.name, high - low))
    mode = 'amsl' if abs(high) > 0.5 else 'relative'
    if reference_altitude is not None and not finite(reference_altitude):
        raise ValueError('Reference altitude must be finite when supplied')
    return {
        'name': area.name,
        'kml_altitude_m': high,
        'mode': mode,
        'reference_altitude_m': reference_altitude,
        'flight_altitude_m': (high - reference_altitude
                              if mode == 'amsl' and reference_altitude is not None else None),
        'verified': False,
        'reason': ('KML altitude is treated as %s; ground elevation and terrain '
                   'clearance are not established by this document' % mode),
    }
