"""Planar geometry for aerial coverage plans. Invalid input never becomes a mission.

All work happens in a local east/north tangent plane around the polygon centroid,
because coverage spacing, hole containment and separation corridors are metric
questions. Latitude/longitude is a storage and KML-interchange format, not a
frame in which these checks are meaningful.
"""
import math

EARTH_RADIUS_M = 6378137.0
MIN_AREA_M2 = 1.0
MAX_VERTICES = 4096
# Published waypoints are rounded to 3 decimals, so a point that was exactly on
# the boundary can land just outside it. One millimetre absorbs that rounding
# without hiding any real navigational error.
SEGMENT_TOLERANCE_M = 1e-3


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class Projection:
    """Equirectangular tangent plane. Accurate well inside a single survey area."""

    def __init__(self, origin_latitude, origin_longitude):
        if not finite(origin_latitude) or not finite(origin_longitude):
            raise ValueError('Projection origin must be finite')
        if not -90 <= origin_latitude <= 90 or not -180 <= origin_longitude <= 180:
            raise ValueError('Projection origin outside geographic bounds')
        self.latitude = float(origin_latitude)
        self.longitude = float(origin_longitude)
        self.scale = EARTH_RADIUS_M * math.cos(math.radians(self.latitude))

    def to_enu(self, latitude, longitude):
        if not finite(latitude) or not finite(longitude):
            raise ValueError('Coordinate must be finite')
        east = math.radians(longitude - self.longitude) * self.scale
        north = math.radians(latitude - self.latitude) * EARTH_RADIUS_M
        return (east, north)

    def to_latlon(self, east, north):
        if not finite(east) or not finite(north):
            raise ValueError('Coordinate must be finite')
        longitude = self.longitude + math.degrees(east / self.scale)
        latitude = self.latitude + math.degrees(north / EARTH_RADIUS_M)
        return (latitude, longitude)


def signed_area(ring):
    """Twice-signed shoelace area in square metres. Positive means counter-clockwise."""
    total = 0.0
    for index in range(len(ring)):
        x1, y1 = ring[index]
        x2, y2 = ring[(index + 1) % len(ring)]
        total += x1 * y2 - x2 * y1
    return total / 2.0


def _orientation(a, b, c):
    value = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _on_segment(a, b, point):
    if _orientation(a, b, point) != 0:
        return False
    return (min(a[0], b[0]) <= point[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= point[1] <= max(a[1], b[1]))


def _segments_cross(a, b, c, d):
    turn1 = _orientation(a, b, c)
    turn2 = _orientation(a, b, d)
    turn3 = _orientation(c, d, a)
    turn4 = _orientation(c, d, b)
    if turn1 != turn2 and turn3 != turn4:
        return True
    return (_on_segment(a, b, c) or _on_segment(a, b, d)
            or _on_segment(c, d, a) or _on_segment(c, d, b))


def self_intersects(ring):
    """True if any two non-adjacent edges of the ring cross or overlap."""
    count = len(ring)
    if count < 4:
        return False
    for first in range(count):
        a, b = ring[first], ring[(first + 1) % count]
        for second in range(first + 1, count):
            if second == first or (second + 1) % count == first or second == (first + 1) % count:
                continue
            c, d = ring[second], ring[(second + 1) % count]
            if _segments_cross(a, b, c, d):
                return True
    return False


def remove_collinear(ring, tolerance=1e-6):
    """Drop duplicate and exactly-collinear vertices that carry no shape."""
    if len(ring) < 3:
        return list(ring)
    cleaned = []
    for point in ring:
        if not cleaned or math.dist(cleaned[-1], point) > tolerance:
            cleaned.append(point)
    while len(cleaned) > 1 and math.dist(cleaned[0], cleaned[-1]) <= tolerance:
        cleaned.pop()
    if len(cleaned) < 3:
        return cleaned
    result = []
    count = len(cleaned)
    for index in range(count):
        previous = cleaned[index - 1]
        current = cleaned[index]
        following = cleaned[(index + 1) % count]
        cross = ((current[0] - previous[0]) * (following[1] - previous[1])
                 - (current[1] - previous[1]) * (following[0] - previous[0]))
        if abs(cross) > tolerance:
            result.append(current)
    return result if len(result) >= 3 else cleaned


def point_in_ring(point, ring):
    """Even-odd ray casting on the planar ring."""
    inside = False
    count = len(ring)
    for index in range(count):
        x1, y1 = ring[index]
        x2, y2 = ring[(index + 1) % count]
        if (y1 > point[1]) != (y2 > point[1]):
            crossing = (x2 - x1) * (point[1] - y1) / (y2 - y1) + x1
            if point[0] < crossing:
                inside = not inside
    return inside


def _distance_to_segment(point, a, b):
    """Shortest distance from a point to the closed segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        return math.dist(point, a)
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_squared))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def point_on_ring(point, ring, tolerance=1e-9):
    """True when the point lies on a ring edge within ``tolerance``.

    Distance to the edge is used rather than exact collinearity because
    published coordinates are rounded to millimetres; an exact test would
    reject a lane endpoint that a rounding step moved a fraction of a
    millimetre outside the boundary.
    """
    count = len(ring)
    for index in range(count):
        if _distance_to_segment(point, ring[index], ring[(index + 1) % count]) <= tolerance:
            return True
    return False


def point_in_ring_or_boundary(point, ring, tolerance=1e-9):
    """Containment that accepts the boundary itself.

    A hole or exclusion may legitimately share an edge with the shell — an
    excluded strip along one border is a normal plan. Treating the boundary as
    "outside" would reject that while adding no safety, since the boundary has
    no interior area.
    """
    return point_in_ring(point, ring) or point_on_ring(point, ring, tolerance)


class PlannerRegion:
    """A validated shell with optional holes and exclusions, in local metres.

    A point is inside the region when it is inside the shell, outside every hole,
    and outside every exclusion area. Outside is always the safe answer.
    """

    def __init__(self, shell, holes=None, exclusions=None, name=''):
        self.name = name
        holes = holes or []
        exclusions = exclusions or []
        self.shell = self._validated_ring(shell, 'shell')
        self.holes = [self._validated_ring(ring, 'hole', inside=self.shell) for ring in holes]
        self.exclusions = [self._validated_ring(ring, 'exclusion', inside=self.shell)
                           for ring in exclusions]
        for index, hole in enumerate(self.holes):
            for other in self.holes[index + 1:]:
                if self._rings_overlap(hole, other):
                    raise ValueError('Interior holes overlap each other')
        self.area = abs(signed_area(self.shell)) - sum(
            abs(signed_area(ring)) for ring in self.holes + self.exclusions)
        if self.area < MIN_AREA_M2:
            raise ValueError('Region has no usable area after holes and exclusions')

    @staticmethod
    def _validated_ring(ring, label, inside=None):
        if not isinstance(ring, list) or not 3 <= len(ring) <= MAX_VERTICES:
            raise ValueError('%s ring must have between 3 and %d vertices' % (label, MAX_VERTICES))
        for point in ring:
            if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(
                    finite(value) for value in point):
                raise ValueError('%s ring vertices must be finite east/north pairs' % label)
        ring = remove_collinear([(float(x), float(y)) for x, y in ring])
        if len(ring) < 3:
            raise ValueError('%s ring collapses to fewer than 3 distinct vertices' % label)
        if self_intersects(ring):
            raise ValueError('%s ring is self-intersecting' % label)
        if abs(signed_area(ring)) < MIN_AREA_M2:
            raise ValueError('%s ring encloses no usable area' % label)
        if inside is not None:
            for point in ring:
                if not point_in_ring_or_boundary(point, inside):
                    raise ValueError('%s ring is not contained by the shell' % label)
        return ring

    @staticmethod
    def _rings_overlap(first, second):
        for point in first:
            if point_in_ring(point, second):
                return True
        for point in second:
            if point_in_ring(point, first):
                return True
        return False

    def contains(self, point):
        if not point_in_ring(point, self.shell):
            return False
        for ring in self.holes:
            if point_in_ring(point, ring):
                return False
        for ring in self.exclusions:
            if point_in_ring(point, ring):
                return False
        return True

    def bounds(self):
        xs = [point[0] for point in self.shell]
        ys = [point[1] for point in self.shell]
        return (min(xs), min(ys), max(xs), max(ys))

    def contains_segment(self, start, end):
        """Check the complete straight segment, including concavities and holes.

        Endpoints are compared with a millimetre tolerance because published
        waypoints are rounded to 3 decimals. That rounding can move a point
        that was exactly on the boundary a fraction of a millimetre outside,
        and an exact test would then reject a valid survey. The tolerance is
        far below any real navigational margin, so it cannot hide a genuine
        exit from the area.
        """
        if any(len(p)!=2 or not all(finite(x) for x in p) for p in (start,end)):
            return False
        tolerance=SEGMENT_TOLERANCE_M
        def allowed(p):
            return point_in_ring_or_boundary(p,self.shell,tolerance) and not any(
                point_in_ring(p,r) for r in self.holes+self.exclusions)
        if not allowed(start) or not allowed(end):return False
        dx,dy=end[0]-start[0],end[1]-start[1]
        cuts=[0.,1.]
        for ring in [self.shell]+self.holes+self.exclusions:
            for a,b in zip(ring,ring[1:]+ring[:1]):
                ex,ey=b[0]-a[0],b[1]-a[1];den=dx*ey-dy*ex
                if abs(den)<1e-12:continue
                ax,ay=a[0]-start[0],a[1]-start[1]
                t=(ax*ey-ay*ex)/den;u=(ax*dy-ay*dx)/den
                if 0<=t<=1 and 0<=u<=1:cuts.append(t)
        cuts=sorted(set(cuts))
        return all(allowed((start[0]+dx*(a+b)/2,start[1]+dy*(a+b)/2))
                   for a,b in zip(cuts,cuts[1:]))

    def contains_entry_segment(self, start, end):
        """Transit from a launch point outside the area to a waypoint inside it.

        Allowed only when the segment ends inside the region, enters the shell once and
        never crosses a hole or an exclusion (no-fly) area on the way.
        """
        if any(len(p)!=2 or not all(finite(x) for x in p) for p in (start,end)):
            return False
        if point_in_ring_or_boundary(start,self.shell,SEGMENT_TOLERANCE_M):
            return False  # starts inside: the normal contains_segment rule applies
        blocked=self.holes+self.exclusions
        if not point_in_ring_or_boundary(end,self.shell,SEGMENT_TOLERANCE_M) or any(point_in_ring(end,r) for r in blocked):
            return False
        dx,dy=end[0]-start[0],end[1]-start[1]
        cuts=[0.,1.]
        for ring in [self.shell]+blocked:
            for a,b in zip(ring,ring[1:]+ring[:1]):
                ex,ey=b[0]-a[0],b[1]-a[1];den=dx*ey-dy*ex
                if abs(den)<1e-12:continue
                ax,ay=a[0]-start[0],a[1]-start[1]
                t=(ax*ey-ay*ex)/den;u=(ax*dy-ay*dx)/den
                if 0<=t<=1 and 0<=u<=1:cuts.append(t)
        cuts=sorted(set(cuts));entered=False
        for a,b in zip(cuts,cuts[1:]):
            mid=(start[0]+dx*(a+b)/2,start[1]+dy*(a+b)/2)
            if any(point_in_ring(mid,r) for r in blocked):return False
            inside=point_in_ring_or_boundary(mid,self.shell,SEGMENT_TOLERANCE_M)
            if entered and not inside:return False  # leaves the area again
            entered=entered or inside
        return entered

    def contains_circle(self, center, radius):
        """Conservative test used before placing a corridor or a delivery point."""
        if not self.contains(center):
            return False
        steps = 16
        for index in range(steps):
            angle = 2 * math.pi * index / steps
            edge = (center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle))
            if not self.contains(edge):
                return False
        return True


def normalize_ring_winding(ring, counter_clockwise=True):
    """Return the ring wound in the requested direction."""
    area = signed_area(ring)
    if (area > 0) == counter_clockwise:
        return list(ring)
    return list(reversed(ring))
