const std = @import("std");
const rl = @import("raylib");
const math = @import("math.zig");
const Vec2 = math.Vec2;
const triple_product = Vec2.triple_product;
const Simplex = math.Simplex;
const CollisionData = math.CollisionData;
const find_max_point_in_direction = math.find_max_point_in_direction;

pub const SimplexResult = union { NoIntersection: void, FoundIntersection: void, StillEvolving: usize };

pub fn detect_collision(s1: CollisionData, s2: CollisionData) !bool {
    var buf: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buf);
    const allocator = fba.allocator();
    var simplex: Simplex = .empty;
    var direction: Vec2 = s2.center.sub(s1.center.*);
    try simplex.append(allocator, support(s1.vertices, s2.vertices, direction));
    direction = direction.neg();
    while (true) {
        try simplex.append(allocator, support(s1.vertices, s2.vertices, direction));
        if (simplex.getLast().dot(direction) <= 0) {
            simplex.drawSimplex();
            return false;
        } else {
            const containsOrigin = try simplex.containsOrigin(&direction);
            if (containsOrigin) {
                simplex.drawSimplex();
                return true;
            }
        }
    }
}

pub fn detect_collision_no_alloc(s1: CollisionData, s2: CollisionData) bool {
    var iter_count: u8 = 0;
    var index: u8 = 0;
    var points: [3]Vec2 = undefined;

    var direction: Vec2 = s2.center.sub(s1.center);

    points[0] = support(s1.vertices, s2.vertices, direction);
    var a = points[0];
    var ao = a.neg();

    if (points[0].dot(direction) <= 0) {
        std.debug.print("Initial Exit\n", .{});
        return false;
    }
    direction = direction.neg();
    while (true) {
        iter_count += 1;
        index += 1;
        points[index] = support(s1.vertices, s2.vertices, direction);
        a = points[index];
        if (a.dot(direction) <= 0) {
            std.debug.print("Did not pass the origin\n", .{});
            return false;
        }
        ao = points[index].neg();
        if (index < 2) {
            const b = points[0];
            const ab = b.sub(a);
            direction = triple_product(ab, ao, ab);
            if (direction.length() == 0) direction = ab.perp();
            continue;
        }
        const b = points[1];
        const c = points[0];
        const ab = b.sub(a);
        const ac = c.sub(a);
        const acperp = triple_product(ab, ac, ac);
        if (acperp.dot(ao) >= 0) {
            direction = acperp;
        } else {
            const abperp = triple_product(ac, ab, ab);
            if (abperp.dot(ao) < 0) {
                std.debug.print("Colliding\n", .{});
                return true;
            }
            points[0] = points[1];
            direction = abperp;
        }
        points[1] = points[2];
        index = index - 1;
    }
}

pub fn is_full(points: [3]?Vec2) bool {
    for (points) |point| {
        if (point == null) return false;
    }
    return true;
}

pub fn first_empty(points: [3]?Vec2) usize {
    for (0..3) |i| {
        if (points[i] == null) return i;
    }
}

pub fn last(points: [3]?Vec2) ?Vec2 {
    const last_index = first_empty(points);
    if (last_index == 0) {
        return null;
    } else {
        return points[last_index - 1];
    }
}

pub fn contains_origin(points: [3]?Vec2) bool {
    if (!is_full(points)) return false;

    const d1 = sign(points[0], points[1]);
    const d2 = sign(points[1], points[2]);
    const d3 = sign(points[2], points[0]);

    const neg_sign = d1 < 0 or d2 < 0 or d3 < 0;
    const pos_sign = d1 > 0 or d2 > 0 or d3 > 0;

    return !(neg_sign and pos_sign);
}

fn support(a: []Vec2, b: []Vec2, direction: Vec2) Vec2 {
    const p1 = find_max_point_in_direction(a, direction);
    const p2 = find_max_point_in_direction(b, direction.scale(-1));
    return p1.sub(p2);
}

fn sign(p1: Vec2, p2: Vec2) f32 {
    return (-p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (-p2.y);
}

test "undefined test" {
    const expect = std.testing.expect;
    var a = [3]?f32{ 1, 0, null };
    try expect(a[2] == null);
    a[2] = 3.3;
    try expect(a[2] == 3.3);
    a[2] = null;
    try expect(a[2] == null);
}
