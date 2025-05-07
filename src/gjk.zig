const std = @import("std");
const rl = @import("raylib");
const math = @import("math.zig");
const Vec2 = math.Vec2;
const CollisionData = math.CollisionData;
const find_max_point_in_direction = math.find_max_point_in_direction;

pub const SimplexResult = union { NoIntersection: void, FoundIntersection: void, StillEvolving: usize };

pub fn calculate_simplex(s1: CollisionData, s2: CollisionData) void {
    var buf: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buf);
    const allocator = fba.allocator();
    var point_list = std.ArrayListUnmanaged(Vec2).empty;
    var direction: Vec2 = s2.center - s1.center;
    var new_point = support(s1.vertices, s2.vertices, direction);
    try point_list.append(allocator, new_point);
    new_point = support(s1.vertices, s2.vertices, direction.scale(-1));
    try point_list.append(allocator, new_point);

    const b = point_list.items[1];
    const c = point_list.items[0];

    const cb = b.sub(c);
    const c0 = c.scale(-1);

    direction = Vec2.vector_tripple(cb, c0, cb);

    new_point = support(s1.vertices, s2.vertices, direction);
    try point_list.append(allocator, new_point);

    if (contains_origin(point_list.items)) {
        // Collision
    } else {}
}

pub fn evolve_simplex(points: []Vec2, direction: *Vec2) SimplexResult {
    switch (points.len) {
        1 => {
            direction.* = direction.scale(-1);
            return SimplexResult{.NoIntersection};
        },
        2 => {
            const b = points[1];
            const c = points[0];

            const cb = b.sub(c);
            const c0 = c.scale(-1);
            direction.* = Vec2.triple_product(cb, c0, cb);
            return SimplexResult{.NoIntersection};
        },
        else => {
            const a = points[2];
            const b = points[1];
            const c = points[0];

            const a0 = a.scale(-1);
            const ab = b.sub(a);
            const ac = c.sub(a);

            const ab_perp = Vec2.triple_product(ac, ab, ab);
            const ac_perp = Vec2.triple_product(ab, ac, ac);

            if (ab_perp.dot(a0) > 0) {
                direction = ab_perp;
                return SimplexResult{ .StillEvolving = 0 };
            } else if (ac_perp.dot(a0) > 0) {
                direction = ac_perp;
                return SimplexResult{ .StillEvolving = 1 };
            } else {
                return SimplexResult{.FoundIntersection};
            }
        },
    }
}

pub fn contains_origin(points: []Vec2) bool {
    const a = points[0];
    const b = points[1];
    const c = points[2];

    const a0 = a.scale(-1);
    const ab = b.sub(a);
    const ac = c.sub(a);

    const abPerp = Vec2.triple_product(ac, ab, ab);
    const acPerp = Vec2.triple_product(ab, ac, ac);

    return abPerp.dot(a0) <= 0 and acPerp.dot(a0);
}

pub fn draw_simplex(s1: CollisionData, s2: CollisionData) void {
    const towards_origin = s1.center.scale(-1).normalize();
    const a = support(s1.vertices, s2.vertices, towards_origin);
    const c = support(s1.vertices, s2.vertices, towards_origin.scale(-1));

    var perpendicular_towards_origin: Vec2 = .{ .x = towards_origin.y, .y = -towards_origin.x };

    if (perpendicular_towards_origin.dot(towards_origin) > 0) {
        perpendicular_towards_origin = perpendicular_towards_origin.scale(-1);
    }

    const b = support(s1.vertices, s2.vertices, perpendicular_towards_origin);
    const d = support(s1.vertices, s2.vertices, perpendicular_towards_origin.scale(-1));

    rl.drawLineEx(a.toRl(), b.toRl(), 1, rl.Color.white);
    rl.drawLineEx(b.toRl(), c.toRl(), 1, rl.Color.white);
    rl.drawLineEx(c.toRl(), d.toRl(), 1, rl.Color.white);
    rl.drawLineEx(d.toRl(), a.toRl(), 1, rl.Color.white);
}

fn support(a: []Vec2, b: []Vec2, direction: Vec2) ?Vec2 {
    const p1 = find_max_point_in_direction(a, direction);
    const p2 = find_max_point_in_direction(b, direction.scale(-1));
    const new_point = p1.sub(p2);
    if (direction.dot(new_point) > 0) {
        return p1.sub(p2);
    } else {
        return null;
    }
}
