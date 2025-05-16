const std = @import("std");
const rl = @import("raylib");
const Vec2 = @import("math.zig").Vec2;
const cos = std.math.cos;
const sin = std.math.sin;

pub fn detect_collision(s1: []Vec2, s2: []Vec2) !bool {
    var buf: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buf);
    const allocator = fba.allocator();
    var simplex = std.ArrayList(Vec2).init(allocator);

    const avg1 = average_point(s1);
    const avg2 = average_point(s2);

    var d = avg2.sub(avg1);

    var a = support(s1, s2, d);
    try simplex.append(a);
    d = a.neg();
    while (true) {
        a = support(s1, s2, d);
        if (a.dot(d) < 0) return false;
        try simplex.append(a);
        if (handle_simplex(&simplex, &d)) {
            return true;
        }
    }
}

pub fn handle_simplex(simplex: *std.ArrayList(Vec2), d: *Vec2) bool {
    const len = simplex.items.len;

    if (len == 2) {
        return lineCase(simplex, d);
    }
    return trianlgeCase(simplex, d);
}

pub fn lineCase(simplex: *std.ArrayList(Vec2), d: *Vec2) bool {
    const len = simplex.items.len;
    const a = simplex.items[len - 1];
    const b = simplex.items[len - 2];
    const ab = b.sub(a);
    const ao = a.neg();
    const ab_perp = triple_product(ab, ao, ab);
    d.* = ab_perp;
    return false;
}

pub fn trianlgeCase(simplex: *std.ArrayList(Vec2), d: *Vec2) bool {
    const len = simplex.items.len;
    const a = simplex.items[len - 1];
    const b = simplex.items[len - 2];
    const c = simplex.items[len - 3];
    const ab = b.sub(a);
    const ac = c.sub(a);
    const ao = a.neg();

    const ab_perp = triple_product(ac, ab, ab);
    const ac_perp = triple_product(ab, ac, ac);

    if (ab_perp.dot(ao) > 0) {
        _ = simplex.swapRemove(len - 3);
        d.* = ab_perp;
        return false;
    } else if (ac_perp.dot(ao) > 0) {
        _ = simplex.swapRemove(len - 2);
        d.* = ac_perp;
        return false;
    } else {
        return true;
    }
}

pub fn find_max(verts: []Vec2, direction: Vec2) Vec2 {
    var max_index: usize = 0;
    var max_dot: f32 = 0.0;

    for (0..verts.len) |index| {
        const dp = verts[index].dot(direction);
        if (dp > max_dot) {
            max_index = index;
            max_dot = dp;
        }
    }
    return verts[max_index];
}

pub fn support(a: []Vec2, b: []Vec2, direction: Vec2) Vec2 {
    const p1 = find_max(a, direction);
    const p2 = find_max(b, direction.neg());
    return p1.sub(p2);
}

pub fn triple_product(a: Vec2, b: Vec2, c: Vec2) Vec2 {
    return b.scale(c.dot(a)).sub(a.scale(c.dot(b)));
}

pub fn average_point(vertices: []Vec2) Vec2 {
    var sum: Vec2 = .{ .x = 0, .y = 0 };
    const len: f32 = @floatFromInt(vertices.len);

    for (vertices) |vert| {
        sum = sum.add(vert);
    }
    return .{ .x = sum.x / len, .y = sum.y / len };
}

// pub const Vec2 = struct {
//     x: f32,
//     y: f32,

//     const Self = @This();

//     pub const default: Self = .{ .x = 0, .y = 0 };
//     pub const ONE: Self = .{ .x = 1, .y = 1 };
//     pub const X: Self = .{ .x = 1, .y = 0 };
//     pub const Y: Self = .{ .x = 0, .y = 1 };
//     pub const NEG_X: Self = .{ .x = -1, .y = 1 };
//     pub const NEG_Y: Self = .{ .x = 0, .y = -1 };

//     pub fn toRl(self: Self) rl.Vector2 {
//         return .{ .x = self.x, .y = self.y };
//     }

//     pub fn add(self: Self, rhs: Self) Self {
//         return .{ .x = self.x + rhs.x, .y = self.y + rhs.y };
//     }

//     pub fn sub(self: Self, rhs: Self) Self {
//         return .{ .x = self.x - rhs.x, .y = self.y - rhs.y };
//     }

//     pub fn scale(self: Self, value: f32) Self {
//         return .{ .x = self.x * value, .y = self.y * value };
//     }

//     pub fn negate(self: Self) Self {
//         return .{ .x = -self.x, .y = -self.y };
//     }

//     pub fn dot(self: Self, rhs: Self) f32 {
//         return (self.x * rhs.x) + (self.y * rhs.y);
//     }

//     pub fn perp(self: Self) Self {
//         return .{ .x = self.y, .y = -self.x };
//     }

//     pub fn rotate(self: Self, angle: f32) Self {
//         return .{ .x = self.x * cos(angle) - self.y * sin(angle), .y = self.x * sin(angle) + self.y * cos(angle) };
//     }
// };
