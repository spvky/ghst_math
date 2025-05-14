const std = @import("std");
const rl = @import("raylib");
const cos = std.math.cos;
const sin = std.math.sin;

pub const Vec2 = struct {
    x: f32,
    y: f32,

    const Self = @This();

    pub const default: Self = .{ .x = 0, .y = 0 };
    pub const ONE: Self = .{ .x = 1, .y = 1 };
    pub const X: Self = .{ .x = 1, .y = 0 };
    pub const Y: Self = .{ .x = 0, .y = 1 };
    pub const NEG_X: Self = .{ .x = -1, .y = 1 };
    pub const NEG_Y: Self = .{ .x = 0, .y = -1 };

    pub fn toRl(self: Self) rl.Vector2 {
        return .{ .x = self.x, .y = self.y };
    }

    pub fn add(self: Self, rhs: Self) Self {
        return .{ .x = self.x + rhs.x, .y = self.y + rhs.y };
    }

    pub fn sub(self: Self, rhs: Self) Self {
        return .{ .x = self.x - rhs.x, .y = self.y - rhs.y };
    }

    pub fn mult(self: Self, rhs: Self) Self {
        return .{ .x = self.x * rhs.x, .y = self.y * rhs.y };
    }

    pub fn divide(self: Self, rhs: f32) Self {
        return .{ .x = self.x / rhs, .y = self.y / rhs };
    }

    pub fn scale(self: Self, value: f32) Self {
        return .{ .x = self.x * value, .y = self.y * value };
    }

    pub fn neg(self: Self) Self {
        return .{ .x = self.x * -1, .y = self.y * -1 };
    }

    pub fn squared(self: Self) Self {
        return .{ .x = self.x * self.x, .y = self.y * self.y };
    }

    fn length_squared(self: Self) f32 {
        return std.math.pow(f32, self.x, 2) + std.math.pow(f32, self.y, 2);
    }

    pub fn length(self: Self) f32 {
        return std.math.sqrt(self.length_squared());
    }

    pub fn normalize(self: Self) Self {
        return self.divide(self.length());
    }

    pub fn dot(self: Self, rhs: Self) f32 {
        return self.x * rhs.x + self.y * rhs.y;
    }

    pub fn cross(self: Self, rhs: Self) f32 {
        return (self.x * rhs.y) - (self.y * rhs.x);
    }

    pub fn triple_product(a: Self, b: Self, c: Self) Self {
        return b.scale(c.dot(a)).sub(a.scale(c.dot(b)));
    }

    pub fn perp_axis(self: Self, rhs: Self) Self {
        return .{ .x = -(rhs.y - self.y), .y = rhs.x - self.x };
    }

    pub fn perp(self: Self) Self {
        return .{ .x = -self.y, .y = self.x };
    }

    pub fn distance(self: Self, rhs: Self) f32 {
        const offset = rhs.sub(self);
        return offset.length();
    }

    pub fn vector_projection(self: Self, rhs: Self) Self {
        return rhs.scale(self.dot(rhs) / rhs.length_squared());
    }

    pub fn rotate(self: Self, angle: f32) Self {
        return .{ .x = self.x * cos(angle) - self.y * sin(angle), .y = self.x * sin(angle) + self.y * cos(angle) };
    }

    pub fn extend(self: Self, z: f32) Vec3 {
        return .{ .x = self.x, .y = self.y, .z = z };
    }
};

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    const Self = @This();

    pub fn dot(self: Self, rhs: Self) f32 {
        return self.x * rhs.x + self.y * rhs.y + self.z * rhs.z;
    }

    pub fn cross(self: Self, rhs: Self) Vec3 {
        return Vec3{
            .x = self.y * rhs.z - self.z * rhs.y,
            .y = self.z * rhs.x - self.x * rhs.z,
            .z = self.x * rhs.y - self.y * rhs.x,
        };
    }

    pub fn truncate(self: Self) Vec2 {
        return Vec2{ .x = self.x, .y = self.y };
    }
};

pub fn Simplex(comptime T: type) type {
    const max: usize = switch (T) {
        Vec2 => 3,
        Vec3 => 4,
        else => unreachable,
    };

    return struct {
        max_points: usize = max,
        points: std.ArrayListUnmanaged(T),
        vetor_type: type = T,

        const Self = @This();

        const empty: Self = .{ .points = std.ArrayListUnmanaged(T).empty };

        pub fn append(self: Self, allocator: std.mem.Allocator, point: T) !void {
            try self.points.append(allocator, point);
        }

        pub fn remove(self: Self, index: usize) T {
            return self.points.swapRemove(index);
        }

        pub fn toOwnedSlice(self: Self, allocator: std.mem.Allocator) ![]T {
            return try self.points.toOwnedSlice(allocator);
        }

        pub fn contains(self: Self, point: T) bool {
            _ = self;
            _ = point;

            return false;
        }

        pub fn getDirection(self: Self) T {
            _ = self;
            return T.empty;
        }

        pub fn getLast(self: Self) *T {
            return &self.points.items[self.points.items.len - 1];
        }

        pub fn isFull(self: Self) bool {
            self.points.items.len == self.max_points;
        }
    };
}

pub fn find_max_point_in_direction(verts: []Vec2, direction: Vec2) Vec2 {
    var max_index: usize = 0;
    var max_dot: f32 = 0.0;
    for (0..verts.len) |index| {
        const dot = verts[index].dot(direction);
        if (dot > max_dot) {
            max_index = index;
            max_dot = dot;
        }
    }
    return verts[max_index];
}

pub fn find_max_point_index_direction(verts: []Vec2, direction: Vec2) usize {
    var max_index: usize = 0;
    var max_dot: f32 = 0.0;
    for (0..verts.len) |index| {
        const dot = verts[index].dot(direction);
        if (dot > max_dot) {
            max_index = index;
            max_dot = dot;
        }
    }
    return max_index;
}

pub const Edge = struct {
    start: Vec2,
    end: Vec2,
    normal: Vec2,

    const Self = @This();

    pub fn from(a: Vec2, b: Vec2) Self {
        const normal = b.sub(a).perp().normalize();
        return .{ .start = a, .end = b, .normal = normal };
    }

    pub fn midpoint(self: Self) Vec2 {
        return self.start.add(self.end).scale(0.5);
    }

    pub fn intersect(self: Self, rhs: Self) ?Vec2 {
        return line_intersection(self.start, self.end, rhs.start, rhs.end);
    }
};

pub const Projection = struct {
    min: f32,
    max: f32,

    const Self = @This();

    pub fn overlap(self: Self, rhs: Self) f32 {
        return @min(self.max, rhs.max) - @max(self.min, rhs.min);
    }
};

const RigidbodyType = enum { static, dynamic };

pub fn Shape(comptime vertex_count: comptime_int) type {
    return struct {
        vertices: [vertex_count]Vec2,
        center: Vec2,
        rotation: f32,
        overlapping: bool = false,
        rigidbody: RigidbodyType = .static,

        const Self = @This();

        pub fn collision_data(self: *Self, allocator: std.mem.Allocator) !CollisionData {
            const x_axis = Vec2.X.rotate(self.rotation);
            const y_axis = Vec2.Y.rotate(self.rotation);
            const center = self.center;
            var vert_list = std.ArrayListUnmanaged(Vec2).empty;
            for (self.vertices) |vert| {
                try vert_list.append(allocator, center.add(x_axis.scale(vert.x).add(y_axis.scale(vert.y))));
            }

            const verts = try vert_list.toOwnedSlice(allocator);

            var edge_list = std.ArrayListUnmanaged(Edge).empty;
            var normal_list = std.ArrayListUnmanaged(Vec2).empty;
            var starting_point = verts[0];
            for (verts[1..]) |point| {
                try edge_list.append(allocator, Edge.from(starting_point, point));
                const raw_normal = point.sub(starting_point).perp().normalize();

                try normal_list.append(allocator, raw_normal);
                starting_point = point;
            }
            try edge_list.append(allocator, Edge.from(starting_point, verts[0]));
            const edges = try edge_list.toOwnedSlice(allocator);

            return .{ .vertices = verts, .edges = edges, .rigidbody = self.rigidbody, .center = &self.center };
        }
    };
}

pub fn grahms_scan(points: []Vec2) void {
    const lowest_index = find_max_point_index_direction(points, Vec2.NEG_Y);
    _ = lowest_index;
}

pub fn is_counter_clockwise(a: Vec2, b: Vec2, c: Vec2) bool {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
}

pub const CollisionData = struct {
    center: *Vec2,
    vertices: []Vec2,
    edges: []Edge,
    rigidbody: RigidbodyType,
    velocity: Vec2 = Vec2.default,

    const Self = @This();

    pub fn init(points: []Vec2, rigidbody_type: RigidbodyType) void {
        _ = points;
        _ = rigidbody_type;
    }

    pub fn scalar_projection(self: Self, axis: Vec2) Projection {
        var min = self.vertices[0].dot(axis);
        var max = min;

        for (self.vertices) |v| {
            const proj = v.dot(axis);
            min = @min(min, proj);
            max = @max(max, proj);
        }

        return .{ .min = min, .max = max };
    }
};

pub const Ray2d = struct { origin: Vec2, direction: Vec2, toi: f32 };

pub fn line_intersection(a: Vec2, b: Vec2, c: Vec2, d: Vec2) ?Vec2 {

    // Vectors representing our lines
    const r = b.sub(a);
    const s = d.sub(c);

    const determinate = r.cross(s);
    // If the cross of the 2 vectors is 0 we know they are parallel and can never intersect
    if (determinate == 0) return null;

    const t = c.sub(a).cross(s) / determinate;
    const u = a.sub(c).cross(r) / -determinate;

    // Our parameters, if both t and u are between 0-1, we know our lines are intersecting at a.add(r.scale(t)) and c.add(s.scale(u))
    if (0 <= u and u <= 1 and 0 <= t and t <= 1) {
        return a.add(r.scale(t));
    } else {
        return null;
    }
}

pub const Triangle = Shape(3);
pub const Quad = Shape(4);

// Testing
comptime {
    _ = @import("broad.zig");
    _ = @import("sat.zig");
    _ = @import("gjk.zig");
}

const expect = std.testing.expect;
const expectApprox = std.testing.expectApproxEqRel;

test "dot debug" {
    const a: Vec3 = .{ .x = 45, .y = -20, .z = 0 };
    const b: Vec3 = .{ .x = 52, .y = 31, .z = 0 };

    const c = a.cross(b);
    const d = c.truncate();

    std.debug.print("C = x: {d:.2}, y: {d:.2}, z: {d:.2}\n", .{ c.x, c.y, c.z });
    std.debug.print("D = x: {d:.2}, y: {d:.2}\n", .{ d.x, d.y });
}

test "intersection" {
    const a = Vec2.default;
    const b = Vec2.ONE.scale(2);
    const c = Vec2.Y.scale(2);
    const d = Vec2.X.scale(2);
    const e = line_intersection(a, b, c, d).?;

    try expect(e.x == 1);
}

test "length" {
    try expect(Vec2.X.length() == 1);
}

test "normalization" {
    const a = Vec2{ .x = 100, .y = 100 };
    const normalized_a = a.normalize();
    const normalized_length = normalized_a.length();
    try expectApprox(normalized_length, 1, 0.01);
}

test "dot" {
    const dot_product = Vec2.X.dot(Vec2.NEG_X);
    try expect(dot_product == -1);
}

test "projection overlap" {
    const a = Projection{ .min = -10, .max = 1 };
    const b = Projection{ .min = 0, .max = 31 };
    try expect(a.overlap(b) == 1);
}
