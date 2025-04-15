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
};

pub const Projection = struct {
    min: f32,
    max: f32,

    const Self = @This();

    pub fn overlap(self: Self, rhs: Self) bool {
        return self.min <= rhs.max and rhs.min <= self.max;
    }
};

test "projection overlap" {
    const a = Projection{ .min = -1, .max = 1 };
    const b = Projection{ .min = 2, .max = 3 };
    try expect(!a.overlap(b));
}

pub fn Shape(comptime vertex_count: comptime_int) type {
    return struct {
        vertices: [vertex_count]Vec2,
        center: Vec2,
        rotation: f32,
        overlapping: bool = false,

        const Self = @This();

        pub fn verts(self: Self, allocator: std.mem.Allocator) ![]Vec2 {
            const x_axis = Vec2.X.rotate(self.rotation);
            const y_axis = Vec2.Y.rotate(self.rotation);
            const center = self.center;

            var list = std.ArrayListUnmanaged(Vec2).empty;

            for (self.vertices) |vert| {
                try list.append(allocator, center.add(x_axis.scale(vert.x).add(y_axis.scale(vert.y))));
            }

            return list.toOwnedSlice(allocator);
        }

        pub fn collision_data(self: *Self, allocator: std.mem.Allocator) !CollisionData {
            const my_verts = try self.verts(allocator);
            const my_normals = try Self.normals(allocator, my_verts);
            return .{ .vertices = my_verts, .normals = my_normals };
        }

        pub fn normals(allocator: std.mem.Allocator, points: []Vec2) ![]Vec2 {
            var list = std.ArrayListUnmanaged(Vec2).empty;
            var starting_point = points[0];
            for (points[1..]) |point| {
                try list.append(allocator, point.sub(starting_point).perp().normalize());
                starting_point = point;
            }
            try list.append(allocator, points[0].sub(starting_point).perp().normalize());
            return list.toOwnedSlice(allocator);
        }
    };
}

pub const Triangle = Shape(3);
pub const Quad = Shape(4);

pub const CollisionData = struct {
    vertices: []Vec2,
    normals: []Vec2,

    const Self = @This();

    pub fn sat_collision(self: Self, rhs: Self) bool {
        for (self.normals) |axis| {
            const p1 = self.scalar_projection(axis);
            const p2 = rhs.scalar_projection(axis);

            if (!p1.overlap(p2)) return false;
        }

        for (rhs.normals) |axis| {
            const p1 = self.scalar_projection(axis);
            const p2 = rhs.scalar_projection(axis);

            if (!p1.overlap(p2)) return false;
        }

        return true;
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

const expect = std.testing.expect;

test "length" {
    try expect(Vec2.X.length() == 1);
}

test "dot" {
    const dot_product = Vec2.X.dot(Vec2.NEG_X);
    try expect(dot_product == -1);
}
