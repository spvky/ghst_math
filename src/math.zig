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

    pub fn overlap(self: Self, rhs: Self) f32 {
        return @min(self.max, rhs.max) - @max(self.min, rhs.min);
    }
};

pub const Mtv = struct {
    axis: Vec2,
    magnitude: f32,
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

            var normal_list = std.ArrayListUnmanaged(Vec2).empty;
            var starting_point = verts[0];
            for (verts[1..]) |point| {
                // const midpoint = starting_point.add(point).scale(0.5);
                const raw_normal = point.sub(starting_point).perp().normalize();

                try normal_list.append(allocator, raw_normal);
                starting_point = point;
            }
            // const midpoint = starting_point.add(verts[0]).scale(0.5);
            const raw_normal = verts[0].sub(starting_point).perp().normalize();
            try normal_list.append(allocator, raw_normal);
            const normals = try normal_list.toOwnedSlice(allocator);

            return .{ .vertices = verts, .normals = normals, .rigidbody = self.rigidbody, .center = &self.center };
        }
    };
}

fn index_to_color(index: usize) u8 {
    return switch (index) {
        0 => return 'Y',
        1 => return 'R',
        2 => return 'B',
        3 => return 'G',
        else => 'W',
    };
}

pub const CollisionData = struct {
    center: *Vec2,
    vertices: []Vec2,
    normals: []Vec2,
    rigidbody: RigidbodyType,
    velocity: Vec2 = Vec2.default,

    const Self = @This();

    pub fn sat_collision(self: Self, rhs: Self) ?Mtv {
        var overlap = std.math.floatMax(f32);
        var smallest: Vec2 = .default;
        // var buffer: [1024]u8 = undefined;
        // var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
        // const allocator = fba.allocator();

        // var tested_axes = std.AutoArrayHashMapUnmanaged(i64, void).empty;

        for (self.normals) |axis| {
            // const x_32: i32 = @intFromFloat(axis.x);
            // const y_32: i32 = @intFromFloat(axis.y);

            // const axis_hash: i64 = @as(i64, @intCast(x_32)) << 32 | @as(i64, @intCast(y_32));
            // if (tested_axes.get(axis_hash)) |_| {
            //     std.debug.print("Axis already tested: {{ {d:.2},{d:.2} }}\nSkipping\n", .{ axis.x, axis.y });
            if (false) {} else {
                const p1 = self.scalar_projection(axis);
                const p2 = rhs.scalar_projection(axis);

                const o = p1.overlap(p2);
                if (o <= 0) {
                    return null;
                } else {
                    if (o < overlap) {
                        overlap = o;
                        smallest = axis;
                    }
                }
                // tested_axes.put(allocator, axis_hash, {}) catch |err| std.debug.print("{}\n", .{err});
            }
        }

        for (rhs.normals) |axis| {
            // const x_32: i32 = @intFromFloat(axis.x);
            // const y_32: i32 = @intFromFloat(axis.y);

            // const axis_hash: i64 = @as(i64, @intCast(x_32)) << 32 | @as(i64, @intCast(y_32));
            // if (tested_axes.get(axis_hash)) |_| {
            //     std.debug.print("Axis already tested: {{ {d:.2},{d:.2} }}\nSkipping\n", .{ axis.x, axis.y });
            if (false) {} else {
                const p1 = self.scalar_projection(axis);
                const p2 = rhs.scalar_projection(axis);

                const o = p1.overlap(p2);
                if (o <= 0) {
                    return null;
                } else {
                    if (o < overlap) {
                        overlap = o;
                        smallest = axis;
                    }
                }
                // tested_axes.put(allocator, axis_hash, {}) catch |err| std.debug.print("{}\n", .{err});
            }
        }

        std.debug.print("----------\nMtv found\naxis: {{ {d:.2}, {d:.2} }}\nmagnitude: {d:.2}\n\n", .{ smallest.x, smallest.y, overlap });
        const center = self.center.*;
        const other = rhs.center.*;

        rl.drawLineEx(center.toRl(), other.toRl(), 1, rl.Color.orange);

        // if (center.sub(other).normalize().dot(smallest) > 0) {
        //     smallest = smallest.scale(-1);
        // }
        return Mtv{ .axis = smallest, .magnitude = overlap };
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

pub const Triangle = Shape(3);
pub const Quad = Shape(4);

const expect = std.testing.expect;

test "length" {
    try expect(Vec2.X.length() == 1);
}

test "normalization" {
    const a = Vec2{ .x = 100, .y = 100 };
    const normalized_a = a.normalize();
    const normalized_length = normalized_a.length();
    std.debug.print("a: {{ {d:.2},{d:.2} }}\n", .{ normalized_a.x, normalized_a.y });
    std.debug.print("normalized_length: {d:.2}\n", .{normalized_length});
    try expect(normalized_length == 1);
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
