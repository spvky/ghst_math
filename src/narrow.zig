const std = @import("std");
const math = @import("math.zig");
const Vec2 = math.Vec2;
const CollisionData = math.CollisionData;

pub fn sat_collision(self: CollisionData, rhs: CollisionData) ?Mtv {
    var overlap = std.math.floatMax(f32);
    var smallest: Vec2 = .default;
    var buffer: [1024]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    var tested_axes = std.AutoArrayHashMapUnmanaged(i64, void).empty;

    for (self.edges) |edge| {
        const axis = edge.normal;
        const x_32: i32 = @intFromFloat(axis.x);
        const y_32: i32 = @intFromFloat(axis.y);

        const axis_hash: i64 = @as(i64, @intCast(x_32)) << 32 | @as(i64, @intCast(y_32));
        if (tested_axes.get(axis_hash)) |_| {} else {
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
            tested_axes.put(allocator, axis_hash, {}) catch |err| std.debug.print("{}\n", .{err});
        }
    }

    for (rhs.edges) |edge| {
        const axis = edge.normal;
        const x_32: i32 = @intFromFloat(axis.x);
        const y_32: i32 = @intFromFloat(axis.y);

        const axis_hash: i64 = @as(i64, @intCast(x_32)) << 32 | @as(i64, @intCast(y_32));
        if (tested_axes.get(axis_hash)) |_| {} else {
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
            tested_axes.put(allocator, axis_hash, {}) catch |err| std.debug.print("{}\n", .{err});
        }
    }
    return Mtv{ .axis = smallest, .magnitude = overlap };
}

pub const Mtv = struct {
    axis: Vec2,
    magnitude: f32,
};

pub fn minkowski_sum(s1: CollisionData, s2: CollisionData) void {
    const towards_origin = s1.center.scale(-1).normalize();
    const a = support(s1.vertices, s2.vertices, towards_origin);
    if (a.dot(towards_origin) < 0) {
        //exit
    }
    const b = support(s1.vertices, s2.vertices, towards_origin.scale(-1));
    if (b.dot(towards_origin) < 0) {
        //exit
    }

    var perpendicular_towards_origin: Vec2 = .{ .x = towards_origin.y, .y = -towards_origin.x };

    if (perpendicular_towards_origin.dot(towards_origin) > 0) {
        perpendicular_towards_origin = perpendicular_towards_origin.scale(-1);
    }

    const c = support(s1.vertices, s2.vertices, perpendicular_towards_origin);
    _ = c;
}

fn simplex_sweep(simplex: *std.ArrayListUnmanaged(Vec2)) bool {
    _ = simplex;
}

fn support(a: []Vec2, b: []Vec2, direction: Vec2) Vec2 {
    const p1 = find_maximum_in_direction(a, direction);
    const p2 = find_maximum_in_direction(b, direction.scale(-1));
    return p1.sub(p2);
}

pub fn find_maximum_in_direction(verts: []Vec2, direction: Vec2) Vec2 {
    var max_index: usize = 0;
    var max_dot = 0.0;
    for (0..verts.len) |index| {
        const dot = verts[index].dot(direction);
        if (dot > max_dot) {
            max_index = index;
            max_dot = dot;
        }
    }
    return verts[max_index];
}
