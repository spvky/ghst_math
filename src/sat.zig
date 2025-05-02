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
