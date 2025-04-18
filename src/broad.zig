const std = @import("std");
const math = @import("math.zig");
const Vec2 = math.Vec2;

pub fn sort_and_sweep(allocator: std.mem.Allocator, colliders: []math.CollisionData) ![]CollisionPair {
    var pairs = std.AutoArrayHashMapUnmanaged(usize, CollisionPair).empty;
    var list = std.ArrayListUnmanaged(CollisionPair).empty;

    for (0..colliders.len) |i| {
        var a = colliders[i];
        const a_proj_x = a.scalar_projection(Vec2.X);
        const a_proj_y = a.scalar_projection(Vec2.Y);

        for (0..colliders.len) |j| {
            if (i == j) continue;
            var b = colliders[j];
            const b_proj_x = b.scalar_projection(Vec2.X);
            const b_proj_y = b.scalar_projection(Vec2.Y);

            if (a_proj_x.overlap(b_proj_x) > 0 and a_proj_y.overlap(b_proj_y) > 0) {
                var hash: usize = std.math.minInt(usize);

                if (i > j) {
                    hash = (i << 32) | j;
                } else {
                    hash = (j << 32) | i;
                }

                if (pairs.get(hash)) |_| {} else {
                    try list.append(allocator, .{ .a = &a, .b = &b });
                }
                try pairs.put(allocator, hash, .{ .a = &a, .b = &b });
            }
        }
    }

    return list.toOwnedSlice(allocator);
}

pub const CollisionPair = struct {
    a: *math.CollisionData,
    b: *math.CollisionData,
};
