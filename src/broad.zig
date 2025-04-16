const std = @import("std");
const math = @import("math.zig");
const Vec2 = math.Vec2;

pub fn sort_and_sweep(allocator: std.mem.Allocator, colliders: []math.CollisionData) ![]CollisionPair {
    var pairs = std.AutoArrayHashMapUnmanaged(usize, CollisionPair).empty;

    for (0..colliders.len) |i| {
        var a = colliders[i];
        const a_proj = a.scalar_projection(Vec2.X);

        for (0..colliders.len) |j| {
            if (i == j) continue;
            var b = colliders[j];
            const b_proj = b.scalar_projection(Vec2.X);

            const a_ptr = @intFromPtr(&a);
            const b_ptr = @intFromPtr(&b);
            if (a_proj.overlap(b_proj)) {
                var hash: usize = std.math.minInt(usize);
                if (a_ptr > b_ptr) {
                    hash = (a_ptr << 32) | b_ptr;
                } else {
                    hash = (b_ptr << 32) | a_ptr;
                }
                try pairs.put(allocator, hash, .{ .a = &a, .b = &b });
            }
        }
    }

    var list = std.ArrayListUnmanaged(CollisionPair).empty;

    var iterator = pairs.iterator();

    while (iterator.next()) |pair| {
        try list.append(allocator, pair.value_ptr.*);
    }
    std.debug.print("{} pairs found\n", .{list.items.len});
    return list.toOwnedSlice(allocator);
}

pub const CollisionPair = struct {
    a: *math.CollisionData,
    b: *math.CollisionData,
};
