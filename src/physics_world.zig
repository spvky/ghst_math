const std = @import("std");
const math = @import("math.zig");
const Vec2 = math.Vec2;
const Shape = math.Shape;
const CollisionData = math.CollisionData;
const sort_and_sweep = @import("broad.zig").sort_and_sweep;

pub const World = struct {
    general_allocator: std.heap.GeneralPurposeAllocator,
    collision_allocator: std.heap.ArenaAllocator,
    shapes: std.ArrayListUnmanaged(Shape),
    colliders: std.ArrayListUnmanaged(CollisionData),

    const Self = @This();

    pub fn init() Self {
        const general_allocator = std.heap.GeneralPurposeAllocator(.{}).init;
        const collision_allocator = std.heap.ArenaAllocator(std.heap.page_allocator);
        const shapes = std.ArrayListUnmanaged(Shape).empty;

        return .{ .general_allocator = general_allocator, .collision_allocator = collision_allocator, .shapes = shapes };
    }

    pub fn recalculate_colliders(self: Self) void {
        _ = self.collision_allocator.reset(.retain_capacity);
    }

    pub fn deinit(self: Self) void {
        _ = self;
    }
};
