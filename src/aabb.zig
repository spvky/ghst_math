const std = @import("std");
const Vec2 = @import("math.zig").Vec2;

pub const AABB = struct {
    width: f32,
    height: f32,
    center: Vec2,

    const Self = @This();

    pub fn intersects(self: Self, rhs: Self) bool {
        const self_min: Vec2 = .{ .x = self.center.x - self.width / 2, .y = self.center.y - self.height / 2 };
        const rhs_min: Vec2 = .{ .x = rhs.center.x - rhs.width / 2, .y = rhs.center.y - rhs.height / 2 };
        const self_max: Vec2 = .{ .x = self.center.x + self.width / 2, .y = self.center.y + self.height / 2 };
        const rhs_max: Vec2 = .{ .x = rhs.center.x + rhs.width / 2, .y = rhs.center.y + rhs.height / 2 };

        const md_min = self_min.sub(rhs_min);
        const md_max = self_max.sub(rhs_max);

        return md_min.x <= 0 and md_max.x >= 0 and md_min.y <= 0 and md_max.y >= 0;
    }
};
