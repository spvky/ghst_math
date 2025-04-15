const std = @import("std");
const rl = @import("raylib");
const Vec2 = @import("math.zig").Vec2;
const CollisionData = @import("math.zig").CollisionData;
const Triangle = @import("math.zig").Triangle;

comptime {
    _ = @import("math.zig");
}

pub fn draw_collision_data(shape: CollisionData, color: rl.Color) void {
    const p = shape.vertices;
    for (1..p.len) |i| {
        rl.drawLineEx(p[i - 1].toRl(), p[i].toRl(), 1, color);
    }
    rl.drawLineEx(p[p.len - 1].toRl(), p[0].toRl(), 1, color);
}

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const allocator = arena.allocator();
    rl.setConfigFlags(.{ .window_resizable = true, .borderless_windowed_mode = true });
    rl.initWindow(1600, 900, "maths");

    const zoom = 10;

    const camera = rl.Camera2D{ .target = .{ .x = -50, .y = -50 }, .offset = .{ .x = 0, .y = 0 }, .rotation = 0, .zoom = zoom };

    const triangle_points = [_]Vec2{
        Vec2.Y.scale(10),
        .{ .x = -10, .y = -10 },
        .{ .x = 10, .y = -10 },
    };
    var a: Triangle = .{
        .rotation = 30,
        .center = Vec2.default,
        .vertices = triangle_points,
    };
    var b: Triangle = .{
        .rotation = 30,
        .center = Vec2.default,
        .vertices = triangle_points,
    };
    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        camera.begin();

        if (rl.isKeyDown(.d)) {
            b.center.x += 0.001;
        }
        if (rl.isKeyDown(.a)) {
            b.center.x -= 0.001;
        }
        if (rl.isKeyDown(.w)) {
            b.center.y -= 0.001;
        }
        if (rl.isKeyDown(.s)) {
            b.center.y += 0.001;
        }

        if (rl.isKeyDown(.r)) {
            b.rotation += 0.001;
        }

        const a_col = try a.collision_data(allocator);
        const b_col = try b.collision_data(allocator);

        b.overlapping = b_col.sat_collision(a_col);
        const b_color: rl.Color = if (b.overlapping) rl.Color.blue else rl.Color.red;
        draw_collision_data(a_col, rl.Color.red);
        draw_collision_data(b_col, b_color);

        camera.end();
        defer rl.endDrawing();
        _ = arena.reset(.retain_capacity);
    }
}
