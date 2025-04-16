const std = @import("std");
const rl = @import("raylib");
const Vec2 = @import("math.zig").Vec2;
const CollisionData = @import("math.zig").CollisionData;
const Triangle = @import("math.zig").Triangle;
const sort_and_sweep = @import("broad.zig").sort_and_sweep;

comptime {
    _ = @import("math.zig");
    _ = @import("broad.zig");
}

pub fn draw_collision_data(shapes: []CollisionData, colors: []rl.Color) void {
    for (shapes, colors) |shape, color| {
        const p = shape.vertices;
        for (1..p.len) |i| {
            rl.drawLineEx(p[i - 1].toRl(), p[i].toRl(), 1, color);
        }
        rl.drawLineEx(p[p.len - 1].toRl(), p[0].toRl(), 1, color);
    }
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

    var c: Triangle = .{
        .rotation = 30,
        .center = Vec2.default,
        .vertices = triangle_points,
    };

    var d: Triangle = .{
        .rotation = 30,
        .center = Vec2.default,
        .vertices = triangle_points,
    };

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        camera.begin();

        if (rl.isKeyDown(.d)) {
            b.center.x += 0.01;
        }
        if (rl.isKeyDown(.a)) {
            b.center.x -= 0.01;
        }
        if (rl.isKeyDown(.w)) {
            b.center.y -= 0.01;
        }
        if (rl.isKeyDown(.s)) {
            b.center.y += 0.01;
        }

        if (rl.isKeyDown(.r)) {
            b.rotation += 0.01;
        }

        const a_col = try a.collision_data(allocator);
        const b_col = try b.collision_data(allocator);
        const c_col = try c.collision_data(allocator);
        const d_col = try d.collision_data(allocator);
        const collision = b_col.sat_collision(a_col);
        if (collision) |_| {
            b.overlapping = true;
        } else {
            b.overlapping = false;
        }
        const b_color: rl.Color = if (b.overlapping) rl.Color.blue else rl.Color.red;
        var shapes = [_]CollisionData{ a_col, b_col, c_col, d_col };
        var colors = [_]rl.Color{
            rl.Color.red,
            b_color,
            rl.Color.red,
            rl.Color.red,
        };
        draw_collision_data(shapes[0..], colors[0..]);
        const collision_pairs = try sort_and_sweep(allocator, shapes[0..]);
        var pairsString: [10]u8 = undefined;
        const printable = try std.fmt.bufPrintZ(&pairsString, "{} pairs", .{collision_pairs.len});
        rl.drawText(printable, 0, 0, 24, rl.Color.white);

        camera.end();
        defer rl.endDrawing();
        _ = arena.reset(.retain_capacity);
    }
}
