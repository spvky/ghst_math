const std = @import("std");
const rl = @import("raylib");
const Vec2 = @import("math.zig").Vec2;
const CollisionData = @import("math.zig").CollisionData;
const Triangle = @import("math.zig").Triangle;
const Quad = @import("math.zig").Quad;
const sort_and_sweep = @import("broad.zig").sort_and_sweep;

comptime {
    _ = @import("math.zig");
    _ = @import("broad.zig");
}

pub fn draw_color(index: usize) rl.Color {
    switch (index % 4) {
        0 => return rl.Color.red,
        1 => return rl.Color.blue,
        2 => return rl.Color.green,
        3 => return rl.Color.yellow,
        else => return rl.Color.white,
    }
}

pub fn draw_collision_data(shapes: []CollisionData, colors: []rl.Color) void {
    for (shapes, colors) |shape, color| {
        _ = color;
        const p = shape.vertices;
        for (1..p.len) |i| {
            rl.drawLineEx(p[i - 1].toRl(), p[i].toRl(), 1, draw_color(i));
        }
        rl.drawLineEx(p[p.len - 1].toRl(), p[0].toRl(), 1, draw_color(0));
    }
}

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const allocator = arena.allocator();
    rl.setConfigFlags(.{ .window_resizable = true, .borderless_windowed_mode = true });
    rl.initWindow(1600, 900, "maths");

    const zoom = 10;

    const camera = rl.Camera2D{ .target = .{ .x = -50, .y = -50 }, .offset = .{ .x = 0, .y = 0 }, .rotation = 0, .zoom = zoom };

    const rectangle_points = [_]Vec2{
        .{ .x = -5, .y = -5 },
        .{ .x = -5, .y = 5 },
        .{ .x = 5, .y = 5 },
        .{ .x = 5, .y = -5 },
    };

    var a: Quad = .{
        .rotation = 0,
        .center = Vec2.default,
        // .center = Vec2.NEG_X.scale(20).add(Vec2.NEG_Y.scale(10)),
        .vertices = rectangle_points,
    };
    var b: Quad = .{
        .rotation = 0,
        .center = Vec2.X.scale(30),
        .vertices = rectangle_points,
        .rigidbody = .dynamic,
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

        if (rl.isKeyPressed(.r)) {
            b.rotation += 90;
        }

        const a_col = try a.collision_data(allocator);
        var b_col = try b.collision_data(allocator);
        const collision = b_col.sat_collision(a_col);
        if (collision) |mtv| {
            if (b_col.rigidbody == .dynamic) {
                std.debug.print("Collision found\nAxis: {{ {d:.2},{d:.2} }}\nAxis Magnitude: {d:.2}\n", .{ mtv.axis.x, mtv.axis.y, mtv.axis.length() });
                b_col.center.* = b_col.center.*.add(mtv.axis.scale(mtv.magnitude));
            }
            b.overlapping = true;
            // debug sleep
            std.time.sleep(std.time.ns_per_ms * 100);
        } else {
            b.overlapping = false;
        }
        const b_color: rl.Color = if (b.overlapping) rl.Color.blue else rl.Color.red;
        var shapes = [_]CollisionData{ a_col, b_col };
        var colors = [_]rl.Color{
            rl.Color.red,
            b_color,
        };
        draw_collision_data(shapes[0..], colors[0..]);
        const collision_pairs = try sort_and_sweep(allocator, shapes[0..]);
        var pairsString: [10]u8 = undefined;
        const printable = try std.fmt.bufPrintZ(&pairsString, "{} pairs", .{collision_pairs.len});
        rl.drawText(printable, 0, 40, 24, rl.Color.white);

        rl.drawCircleV(.{ .x = 0, .y = 0 }, 1, rl.Color.white);
        camera.end();
        defer rl.endDrawing();
        _ = arena.reset(.retain_capacity);
    }
}
