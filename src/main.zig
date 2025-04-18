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

pub fn draw_collision_data(shapes: []CollisionData) void {
    for (shapes) |shape| {
        const p = shape.vertices;
        const n = shape.normals;
        // Draw edges
        for (1..p.len) |i| {
            rl.drawLineEx(p[i - 1].toRl(), p[i].toRl(), 1, draw_color(i));
        }
        rl.drawLineEx(p[p.len - 1].toRl(), p[0].toRl(), 1, draw_color(0));

        // Draw Normals
        const mid_1 = p[0].add(p[1]).scale(0.5);
        rl.drawLineEx(mid_1.toRl(), mid_1.add(n[0].scale(5)).toRl(), 1, rl.Color.purple);

        const mid_2 = p[1].add(p[2]).scale(0.5);
        rl.drawLineEx(mid_2.toRl(), mid_2.add(n[1].scale(5)).toRl(), 1, rl.Color.purple);

        const mid_3 = p[2].add(p[3]).scale(0.5);
        rl.drawLineEx(mid_3.toRl(), mid_3.add(n[2].scale(5)).toRl(), 1, rl.Color.purple);

        const mid_4 = p[3].add(p[0]).scale(0.5);
        rl.drawLineEx(mid_4.toRl(), mid_4.add(n[3].scale(5)).toRl(), 1, rl.Color.purple);
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
        .center = Vec2.X.scale(-20).add(Vec2.NEG_Y.scale(-10)),
        .vertices = rectangle_points,
    };
    var b: Quad = .{
        .rotation = 0,
        .center = Vec2.X.scale(50),
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
                const center = b_col.center.*;
                const other = a_col.center.*;

                if (other.sub(center).dot(mtv.axis) > 0) {
                    b_col.center.* = b_col.center.*.add(mtv.axis.scale(-mtv.magnitude));
                } else {
                    b_col.center.* = b_col.center.*.add(mtv.axis.scale(mtv.magnitude));
                }

                rl.drawLineEx(center.toRl(), other.toRl(), 1, rl.Color.orange);
            }
            b.overlapping = true;
            std.time.sleep(std.time.ns_per_ms * 100);
        } else {
            b.overlapping = false;
        }

        var shapes = [_]CollisionData{ a_col, b_col };
        draw_collision_data(shapes[0..]);
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
