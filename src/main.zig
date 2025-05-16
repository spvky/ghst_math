const std = @import("std");
const rl = @import("raylib");
const Vec2 = @import("math.zig").Vec2;
const CollisionData = @import("math.zig").CollisionData;
const Triangle = @import("math.zig").Triangle;
const Quad = @import("math.zig").Quad;
const sort_and_sweep = @import("broad.zig").sort_and_sweep;
const sat_collision = @import("sat.zig").sat_collision;
const detect_collision = @import("taylors_version.zig").detect_collision;

pub fn draw_collision_data(shapes: []CollisionData) !void {
    const colliding = try detect_collision(shapes[0].vertices, shapes[1].vertices);
    for (shapes) |shape| {
        for (shape.edges) |edge| {
            const color = if (colliding) rl.Color.red else rl.Color.blue;
            rl.drawLineEx(edge.start.toRl(), edge.end.toRl(), 1, color);
        }
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

        if (rl.isKeyDown(.r)) {
            b.rotation += 5 * rl.getFrameTime();
        }

        const a_col = try a.collision_data(allocator);
        const b_col = try b.collision_data(allocator);

        var shapes = [_]CollisionData{ a_col, b_col };
        try draw_collision_data(shapes[0..]);

        const Edge = @import("math.zig").Edge;
        // Test origin containment
        const origin_ray = Edge.from(Vec2.default, Vec2.X.scale(1000));

        var intersections: u8 = 0;
        for (b_col.edges) |edge| {
            const intersection_point = origin_ray.intersect(edge);
            if (intersection_point) |_| {
                intersections += 1;
            }
        }

        const collision_pairs = try sort_and_sweep(allocator, shapes[0..]);
        _ = collision_pairs;
        var pairsString: [30]u8 = undefined;
        const printable = try std.fmt.bufPrintZ(&pairsString, "{} intersections", .{intersections});

        rl.drawText(printable, 0, 40, 24, rl.Color.white);

        rl.drawCircleV(.{ .x = 0, .y = 0 }, 1, rl.Color.white);
        camera.end();
        defer rl.endDrawing();
        _ = arena.reset(.retain_capacity);
    }
}
