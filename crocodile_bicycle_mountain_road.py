import math
from mathutils import Vector

import bpy


SCENE_NAME = "Crocodile Riding a Bicycle on a Mountain Road"


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def make_material(name, color, roughness=0.55, metallic=0.0):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    return material


def assign_material(obj, material):
    obj.data.materials.append(material)
    return obj


def add_uv_sphere(name, location, scale, material, segments=32, rings=16):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=rings, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    assign_material(obj, material)
    return obj


def add_cube(name, location, scale, material):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    assign_material(obj, material)
    return obj


def add_cylinder(name, location, radius, depth, material, vertices=48, rotation=(0, 0, 0)):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    assign_material(obj, material)
    return obj


def add_torus(name, location, major_radius, minor_radius, material, rotation=(0, 0, 0)):
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius,
        minor_radius=minor_radius,
        major_segments=96,
        minor_segments=12,
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    assign_material(obj, material)
    return obj


def look_at(obj, target):
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def add_between(name, start, end, radius, material, vertices=16):
    start_vec = Vector(start)
    end_vec = Vector(end)
    midpoint = (start_vec + end_vec) / 2
    direction = end_vec - start_vec
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=direction.length,
        location=midpoint,
    )
    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()
    assign_material(obj, material)
    return obj


def add_text(name, text, location, size, material, rotation=(math.radians(72), 0, math.radians(0))):
    bpy.ops.object.text_add(location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.data.body = text
    obj.data.align_x = "CENTER"
    obj.data.align_y = "CENTER"
    obj.data.size = size
    obj.data.extrude = 0.02
    assign_material(obj, material)
    return obj


def create_world():
    world = bpy.context.scene.world or bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.color = (0.45, 0.62, 0.86)


def create_camera_and_lighting():
    bpy.ops.object.light_add(type="SUN", location=(-7, -5, 10))
    sun = bpy.context.object
    sun.name = "Warm alpine sun"
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(42), 0, math.radians(-35))

    bpy.ops.object.light_add(type="AREA", location=(1, -5, 5))
    area = bpy.context.object
    area.name = "Soft sky fill"
    area.data.energy = 350
    area.data.size = 6

    bpy.ops.object.camera_add(location=(7.2, -12, 5.1), rotation=(math.radians(63), 0, math.radians(34)))
    camera = bpy.context.object
    camera.name = "Cinematic roadside camera"
    look_at(camera, (0.5, 0.25, 1.6))
    bpy.context.scene.camera = camera
    camera.data.lens = 36
    camera.data.dof.use_dof = True
    camera.data.dof.focus_distance = 11
    camera.data.dof.aperture_fstop = 5.6


def create_landscape(materials):
    ground = add_cube("wide alpine ground plane", (0, 0, -0.08), (18, 16, 0.08), materials["grass"])

    road = add_cube("curving mountain road base", (0, -0.15, 0.02), (2.5, 15, 0.035), materials["asphalt"])
    road.rotation_euler[2] = math.radians(-7)

    for y in range(-7, 8, 2):
        stripe = add_cube("weathered road center stripe", (0, y, 0.08), (0.08, 0.55, 0.01), materials["stripe"])
        stripe.rotation_euler[2] = road.rotation_euler[2]

    for side in (-1, 1):
        for y in range(-7, 8, 2):
            rock = add_uv_sphere(
                "roadside rounded stone",
                (side * (2.1 + 0.25 * math.sin(y)), y, 0.14),
                (0.16, 0.12, 0.1),
                materials["rock"],
                16,
                8,
            )
            rock.rotation_euler = (0, 0, y * 0.2)

    mountain_specs = [
        (-8.0, 5.5, 4.4, 5.5),
        (-4.7, 7.0, 3.2, 4.3),
        (4.8, 6.8, 3.8, 5.0),
        (8.4, 4.4, 4.8, 6.2),
        (-9.2, -2.0, 3.2, 3.6),
        (9.0, -1.2, 3.0, 3.8),
    ]
    for index, (x, y, radius, height) in enumerate(mountain_specs, start=1):
        bpy.ops.mesh.primitive_cone_add(
            vertices=5,
            radius1=radius,
            radius2=0.0,
            depth=height,
            location=(x, y, height / 2 - 0.05),
            rotation=(0, 0, math.radians(36 * index)),
        )
        mountain = bpy.context.object
        mountain.name = f"jagged distant mountain {index}"
        assign_material(mountain, materials["mountain"])

        snow = add_uv_sphere(
            f"snow cap {index}",
            (x, y, height - 0.2),
            (radius * 0.25, radius * 0.23, 0.2),
            materials["snow"],
            16,
            8,
        )
        snow.rotation_euler[2] = math.radians(18 * index)

    for i in range(24):
        x = -8 + (i % 8) * 2.3 + 0.25 * math.sin(i)
        y = -6 + (i // 8) * 5.5 + 0.45 * math.cos(i * 1.3)
        if abs(x) < 2.4 and abs(y) < 7.5:
            x += 3.8 if x > 0 else -3.8
        trunk = add_cylinder(f"pine trunk {i + 1}", (x, y, 0.45), 0.06, 0.9, materials["bark"], vertices=10)
        crown1 = add_cylinder(
            f"pine lower crown {i + 1}",
            (x, y, 1.05),
            0.46,
            0.9,
            materials["pine"],
            vertices=18,
            rotation=(0, 0, 0),
        )
        crown1.scale.z = 0.75
        crown2 = add_cylinder(f"pine upper crown {i + 1}", (x, y, 1.55), 0.31, 0.72, materials["pine"], vertices=18)
        crown2.scale.z = 0.75
        trunk.rotation_euler[2] = i * 0.2

    for i in range(7):
        cloud = add_uv_sphere(
            f"soft mountain cloud {i + 1}",
            (-6 + i * 2.0, 8.5 + 0.2 * math.sin(i), 5.2 + 0.35 * math.cos(i)),
            (0.95, 0.36, 0.24),
            materials["cloud"],
            24,
            12,
        )
        cloud.rotation_euler[2] = math.radians(i * 11)

    return ground


def create_bicycle(materials):
    wheel_positions = [(-1.15, 0, 0.72), (1.15, 0, 0.72)]
    for index, x in enumerate((-1.15, 1.15), start=1):
        add_torus(f"bicycle tire {index}", (x, 0, 0.72), 0.58, 0.045, materials["rubber"], rotation=(math.pi / 2, 0, 0))
        add_torus(f"silver wheel rim {index}", (x, 0, 0.72), 0.48, 0.016, materials["metal"], rotation=(math.pi / 2, 0, 0))
        add_cylinder(
            f"wheel hub {index}",
            (x, 0, 0.72),
            0.07,
            0.22,
            materials["metal"],
            vertices=24,
            rotation=(math.pi / 2, 0, 0),
        )
        for spoke in range(12):
            angle = spoke * math.tau / 12
            rim_point = (x + 0.48 * math.cos(angle), 0, 0.72 + 0.48 * math.sin(angle))
            add_between(f"wheel {index} spoke {spoke + 1}", (x, 0, 0.72), rim_point, 0.008, materials["metal"], vertices=8)

    rear = wheel_positions[0]
    front = wheel_positions[1]
    seat = (-0.32, 0, 1.63)
    handle = (1.55, 0, 1.55)
    crank = (0.0, 0, 0.95)
    frame_points = [
        (rear, crank),
        (crank, front),
        (rear, seat),
        (seat, handle),
        (seat, crank),
        (handle, front),
        (rear, front),
    ]
    for index, (start, end) in enumerate(frame_points, start=1):
        add_between(f"emerald bicycle frame tube {index}", start, end, 0.035, materials["bike_frame"])

    add_cube("brown leather bicycle saddle", (-0.42, 0, 1.73), (0.36, 0.18, 0.045), materials["leather"])
    add_between("left handlebar", (1.45, 0, 1.55), (1.72, -0.28, 1.62), 0.025, materials["metal"])
    add_between("right handlebar", (1.45, 0, 1.55), (1.72, 0.28, 1.62), 0.025, materials["metal"])
    add_torus("golden front headlight rim", (1.76, -0.02, 1.43), 0.12, 0.018, materials["gold"], rotation=(math.pi / 2, 0, 0))
    add_uv_sphere("glowing front headlight glass", (1.76, -0.07, 1.43), (0.09, 0.02, 0.09), materials["headlight"], 24, 12)

    for angle in (0, math.pi):
        pedal_end = (0.33 * math.cos(angle), 0.18 * math.sin(angle), 0.95 + 0.33 * math.sin(angle))
        add_between("pedal crank arm", crank, pedal_end, 0.018, materials["metal"])
        add_cube("flat pedal", pedal_end, (0.18, 0.055, 0.025), materials["rubber"])


def create_crocodile(materials):
    body = add_uv_sphere("crocodile long armored body", (0.05, 0, 1.83), (1.05, 0.33, 0.32), materials["croc_skin"], 48, 20)
    body.rotation_euler[1] = math.radians(-4)

    belly = add_uv_sphere("pale crocodile belly", (0.02, -0.01, 1.72), (0.86, 0.28, 0.12), materials["belly"], 32, 12)
    belly.rotation_euler[1] = math.radians(-4)

    head = add_uv_sphere("crocodile alert head", (1.0, 0, 2.05), (0.44, 0.29, 0.26), materials["croc_skin"], 40, 18)
    snout = add_uv_sphere("long crocodile snout", (1.45, 0, 2.0), (0.46, 0.2, 0.16), materials["croc_skin"], 32, 14)
    lower_jaw = add_uv_sphere("slightly open pale lower jaw", (1.47, 0, 1.89), (0.39, 0.17, 0.055), materials["belly"], 32, 10)

    for side in (-1, 1):
        eye = add_uv_sphere("bright crocodile eye", (1.1, side * 0.22, 2.25), (0.075, 0.055, 0.075), materials["eye"], 16, 8)
        pupil = add_uv_sphere("vertical black pupil", (1.13, side * 0.245, 2.25), (0.025, 0.012, 0.04), materials["pupil"], 12, 6)
        eye.rotation_euler[0] = math.radians(8 * side)
        pupil.rotation_euler[0] = math.radians(8 * side)

    for i in range(9):
        x = -0.75 + i * 0.22
        bpy.ops.mesh.primitive_cone_add(
            vertices=3,
            radius1=0.07,
            depth=0.18,
            location=(x, 0, 2.17 + 0.05 * math.sin(i)),
            rotation=(math.radians(90), 0, math.radians(30)),
        )
        scale = bpy.context.object
        scale.name = f"raised back scale {i + 1}"
        assign_material(scale, materials["dark_croc"])

    for i in range(14):
        x = 1.25 + 0.06 * (i % 7)
        y = (-0.12 if i < 7 else 0.12)
        z = 1.88 + 0.09 * (i % 2)
        bpy.ops.mesh.primitive_cone_add(
            vertices=8,
            radius1=0.025,
            depth=0.11,
            location=(x, y, z),
            rotation=(0, math.radians(83), 0),
        )
        tooth = bpy.context.object
        tooth.name = f"small visible tooth {i + 1}"
        assign_material(tooth, materials["tooth"])

    tail_points = [(-0.82, 0, 1.83), (-1.3, 0.02, 1.78), (-1.75, 0.04, 1.68), (-2.2, 0.0, 1.55)]
    for index, (start, end) in enumerate(zip(tail_points, tail_points[1:]), start=1):
        add_between(f"curving crocodile tail segment {index}", start, end, 0.16 - index * 0.025, materials["croc_skin"], vertices=18)

    limb_specs = [
        ((0.58, -0.22, 1.78), (1.35, -0.1, 1.55), "front left arm reaching handlebar"),
        ((0.58, 0.22, 1.78), (1.35, 0.1, 1.55), "front right arm reaching handlebar"),
        ((-0.48, -0.2, 1.66), (-0.15, -0.18, 1.02), "rear left leg pressing pedal"),
        ((-0.48, 0.2, 1.66), (0.22, 0.18, 1.26), "rear right leg lifted from pedal"),
    ]
    for start, end, name in limb_specs:
        add_between(name, start, end, 0.075, materials["croc_skin"], vertices=18)
        add_uv_sphere(f"{name} clawed foot", end, (0.12, 0.06, 0.055), materials["dark_croc"], 16, 8)

    scarf = add_torus("wind blown red adventure scarf", (0.75, 0, 2.03), 0.28, 0.035, materials["scarf"], rotation=(math.pi / 2, 0, 0))
    scarf.scale.x = 0.55
    scarf_tail = add_between("fluttering scarf tail", (0.56, 0.22, 2.02), (-0.35, 0.7, 2.13), 0.045, materials["scarf"], vertices=16)
    scarf_tail.rotation_euler[2] += math.radians(4)


def create_scene_details(materials):
    add_text(
        "painted roadside sign text",
        "鳄鱼骑行山路",
        (-3.1, -2.3, 1.35),
        0.32,
        materials["dark_croc"],
        rotation=(math.radians(78), 0, math.radians(18)),
    )
    add_between("wooden sign post", (-3.1, -2.3, 0.08), (-3.1, -2.3, 1.15), 0.05, materials["bark"])
    sign = add_cube("rustic wooden roadside sign board", (-3.1, -2.3, 1.35), (0.92, 0.05, 0.28), materials["wood"])
    sign.rotation_euler[2] = math.radians(18)

    for i in range(10):
        flower = add_uv_sphere(
            f"tiny alpine wildflower {i + 1}",
            (-4.5 + 0.45 * i, -4.0 + 0.25 * math.sin(i), 0.12),
            (0.055, 0.055, 0.035),
            materials["flower_a" if i % 2 else "flower_b"],
            12,
            6,
        )
        flower.rotation_euler[2] = i

    for i in range(5):
        dust = add_uv_sphere(
            f"soft road dust puff {i + 1}",
            (-1.4 - i * 0.27, -0.55 + 0.08 * i, 0.4 + 0.05 * i),
            (0.18 + i * 0.03, 0.09, 0.08),
            materials["dust"],
            16,
            8,
        )
        dust.rotation_euler[2] = math.radians(i * 22)


def set_render_settings():
    scene = bpy.context.scene
    scene.name = SCENE_NAME
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 96
    scene.view_settings.view_transform = "Filmic"
    scene.view_settings.look = "Medium High Contrast"
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.eevee.taa_render_samples = 64


def main():
    clear_scene()

    materials = {
        "grass": make_material("varied alpine grass", (0.18, 0.42, 0.16, 1)),
        "asphalt": make_material("dark wet mountain asphalt", (0.06, 0.065, 0.07, 1), 0.82),
        "stripe": make_material("worn warm road stripe", (0.95, 0.78, 0.28, 1), 0.7),
        "rock": make_material("cool gray roadside rock", (0.34, 0.34, 0.32, 1), 0.9),
        "mountain": make_material("blue gray rugged mountain", (0.25, 0.32, 0.38, 1), 0.85),
        "snow": make_material("clean snow caps", (0.92, 0.96, 1.0, 1), 0.45),
        "pine": make_material("deep pine needles", (0.03, 0.18, 0.08, 1), 0.75),
        "bark": make_material("rough pine bark", (0.22, 0.11, 0.045, 1), 0.88),
        "cloud": make_material("soft bright cloud", (0.9, 0.92, 0.95, 0.82), 0.35),
        "rubber": make_material("matte black rubber", (0.005, 0.005, 0.005, 1), 0.7),
        "metal": make_material("brushed silver metal", (0.72, 0.72, 0.68, 1), 0.28, 0.8),
        "bike_frame": make_material("metallic emerald bicycle frame", (0.02, 0.58, 0.42, 1), 0.35, 0.45),
        "leather": make_material("aged brown leather", (0.31, 0.13, 0.045, 1), 0.62),
        "gold": make_material("polished brass gold", (1.0, 0.63, 0.13, 1), 0.28, 0.85),
        "headlight": make_material("warm glowing headlight glass", (1.0, 0.86, 0.45, 1), 0.2),
        "croc_skin": make_material("realistic olive crocodile skin", (0.17, 0.36, 0.11, 1), 0.8),
        "dark_croc": make_material("dark crocodile scales and claws", (0.05, 0.14, 0.035, 1), 0.75),
        "belly": make_material("pale yellow green belly", (0.58, 0.66, 0.28, 1), 0.72),
        "eye": make_material("gold crocodile iris", (0.95, 0.72, 0.16, 1), 0.35),
        "pupil": make_material("glossy black pupil", (0.0, 0.0, 0.0, 1), 0.18),
        "tooth": make_material("ivory sharp teeth", (0.96, 0.9, 0.73, 1), 0.42),
        "scarf": make_material("cinematic crimson scarf", (0.78, 0.03, 0.025, 1), 0.52),
        "wood": make_material("weathered wooden sign", (0.47, 0.25, 0.1, 1), 0.82),
        "flower_a": make_material("violet alpine flower", (0.5, 0.2, 0.95, 1), 0.5),
        "flower_b": make_material("gold alpine flower", (1.0, 0.78, 0.12, 1), 0.5),
        "dust": make_material("transparent road dust", (0.62, 0.54, 0.42, 0.42), 0.95),
    }

    create_world()
    create_landscape(materials)
    create_bicycle(materials)
    create_crocodile(materials)
    create_scene_details(materials)
    create_camera_and_lighting()
    set_render_settings()

    bpy.ops.wm.save_as_mainfile(filepath="crocodile_bicycle_mountain_road.blend")


if __name__ == "__main__":
    main()
