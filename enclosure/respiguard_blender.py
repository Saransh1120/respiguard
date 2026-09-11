"""
RespiGuard — assembled device, built in Blender from the real dimensions.

Run it from Blender's Scripting tab: open Blender, switch to Scripting, click
"Open" and pick this file, then "Run Script". It clears the scene and builds
the whole device, so start from a fresh file or expect to lose what is there.

Everything is driven by the numbers at the top, which are the same ones the
enclosure and the bring-up guide use. One Blender unit is one millimetre.

Two views are available. EXPLODED = False gives the assembled device, which is
what you want for a photo of the product. EXPLODED = True lifts each layer
apart so the internals are visible, which is what you want for a slide.

Nothing here is a substitute for measuring the real modules. The module sizes
below are catalogue figures; the case is drawn to them, so if a real board
turns out 2 mm wider the model will be wrong in the same way the print would.
"""

import bpy
import bmesh
from mathutils import Vector

# --------------------------------------------------------------- settings
EXPLODED = False       # True lifts the layers apart
EXPLODE_GAP = 22.0     # mm between layers when exploded
BUILD_STRAP = True

# --------------------------------------------------------------- geometry
CASE_W, CASE_D, CASE_H = 80.0, 54.0, 26.0
WALL = 2.0
CORNER_R = 3.0

# Layer stack, measured down from the inside face of the lid.
LID_T = 2.0
OLED_TOP, OLED_BOT = 2.0, 7.0
MODULE_TOP, MODULE_BOT = 7.0, 13.0
BOARD_TOP, BOARD_BOT = 13.0, 14.6
BATT_TOP, BATT_BOT = 14.6, 20.6
SUB_TOP, SUB_BOT = 20.6, 22.2
BASE_TOP = 22.2

# Openings, in the same coordinates the enclosure uses.
OLED_WIN = (6.0, 8.0, 30.0, 20.0)      # x, y, w, d  — lid
LED_A = (48.0, 13.0, 1.6)              # x, y, r
LED_B = (48.0, 21.5, 1.6)
BUTTON = (65.0, 17.0, 5.6)
OPTICAL_WIN = (23.0, 19.0, 18.0, 14.0)  # base, skin side
MIC_PORT = (50.0, 26.0, 0.9)
VENT_X0, VENT_N, VENT_W, VENT_GAP = 50.0, 6, 2.0, 1.4
VENT_Z0, VENT_Z1 = 9.0, 18.0           # from the top of the case
SWITCH = (29.0, 9.5, 9.0, 6.5)         # front face: x, z, w, h
USBC = (21.0, 8.0, 12.0, 5.0)          # end wall: y, z, w, h
STRAP_SLOT = (15.0, 17.0, 24.0, 4.0)   # end wall: y, z, w, h

# Modules on the perf board, as (x, y, w, d, h, name).
MODULES = [
    (5.0, 4.0, 63.0, 25.5, 5.0, "ESP32-S3 DevKitC-1"),
    (4.0, 33.0, 26.0, 17.0, 3.5, "TP4056"),
    (32.0, 33.0, 22.0, 17.0, 4.0, "Buck-boost 3V3"),
    (56.0, 33.0, 18.0, 13.0, 3.0, "BME680"),
]
SUB_MODULES = [
    (23.0, 19.0, 18.0, 14.0, 3.0, "MAX30102"),
    (46.0, 21.0, 10.0, 9.0, 2.5, "INMP441"),
    (58.0, 13.0, 12.0, 10.0, 2.5, "BMI270"),
]

# ----------------------------------------------------------------- colours
PALETTE = {
    "case":    (0.10, 0.11, 0.12, 1),
    "lid":     (0.13, 0.14, 0.15, 1),
    "pcb":     (0.05, 0.16, 0.14, 1),
    "module":  (0.07, 0.20, 0.22, 1),
    "shield":  (0.62, 0.66, 0.68, 1),
    "battery": (0.72, 0.74, 0.76, 1),
    "glass":   (0.02, 0.03, 0.04, 1),
    "screen":  (0.10, 0.85, 0.80, 1),
    "led_ok":  (0.15, 0.80, 0.35, 1),
    "led_bad": (0.85, 0.20, 0.15, 1),
    "button":  (0.30, 0.32, 0.34, 1),
    "strap":   (0.06, 0.07, 0.08, 1),
    "gold":    (0.78, 0.64, 0.28, 1),
}


# ------------------------------------------------------------------ helpers
def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def material(name, rgba, rough=0.55, metal=0.0, emit=0.0):
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if emit:
        bsdf.inputs["Emission Color"].default_value = rgba
        bsdf.inputs["Emission Strength"].default_value = emit
    return mat


def box(name, x, y, z, w, d, h, mat_key=None, emit=0.0):
    """A box with its minimum corner at (x, y, z)."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x + w / 2, y + d / 2, z + h / 2))
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (w, d, h)
    bpy.ops.object.transform_apply(scale=True)
    if mat_key:
        ob.data.materials.append(material(mat_key, PALETTE[mat_key], emit=emit))
    return ob


def cylinder(name, x, y, z, r, h, axis="Z", mat_key=None, emit=0.0):
    rot = {"Z": (0, 0, 0), "X": (0, 1.5707963, 0), "Y": (1.5707963, 0, 0)}[axis]
    loc = {
        "Z": (x, y, z + h / 2),
        "X": (x + h / 2, y, z),
        "Y": (x, y + h / 2, z),
    }[axis]
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h, location=loc, rotation=rot,
                                        vertices=48)
    ob = bpy.context.active_object
    ob.name = name
    if mat_key:
        ob.data.materials.append(material(mat_key, PALETTE[mat_key], emit=emit))
    return ob


def cut(target, cutter):
    """Boolean-difference cutter out of target, then delete the cutter."""
    mod = target.modifiers.new("cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    mod.solver = "EXACT"
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def bevel(ob, width=CORNER_R, segments=6):
    mod = ob.modifiers.new("round", "BEVEL")
    mod.width = width
    mod.segments = segments
    mod.limit_method = "ANGLE"
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=mod.name)


def lift(ob, amount):
    if amount:
        ob.location.z += amount


# -------------------------------------------------------------------- build
def build_base():
    """Lower shell: everything from the base plate up to the parting line."""
    depth = CASE_H - (OLED_BOT)     # the lid takes the top 7 mm
    z0 = -CASE_H
    shell = box("Case base", 0, 0, z0, CASE_W, CASE_D, depth, "case")
    bevel(shell)

    # hollow it out
    inner = box("_inner", WALL, WALL, z0 + WALL,
                CASE_W - 2 * WALL, CASE_D - 2 * WALL, depth)
    cut(shell, inner)

    # optical window and microphone port, both in the base, facing the skin
    x, y, w, d = OPTICAL_WIN
    cut(shell, box("_optical", x, y, z0 - 1, w, d, WALL + 2))
    mx, my, mr = MIC_PORT
    cut(shell, cylinder("_mic", mx, my, z0 - 1, mr, WALL + 2))

    # air vents and the power switch, on the front long face
    for i in range(VENT_N):
        vx = VENT_X0 + i * (VENT_W + VENT_GAP)
        cut(shell, box("_vent", vx, -1, -VENT_Z1, VENT_W, WALL + 2,
                       VENT_Z1 - VENT_Z0))
    sx, sz, sw, sh = SWITCH
    cut(shell, box("_switch", sx, -1, -(sz + sh), sw, WALL + 2, sh))

    # USB-C window and strap slots, on the end walls
    uy, uz, uw, uh = USBC
    cut(shell, box("_usb", CASE_W - WALL - 1, uy, -(uz + uh), WALL + 2, uw, uh))
    sy, sz2, sw2, sh2 = STRAP_SLOT
    for wall_x in (-1, CASE_W - WALL - 1):
        cut(shell, box("_strap", wall_x, sy, -(sz2 + sh2), WALL + 2, sw2, sh2))

    return shell


def build_lid():
    lid = box("Case lid", 0, 0, -OLED_BOT, CASE_W, CASE_D, OLED_BOT, "lid")
    bevel(lid)

    # hollow the underside so the OLED and switches sit inside it
    cut(lid, box("_lid_inner", WALL, WALL, -OLED_BOT,
                 CASE_W - 2 * WALL, CASE_D - 2 * WALL, OLED_BOT - LID_T))

    x, y, w, d = OLED_WIN
    cut(lid, box("_oled_win", x, y, -1, w, d, LID_T + 2))
    for lx, ly, lr in (LED_A, LED_B):
        cut(lid, cylinder("_led", lx, ly, -1, lr, LID_T + 2))
    bx, by, br = BUTTON
    cut(lid, cylinder("_btn", bx, by, -1, br, LID_T + 2))
    return lid


def build_internals():
    made = []

    # the screen, sitting just under its window
    x, y, w, d = OLED_WIN
    made.append(box("OLED module", x - 1, y - 1, -OLED_BOT, w + 2, d + 2, 4.0, "pcb"))
    made.append(box("OLED glass", x + 1, y + 1, -OLED_TOP - 0.6,
                    w - 2, d - 2, 0.6, "glass"))
    made.append(box("OLED pixels", x + 3, y + 3, -OLED_TOP - 0.2,
                    w - 6, d - 8, 0.2, "screen", emit=2.5))

    # status LEDs and the function button, mounted to the lid
    made.append(cylinder("LED green", *LED_A[:2], -OLED_TOP - 2.0, LED_A[2], 2.0,
                         mat_key="led_ok", emit=3.0))
    made.append(cylinder("LED red", *LED_B[:2], -OLED_TOP - 2.0, LED_B[2], 2.0,
                         mat_key="led_bad", emit=0.4))
    made.append(cylinder("Button", BUTTON[0], BUTTON[1], -OLED_TOP - 3.0,
                         BUTTON[2] - 0.4, 3.0, mat_key="button"))

    # perf board and everything standing on it
    made.append(box("Perf board", 3, 3, -BOARD_BOT, 74, 48, 1.6, "pcb"))
    for x, y, w, d, h, name in MODULES:
        made.append(box(name, x, y, -BOARD_TOP, w, d, h, "module"))
    made.append(box("WROOM shield", 6.5, 5.0, -BOARD_TOP + 1.6, 18, 23.5, 3.1,
                    "shield"))
    made.append(cylinder("470uF cap", 72, 19, -BOARD_TOP, 4.0, 10.0,
                         mat_key="module"))

    # battery
    made.append(box("Li-Po 1000mAh", 14, 9, -BATT_BOT, 50, 34, 6.0, "battery"))

    # sensor sub-board on the base, facing the skin
    made.append(box("Sensor sub-board", 8, 10, -SUB_BOT, 64, 34, 1.6, "pcb"))
    for x, y, w, d, h, name in SUB_MODULES:
        made.append(box(name, x, y, -SUB_BOT - h, w, d, h, "module"))
    made.append(cylinder("Vibration motor", 63, 40, -SUB_BOT - 3.4, 5.0, 3.4,
                         mat_key="shield"))
    return made


def build_strap():
    sy, sz, sw, sh = STRAP_SLOT
    z = -(sz + sh / 2)
    left = box("Strap left", -70, sy, z - 1.2, 70, sw, 2.4, "strap")
    right = box("Strap right", CASE_W, sy, z - 1.2, 70, sw, 2.4, "strap")
    return [left, right]


def add_camera_and_light():
    bpy.ops.object.camera_add(location=(150, -150, 110),
                              rotation=(1.02, 0, 0.79))
    cam = bpy.context.active_object
    cam.data.lens = 85
    cam.data.clip_start = 1.0
    cam.data.clip_end = 2000.0
    bpy.context.scene.camera = cam

    bpy.ops.object.light_add(type="AREA", location=(90, -120, 160))
    key = bpy.context.active_object
    key.data.energy = 900000
    key.data.size = 180

    bpy.ops.object.light_add(type="AREA", location=(-110, -60, 90))
    fill = bpy.context.active_object
    fill.data.energy = 260000
    fill.data.size = 220

    bpy.ops.mesh.primitive_plane_add(size=1200, location=(40, 27, -CASE_H - 0.1))
    ground = bpy.context.active_object
    ground.name = "Ground"
    ground.data.materials.append(material("ground", (0.86, 0.87, 0.88, 1),
                                          rough=0.75))

    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 200
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.film_transparent = False


def main():
    clear_scene()

    base = build_base()
    lid = build_lid()
    internals = build_internals()
    strap = build_strap() if BUILD_STRAP else []

    if EXPLODED:
        # Lift each layer by how far up the stack it sits, so the order of
        # assembly is what the eye reads first.
        lift(lid, EXPLODE_GAP * 3)
        for ob in internals:
            z = ob.location.z
            if z > -BOARD_TOP:
                lift(ob, EXPLODE_GAP * 2.2)   # lid-mounted parts
            elif z > -BATT_TOP:
                lift(ob, EXPLODE_GAP * 1.4)   # board layer
            elif z > -SUB_TOP:
                lift(ob, EXPLODE_GAP * 0.8)   # battery
            else:
                lift(ob, EXPLODE_GAP * 0.3)   # sub-board
        for ob in strap:
            lift(ob, -EXPLODE_GAP * 0.2)

    add_camera_and_light()

    print("RespiGuard built: %d objects, %s view"
          % (len(bpy.data.objects), "exploded" if EXPLODED else "assembled"))


if __name__ == "__main__":
    main()
