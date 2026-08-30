/*
  RespiGuard enclosure — 80 x 54 x 26 mm, two parts, FDM.

  Everything here is driven by the variables at the top. That is not tidiness
  for its own sake: none of the breakout boards have been measured yet, because
  they have not arrived. The numbers below are catalogue dimensions, and every
  one of them should be checked with calipers on the real part before anything
  is printed at full size. Changing one variable moves every feature that
  depends on it, so correcting a measurement is a one-line edit rather than a
  redesign.

  Two decisions are already settled and are built in rather than parameterised:

  USB-C sits on an end wall. The DevKitC-1 is 63 mm long with its USB on a short
  end, and the case is only 54 mm deep — the connector physically cannot face
  the front.

  The microphone port is on the base, facing the chest. Breath sounds travel
  better through the body than through air, so a port on the top edge would hear
  the room instead of the lungs.

  Printing:
    Material   PETG. PLA softens near 60 C and this is worn against a body and
               left in bags and on bikes.
    Layer      0.2 mm
    Walls      3 perimeters — the strap slots take real load
    Infill     20% gyroid
    Supports   none needed. Both halves print open-face down and every cutout is
               on a vertical wall or in the flat base.
    Orientation base: open side up. lid: outer face down.
*/

/* [Case] */
case_l = 80;      // length, USB end to end
case_w = 54;      // depth, chest to outside
case_h = 26;      // total height of the assembled case
wall   = 2.4;     // 3 perimeters at 0.4 mm nozzle, plus a little
floor_t = 2.0;
lid_t  = 2.0;
corner_r = 3;

/* [Fit] */
// FDM prints a fraction wider than nominal. Every opening and every mating
// surface is opened up by this much, which is the difference between a lid that
// closes and one that has to be filed.
clearance = 0.3;
lip_h = 3;        // depth of the lip that locates the lid
lip_t = 1.2;

/* [Boards — CHECK THESE WITH CALIPERS BEFORE PRINTING] */
esp_l = 63.0;   esp_w = 25.5;   esp_h = 4.5;    // DevKitC-1, without headers
oled_l = 27.0;  oled_w = 27.0;                  // 0.96" SSD1306 module
oled_screen_l = 22.0; oled_screen_w = 11.0;     // active glass, not the board
max_l = 20.5;   max_w = 15.5;                   // MAX30102 breakout
bme_l = 15.0;   bme_w = 13.0;                   // BME680 breakout
imu_l = 15.0;   imu_w = 13.0;                   // BMI270 breakout
mic_l = 17.0;   mic_w = 13.0;                   // INMP441 breakout
batt_l = 55.0;  batt_w = 34.0;  batt_h = 6.0;   // 1000 mAh Li-Po

/* [Openings] */
usbc_w = 9.5;   usbc_h = 4.0;                   // plus clearance, applied below
button_d = 4.2;
led_d = 3.2;
mic_port_d = 3.0;
vent_d = 2.0;                                   // BME680 needs outside air

/* [Fasteners] */
boss_od = 5.0;
boss_id = 1.7;    // M2 self-tapping into plastic
boss_h  = 4.0;

$fn = 48;

// ---------------------------------------------------------------- helpers

module rrect(l, w, h, r) {
  hull() for (x = [r, l - r], y = [r, w - r])
    translate([x, y, 0]) cylinder(r = r, h = h);
}

/* A screw boss: post with a pilot hole down the middle. */
module boss(h = boss_h) {
  difference() {
    cylinder(d = boss_od, h = h);
    translate([0, 0, -0.1]) cylinder(d = boss_id, h = h + 0.2);
  }
}

/* Standoff that lifts a board off the floor and takes a screw. */
module standoff(h) {
  difference() {
    cylinder(d = boss_od, h = h);
    translate([0, 0, 1]) cylinder(d = boss_id, h = h);
  }
}

// ---------------------------------------------------------------- base

module base() {
  difference() {
    union() {
      // Outer shell.
      rrect(case_l, case_w, case_h - lid_t, corner_r);

      // Locating lip, standing proud of the wall top for the lid to sit over.
      translate([0, 0, case_h - lid_t])
        difference() {
          rrect(case_l - wall + lip_t, case_w - wall + lip_t, lip_h, corner_r);
          translate([wall, wall, -0.1])
            rrect(case_l - 2 * wall, case_w - 2 * wall, lip_h + 0.2, corner_r - 1);
        }
    }

    // Hollow it out.
    translate([wall, wall, floor_t])
      rrect(case_l - 2 * wall, case_w - 2 * wall, case_h, corner_r - 1);

    // --- USB-C, on the end wall ---
    // Positioned against the ESP32's own board height above the floor, so the
    // hole lines up with the connector rather than with the middle of the wall.
    translate([-1, case_w / 2 - (usbc_w + clearance) / 2, floor_t + 3.5])
      cube([wall + 2, usbc_w + clearance, usbc_h + clearance]);

    // --- microphone port, in the base, facing the chest ---
    // A ring of small holes rather than one large one: it passes sound just as
    // well, keeps fabric and dust out, and does not need a support.
    translate([case_l - 22, case_w / 2, -0.1]) {
      cylinder(d = mic_port_d, h = floor_t + 0.2);
      for (a = [0 : 60 : 300])
        rotate([0, 0, a]) translate([3.5, 0, 0])
          cylinder(d = 1.6, h = floor_t + 0.2);
    }

    // --- BME680 vents, in the base ---
    // The gas sensor reads the air it sits in. Sealed inside a box it measures
    // the box.
    translate([16, 12, -0.1])
      for (x = [0 : 4 : 12], y = [0 : 4 : 8])
        translate([x, y, 0]) cylinder(d = vent_d, h = floor_t + 0.2);

    // --- button, on the front wall ---
    translate([case_l / 2, -1, case_h / 2])
      rotate([-90, 0, 0]) cylinder(d = button_d + clearance, h = wall + 2);

    // --- LEDs, on the front wall ---
    for (x = [case_l / 2 - 10, case_l / 2 + 10])
      translate([x, -1, case_h / 2])
        rotate([-90, 0, 0]) cylinder(d = led_d + clearance, h = wall + 2);

    // --- slide switch, on the end wall opposite USB ---
    translate([case_l - wall - 1, case_w / 2 - 4, floor_t + 4])
      cube([wall + 2, 8, 5]);

    // --- strap slots ---
    // Through the side walls, wide enough for a 25 mm chest strap.
    for (x = [10, case_l - 10 - 25])
      for (y = [-1, case_w - wall - 1])
        translate([x, y, case_h - lid_t - 10])
          cube([25, wall + 2, 4]);
  }

  // --- interior standoffs ---
  // The battery lies flat in the floor; boards sit above it on these.
  esp_x = 6; esp_y = case_w - esp_w - 5;
  for (p = [[esp_x, esp_y], [esp_x + esp_l - 3, esp_y],
            [esp_x, esp_y + esp_w - 3], [esp_x + esp_l - 3, esp_y + esp_w - 3]])
    translate([p[0], p[1], floor_t]) standoff(batt_h + 2);

  // Corner bosses for the lid screws.
  for (p = [[6, 6], [case_l - 6, 6], [6, case_w - 6], [case_l - 6, case_w - 6]])
    translate([p[0], p[1], floor_t]) boss(case_h - lid_t - floor_t);
}

// ---------------------------------------------------------------- lid

module lid() {
  difference() {
    rrect(case_l, case_w, lid_t, corner_r);

    // --- OLED window ---
    // Cut to the active glass, not the module outline: the board is 27 mm
    // square but only a strip of it lights up, and a window the size of the
    // board looks like a mistake.
    translate([case_l / 2 - oled_screen_l / 2,
               case_w / 2 - oled_screen_w / 2, -0.1])
      cube([oled_screen_l + clearance, oled_screen_w + clearance, lid_t + 0.2]);

    // Screw holes, clearance not pilot — the thread bites in the base.
    for (p = [[6, 6], [case_l - 6, 6], [6, case_w - 6], [case_l - 6, case_w - 6]])
      translate([p[0], p[1], -0.1]) cylinder(d = 2.4, h = lid_t + 0.2);
  }

  // Inner rib, so the lid locates on the base lip instead of sliding around.
  translate([0, 0, -lip_h + 0.01])
    difference() {
      translate([wall + clearance, wall + clearance, 0])
        rrect(case_l - 2 * (wall + clearance), case_w - 2 * (wall + clearance),
              lip_h, corner_r - 1);
      translate([wall + lip_t + clearance, wall + lip_t + clearance, -0.1])
        rrect(case_l - 2 * (wall + lip_t + clearance),
              case_w - 2 * (wall + lip_t + clearance), lip_h + 0.2, corner_r - 1);
    }
}

// ---------------------------------------------------------------- output

// Render one at a time. Set part to "base", "lid", or "both".
part = "both";

if (part == "base") base();
else if (part == "lid") lid();
else {
  base();
  translate([0, case_w + 10, 0]) lid();
}
