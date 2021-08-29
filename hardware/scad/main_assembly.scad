//
// Manipulator design
//
// @author Natesh Narain
//

include <NopSCADlib/core.scad>
include <BOSL/constants.scad>
use <BOSL/shapes.scad>
use <BOSL/transforms.scad>

include <openscad-servos/vitamins/servos.scad>
include <openscad-servos/vitamins/servo_joints.scad>

servo = SG90;
bind = 0.01;

holder_thickness = 2;

base_holder_spec = [
    // Servo
    servo,
    // Shell thickness
    holder_thickness,
    // Clearance
    1,
    // Circle joint height
    2.5,
    // Joint
    SG90_JOINT1
];

leg_spec = [
    // Base holder
    base_holder_spec,
    // Length
    35,
    // End thickness
    5,
    // Circle joint scale
    1.2,
    // Slot x offset
    17,
    // Slot dimensions
    [13, 0, 2]
];

foot_spec = [
    // Base holder
    base_holder_spec,
    // Length
    31,
    // End thickness
    5,
    // Circle joint scale
    1.2,
    // Slot x offset
    18,
    // Slot dimensions
    [13, 0, 2.5]
];

main_body_spec = [
    // Dimensions
    [75, 75, 7.5],
    // Thickness
    5,
    // Base holder
    base_holder_spec
];

prop_spec = [
    // main body
    main_body_spec,
    // Prop height
    50,
    // Prop interior cutout
    0.90
];

function base_holder_servo(spec) = spec[0];
function base_holder_thickness(spec) = spec[1];
function base_holder_clearance(spec) = spec[2];
function base_holder_joint(spec) = spec[4];
function base_holder_interior_dims(spec) = [
    servo_base_dim_x(base_holder_servo(spec)) + base_holder_clearance(spec),
    servo_base_dim_y(base_holder_servo(spec)) + base_holder_clearance(spec),
    servo_screw_bar_offset(base_holder_servo(spec))[2]
];
function base_holder_interior_x(spec) = base_holder_interior_dims(spec)[0];
function base_holder_interior_y(spec) = base_holder_interior_dims(spec)[1];
function base_holder_interior_z(spec) = base_holder_interior_dims(spec)[2];
function base_holder_exterior_dims(spec) = [
    base_holder_interior_x(spec) + base_holder_thickness(spec),
    base_holder_interior_y(spec) + base_holder_thickness(spec),
    base_holder_interior_z(spec) + base_holder_thickness(spec)
];
function base_holder_exterior_x(spec) = base_holder_exterior_dims(spec)[0];
function base_holder_exterior_y(spec) = base_holder_exterior_dims(spec)[1];
function base_holder_exterior_z(spec) = base_holder_exterior_dims(spec)[2];
function base_holder_screw_dims(spec) = [
    servo_screw_bar_dims_x(base_holder_servo(spec)),
    base_holder_exterior_y(spec),
    servo_screw_bar_dims_z(base_holder_servo(spec))
];
function base_holder_screw_x(spec) = base_holder_screw_dims(spec)[0];
function base_holder_screw_y(spec) = base_holder_screw_dims(spec)[1];
function base_holder_screw_z(spec) = base_holder_screw_dims(spec)[2];
function base_holder_screw_height(spec) = servo_screw_bar_offset(base_holder_servo(spec))[2] + base_holder_thickness(spec);
function base_holder_base_cable_cutout_height(spec) = base_holder_exterior_z(spec) * 1.0;
function base_holder_base_cable_cutout_width(spec) = base_holder_exterior_y(spec) * 0.45;
function base_holder_circle_joint_radius(spec) = servo_axiel_radius(base_holder_servo(spec));
function base_holder_circle_joint_height(spec) = spec[3];
function base_holder_circle_joint_x_offset(spec) = (servo_base_dim_x(base_holder_servo(spec)) / 2) - (servo_base_dim_y(base_holder_servo(spec)) / 2);

function joint_base_holder(spec) = spec[0];
function joint_length(spec) = spec[1];
// function joint_width(spec) = base_holder_exterior_y(joint_base_holder(spec));
function joint_width(spec) = base_holder_exterior_z(joint_base_holder(spec));
function joint_top_thickness(spec) = base_holder_thickness(joint_base_holder(spec)) + 0.5;
function joint_end_thickness(spec) = spec[2];
function joint_circle_joint_radius(spec) = base_holder_circle_joint_radius(joint_base_holder(spec)) * 1.1;
function joint_slot_x_offset(spec) = spec[4];
function joint_slot_dims(spec) = spec[5];

function main_body_dims(spec) = spec[0];
function main_body_thickness(spec) = spec[1];
function main_body_base_holder(spec) = spec[2];

function main_body_exterior_dims(spec) = [
    main_body_dims(spec)[0] + main_body_thickness(spec),
    main_body_dims(spec)[1] + main_body_thickness(spec),
    main_body_dims(spec)[2]
];
function main_body_exterior_x(spec) = main_body_exterior_dims(spec)[0];
function main_body_exterior_y(spec) = main_body_exterior_dims(spec)[1];
function main_body_exterior_z(spec) = main_body_exterior_dims(spec)[2];

function main_body_base_holder_transforms(spec) = [
    [
        main_body_exterior_x(spec) / 2 + base_holder_exterior_y(main_body_base_holder(spec)) / 2 - bind,
        main_body_exterior_y(spec) / 2 - base_holder_exterior_x(main_body_base_holder(spec)) / 2,
        -base_holder_exterior_z(main_body_base_holder(spec)) + main_body_exterior_z(spec),
        90
    ],
    [
        main_body_exterior_x(spec) / 2 + base_holder_exterior_y(main_body_base_holder(spec)) / 2 - bind,
        -main_body_exterior_y(spec) / 2 + base_holder_exterior_x(main_body_base_holder(spec)) / 2,
        -base_holder_exterior_z(main_body_base_holder(spec)) + main_body_exterior_z(spec),
        -90
    ],
    [
        -main_body_exterior_x(spec) / 2 - base_holder_exterior_y(main_body_base_holder(spec)) / 2 - bind,
        main_body_exterior_y(spec) / 2 - base_holder_exterior_x(main_body_base_holder(spec)) / 2,
        -base_holder_exterior_z(main_body_base_holder(spec)) + main_body_exterior_z(spec),
        90
    ],
    [
        -main_body_exterior_x(spec) / 2 - base_holder_exterior_y(main_body_base_holder(spec)) / 2 - bind,
        -main_body_exterior_y(spec) / 2 + base_holder_exterior_x(main_body_base_holder(spec)) / 2,
        -base_holder_exterior_z(main_body_base_holder(spec)) + main_body_exterior_z(spec),
        -90
    ]
];

function main_body_base_holder_transform_x(spec) = spec[0];
function main_body_base_holder_transform_y(spec) = spec[1];
function main_body_base_holder_transform_z(spec) = spec[2];
function main_body_base_holder_transform_r(spec) = spec[3];

function prop_main_body(spec) = spec[0];
function prop_height(spec) = spec[1];
function prop_interior_cutout(spec) = spec[2];

module base_holder(spec) {
    // 1. An outer cube to contain the servo
    // 2. Another cube under the screw holes
    // 3. A cube used to cutout space for the servo
    // 4. cutouts for the screw holes
    // 5. cutout for the cable
    // 6. Circle joint at bottom

    thickness = base_holder_thickness(spec);
    base_x = servo_base_dim_x(base_holder_servo(spec));
    base_y = servo_base_dim_y(base_holder_servo(spec));

    // Exterior cube
    //  - centered on the origin (x, y)
    exterior_x = base_holder_exterior_x(spec);
    exterior_y = base_holder_exterior_y(spec);
    exterior_z = base_holder_exterior_z(spec);

    ex = exterior_x / 2;
    ey = exterior_y / 2;
    ez = exterior_z;

    // Screw placement cube
    screw_x = base_holder_screw_x(spec);
    screw_y = base_holder_screw_y(spec);
    screw_z = base_holder_screw_z(spec);
    screw_h = base_holder_screw_height(spec);

    sx = screw_x / 2;
    sy = screw_y / 2;
    sz = screw_z / 2;

    // Interior cube
    interior_x = base_holder_interior_x(spec);
    interior_y = base_holder_interior_y(spec);
    interior_z = base_holder_interior_z(spec);

    ix = interior_x / 2;
    iy = interior_y / 2;

    // Screw holes
    screw_hole_radius = servo_screw_hole_radius(base_holder_servo(spec));
    screw_hole_positions = servo_screw_hole_positions(base_holder_servo(spec));
    screw_hole_cutout_height = servo_axiel_height(base_holder_servo(spec));
    screw_hole_x_offset = base_x / 2;
    screw_hole_y_offset = base_y / 2;

    // Cable cutout height
    cable_cutout_width = base_holder_base_cable_cutout_width(spec);
    cable_cutout_height = base_holder_base_cable_cutout_height(spec);

    ccw = cable_cutout_width / 2;

    // Circle Joint
    circle_joint_radius = base_holder_circle_joint_radius(spec);
    circle_joint_height = base_holder_circle_joint_height(spec);
    circle_joint_x_offset = base_holder_circle_joint_x_offset(spec);

    difference() {
        union() {
            // Exterior cube
            span_cube([-ex, ex], [-ey, ey], [0, ez]);
            // Circle joint
            translate([circle_joint_x_offset, 0, -circle_joint_height])
                cylinder(r=circle_joint_radius, h=circle_joint_height);
        }
        // Interior cutout
        span_cube([-ix, ix], [-iy, iy], [thickness, exterior_z + bind]);
        // Cable cutout
        span_cube([ix - (thickness * 2), ex + bind], [-ccw, ccw], [thickness, cable_cutout_height]);
    }
}

module joint(spec) {
    base = joint_base_holder(spec);
    servo_joint = base_holder_joint(base);

    thickness = base_holder_thickness(base);
    top_thickness = joint_top_thickness(spec);
    // thickness = joint_top_thickness(spec);
    circle_joint_height = base_holder_circle_joint_height(base);

    // exterior_y = base_holder_exterior_y(base);
    exterior_y = joint_width(spec);
    ey = exterior_y / 2;

    exterior_x = joint_length(spec);
    exterior_z = servo_axiel_height(base_holder_servo(base)) + top_thickness;

    interior_z = servo_column_height(base_holder_servo(base)) + thickness;
    cutout_y = (exterior_y + bind) / 2;

    circle_joint_radius = joint_circle_joint_radius(spec);
    circle_joint_cutout_height = servo_axiel_height(base_holder_servo(base));
    circle_joint_x_offset = base_holder_circle_joint_x_offset(base);

    ey_adjustment = 1;

    union() {
        difference() {
            union() {
                span_cube([0, exterior_x], [-ey, ey - ey_adjustment], [-circle_joint_height, exterior_z]);

                slot_x_offset = joint_slot_x_offset(spec);
                slot_dims = joint_slot_dims(spec);
                span_cube([slot_x_offset, slot_x_offset + slot_dims[0]], [-ey, ey - ey_adjustment], [exterior_z, exterior_z + slot_dims[2]]);
            }
            span_cube([-bind, exterior_x - joint_end_thickness(spec)], [-cutout_y, cutout_y], [0, interior_z]);

            // Circle joint cutout
            translate([circle_joint_x_offset, 0, -(circle_joint_height + bind)])
                cylinder(r=circle_joint_radius, h=circle_joint_cutout_height + circle_joint_height + thickness + bind * 2);

            axiel_offset = servo_axiel_offset(base_holder_servo(base));
            translate([axiel_offset[0] / 2 - servo_axiel_radius(base_holder_servo(base)), 0, interior_z]) {
                s = 1.1;
                scale([s, s, 1]) servo_joint1(servo_joint);
            }
        }
    }
}

module leg(spec) {
    base = joint_base_holder(spec);
    thickness = base_holder_thickness(base);

    exterior_x = joint_length(spec);
    exterior_z = servo_axiel_height(base_holder_servo(base)) + thickness;

    holder_x_offset = exterior_x + (base_holder_exterior_y(base) / 2) - bind;
    holder_y_offset = base_holder_exterior_z(base) / 2;

    union() {
        joint(spec);

        translate([holder_x_offset, holder_y_offset, exterior_z / 2])
            rotate([0, 90, -90])
                base_holder(base);
    }
}

module foot(spec) {
    base = joint_base_holder(spec);
    length = joint_length(spec);
    end_thickness = joint_end_thickness(spec);
    thickness = base_holder_thickness(base);

    exterior_y = joint_width(spec);
    ey = exterior_y / 2;

    circle_joint_height = base_holder_circle_joint_height(base);

    exterior_z = servo_axiel_height(base_holder_servo(base)) + thickness;

    union() {
        joint(spec);

        hull() {
            span_cube([length, length + end_thickness], [-ey, ey], [-circle_joint_height, exterior_z]);
            translate([length + end_thickness, 0, -circle_joint_height])
                cylinder(r=ey, h=circle_joint_height + exterior_z);
        }
    }
}

module main_body(spec) {
    main_body_dims = main_body_dims(spec);
    thickness = main_body_thickness(spec);

    base_holder_spec = main_body_base_holder(spec);

    // interior_x = main_body_dims[0];
    // interior_y = main_body_dims[1];
    // interior_z = main_body_dims[2];

    exterior_x = main_body_exterior_x(spec);
    exterior_y = main_body_exterior_y(spec);
    exterior_z = main_body_exterior_z(spec);

    ex = exterior_x / 2;
    ey = exterior_y / 2;
    ez = exterior_z;

    base_holder_dims = base_holder_exterior_dims(base_holder_spec);

    union() {
        span_cube([-ex, ex], [-ey, ey], [0, ez]);

        for (trans = main_body_base_holder_transforms(spec)) {
            x = main_body_base_holder_transform_x(trans);
            y = main_body_base_holder_transform_y(trans);
            z = main_body_base_holder_transform_z(trans);
            r = main_body_base_holder_transform_r(trans);

            translate([x, y, z]) {
                zrot(r) base_holder(base_holder_spec);
            }
        }
    }
}

module leg_stl() {
    stl("leg");
    leg(leg_spec);
}

module foot_stl() {
    stl("foot");
    foot(foot_spec);
}

module base_holder_stl() {
    stl("base_holder");
    base_holder(base_holder_spec);
}

module main_body_stl() {
    stl("main_body");
    main_body(main_body_spec);
}

module servo_with_joint(servo, joint) {
    axiel_offset = servo_axiel_offset(servo);

    translate([axiel_offset[0], axiel_offset[1], servo_column_height(servo)]) {
        servo_joint1(joint);
    }

    servo(servo);
}

module leg_with_foot() {
    base_dims = servo_base_dims(servo);

    translate([-base_dims[0] / 2, -base_dims[1] / 2, holder_thickness]) servo_with_joint(servo, SG90_JOINT1);
    // base_holder(base_holder_spec);
    base_holder_stl();

    thickness = base_holder_thickness(joint_base_holder(leg_spec));
    exterior_x = joint_length(leg_spec);
    exterior_z = servo_axiel_height(base_holder_servo(joint_base_holder(leg_spec))) + thickness;

    holder_x_offset = exterior_x + (base_holder_exterior_y(joint_base_holder(leg_spec)) / 2) - bind;
    holder_y_offset = base_holder_exterior_z(joint_base_holder(leg_spec)) / 2;

    leg_stl();
    translate([holder_x_offset, holder_y_offset, exterior_z / 2])
        rotate([0, 90, -90]) {
            translate([-base_dims[0] / 2, -base_dims[1] / 2, holder_thickness]) servo_with_joint(servo, SG90_JOINT1);

            foot_stl();
        }
}

module leg_mount() {
    base_size = 50;
    stand_width = 10;

    union() {
        span_cube([-base_size, base_size], [-base_size, base_size], [0, 10]);
        span_cube([-stand_width, stand_width], [base_size - stand_width, (base_size - stand_width) + stand_width], [10 - 0.01, 63]);

        translate([0, base_size - stand_width - base_holder_exterior_x(base_holder_spec) / 2 + 0.01, 45])
        zrot(-90) base_holder(base_holder_spec);
    }
}

module leg_mount_stl() {
    stl("leg_mount");
    leg_mount();
}

module prop(spec) {
    main_body = prop_main_body(spec);

    main_body_dims = main_body_dims(main_body);
    thickness = main_body_thickness(main_body);

    base_holder_spec = main_body_base_holder(main_body);

    clearance = 10;

    exterior_x = main_body_exterior_x(main_body) - clearance;
    exterior_y = main_body_exterior_y(main_body) - clearance;
    exterior_z = prop_height(spec);

    ex = exterior_x / 2;
    ey = exterior_y / 2;
    ez = exterior_z;

    cutout_percent = prop_interior_cutout(spec);
    ix = ex * cutout_percent;
    iy = ey * cutout_percent;
    iz = ez * cutout_percent;

    base_holder_dims = base_holder_exterior_dims(base_holder_spec);

    difference() {
        span_cube([-ex, ex], [-ey, ey], [0, ez]);

        cutout_z_offset = 5;
        span_cube([-ix, ix], [-iy, iy], [cutout_z_offset, ez + cutout_z_offset]);
    }
}

module prop_stl() {
    stl("prop");
    prop(prop_spec);
}

module main_assembly() {
assembly("main") {
    for (trans = main_body_base_holder_transforms(main_body_spec)) {
        x = main_body_base_holder_transform_x(trans);
        y = main_body_base_holder_transform_y(trans);
        z = main_body_base_holder_transform_z(trans);
        r = main_body_base_holder_transform_r(trans);

        translate([x, y, z]) {
            zrot(r) leg_with_foot();
        }
    }
    main_body_stl();

    zmove(-(prop_height(prop_spec) + 25))
    prop_stl();

    // leg_with_foot();

    // leg_mount_stl();
    // translate([0, 30, 45])
    //     zrot(-90) leg_with_foot();
}
}

if ($preview) {
    main_assembly();
}
