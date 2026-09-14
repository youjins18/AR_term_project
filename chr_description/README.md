# chr_description

`chr` means **cone harvest robot**: a four-tilting-rotor Palletrone with the
three-command-coordinate harvesting arm rigidly attached to its underside.

## Model composition

- `scene.xml`: visualization defaults and the complete scene entry point.
- `chr.xml`: physics options, assets, actuators and composition root.
- `palletrone.xml`: free base, four diagonal tilt joints, rotor sites, CAD visuals
  and lightweight primitive collision geometry.
- `arm.xml`: original cone harvester CAD, inertial data and joints J1--J3. The
  original fourth wrist joint is intentionally fixed because the requested CHR
  command vector contains three arm coordinates.

The Palletrone `BODY.stl` and `PROP.stl` meshes retain the original TPAM metre
scale and X-configuration transforms. All arm meshes are copied without geometric
modification from `cone_harvester_sim`. The attachment transform is the
`arm_mount` body in `chr.xml`; its provisional Z offset is -0.13 m so that the arm
sits immediately below the BODY mesh. Replace that transform after measuring the
mechanical mount.

Frames use MuJoCo conventions: right-handed world with +Z up, free-body
quaternion `(w,x,y,z)`, positions in metres, angles in radians, forces in newtons.
