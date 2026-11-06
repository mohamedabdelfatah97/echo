import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetRootLayer())

BOX_PATH = "/World/RAAFIF_Office/SM_Box_45"
box = stage.GetPrimAtPath(BOX_PATH)

if not box.IsValid():
    raise RuntimeError("RAAFIF box not found!")

# Find the box's existing translation operation
xform = UsdGeom.Xformable(box)
translate_op = next(
    (op for op in xform.GetOrderedXformOps()
     if op.GetOpType() == UsdGeom.XformOp.TypeTranslate),
    None
)

if translate_op is None:
    raise RuntimeError("Box has no translation operation")

old = translate_op.Get()

# Move box 15 cm along world X
new = type(old)(
    old[0] - 0.15,
    old[1],
    old[2]
)

translate_op.Set(new)

print("\n=== RAAFIF OBJECT MOVED ===")
print("Object:", BOX_PATH)
print("Original position:", old)
print("New position:", new)
print("Movement: -0.15 m along X")
