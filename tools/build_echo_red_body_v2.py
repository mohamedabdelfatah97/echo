#!/usr/bin/env python3
"""Build an ECHO red-Carter body-only experimental scene (no Isaac dependency).

This creates a NEW USD stage referencing the committed navigation baseline, plus
one bare Nova Carter actor. It does not edit the validated navigation scene.
"""
from pathlib import Path
import argparse
import math
import os

HOME = Path.home()
DEFAULT_ASSET = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/6.2/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
)


def usd_asset(value: str) -> str:
    if any(c in value for c in ("@", "\n", "\r")):
        raise ValueError(f"Invalid asset path: {value!r}")
    return value.replace("\\", "/")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=2.0)
    parser.add_argument("--y", type=float, default=-3.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--asset", default=DEFAULT_ASSET)
    parser.add_argument("--baseline", type=Path, default=HOME / "echo/scenes/echo_warehouse_nav_meters.usda")
    parser.add_argument("--output", type=Path, default=HOME / "echo/scenes/echo_warehouse_dynamic_red_body_v2.usda")
    args = parser.parse_args()
    for key in ("x", "y", "z", "yaw"):
        if not math.isfinite(getattr(args, key)):
            parser.error(f"--{key} must be finite")

    baseline = args.baseline.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not baseline.is_file():
        parser.error(f"Missing baseline file: {baseline}")
    if output.exists():
        parser.error(f"Refusing to overwrite: {output}")
    if baseline == output:
        parser.error("Output cannot be the baseline")
    base_txt = baseline.read_text()
    for token in ('defaultPrim = "World"', 'metersPerUnit = 1'):
        if token not in base_txt:
            parser.error(f"Expected baseline metadata not found: {token}")

    rel_base = usd_asset(os.path.relpath(baseline, output.parent))
    robot_asset = usd_asset(args.asset)

    # Direct, purpose-restricted bindings to BOTH body meshes. 'full' is
    # important: the RTX renderer may use a purpose-restricted MDL material.
    # 'strongerThanDescendants' intentionally overrides imported GeomSubsets
    # (which otherwise may keep the original white finishes).
    binds = '''rel material:binding = </World/ECHO_Dynamics/Looks/RedPaint> (
                        bindMaterialAs = "strongerThanDescendants"
                    )
                    rel material:binding:full = </World/ECHO_Dynamics/Looks/RedPaint> (
                        bindMaterialAs = "strongerThanDescendants"
                    )
                    rel material:binding:preview = </World/ECHO_Dynamics/Looks/RedPaint> (
                        bindMaterialAs = "strongerThanDescendants"
                    )'''

    scene = f'''#usda 1.0
(
    defaultPrim = "World"
    upAxis = "Z"
    metersPerUnit = 1
    subLayers = [
        @{rel_base}@
    ]
)

# Separate experimental layer. Main navigation robot and its ROS graphs are
# inherited untouched. Do not add any ROS graph to the background actor.
over "World"
{{
    def Xform "ECHO_Dynamics"
    {{
        custom string echo:scenario = "red_body_v2"
        def Scope "Looks"
        {{
            def Material "RedPaint"
            {{
                token outputs:surface.connect = </World/ECHO_Dynamics/Looks/RedPaint/PreviewSurface.outputs:surface>
                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.77, 0.035, 0.045)
                    float inputs:roughness = 0.58
                    float inputs:metallic = 0.02
                    token outputs:surface
                }}
            }}
        }}

        def Xform "Robots"
        {{
            def Xform "Red_Carter" (
                prepend references = @{robot_asset}@
                variants = {{
                    string Physics = "No_Physics"
                    string Sensors = "None"
                }}
            )
            {{
                custom string echo:actor_id = "red_001"
                custom string echo:actor_role = "background_agent"
                double3 xformOp:translate = ({args.x:.9g}, {args.y:.9g}, {args.z:.9g})
                float3 xformOp:rotateXYZ = (0, 0, {args.yaw:.9g})
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]

                # No root material: wheels, casters and sensor hardware retain
                # NVIDIA's own bindings. The prior v1 root bind painted wheels.
                # Instancing under 'chassis_link' is disabled locally for this
                # actor so it is legal to override the referenced body meshes.
                over "chassis_link" (
                    instanceable = false
                )
                {{
                    over "visual" (
                        instanceable = false
                    )
                    {{
                        over "top_body" (
                            instanceable = false
                            prepend apiSchemas = ["MaterialBindingAPI"]
                        )
                        {{
                            {binds}
                        }}
                        over "skirt" (
                            instanceable = false
                            prepend apiSchemas = ["MaterialBindingAPI"]
                        )
                        {{
                            {binds}
                        }}
                    }}
                }}
            }}
        }}

        def Xform "Humans" {{}}
        def Xform "Forklifts" {{}}
        def Xform "MovableObjects" {{}}
        def Xform "ExperimentMarkers" {{}}
    }}
}}
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(scene)
    print("Created experimental red-body scene:", output)
    print("Baseline scene unchanged:", baseline)
    print("Red actor pose:", args.x, args.y, args.z, args.yaw)
    print("Body-only material: top_body + skirt; no root/wheel binding")
    print("UNVERIFIED: check RTX rendering and body material in Isaac Sim 6.2")


if __name__ == "__main__":
    main()
