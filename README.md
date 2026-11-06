# ECHO

Environmental Change and History of Objects

An experimental system for persistent 3D object memory
and environmental change detection using RGB-D observations.

## Platform
- NVIDIA Isaac Sim 6.2
- Python 3.12
- PyTorch and CUDA
- SAM 2.1
- OpenCV and NumPy

## Completed experiments
1. RGB-D image change detection
2. Prompted instance segmentation
3. 3D object displacement estimation
4. Persistent object association baseline
5. Automatic instance-mask generation baseline

## Initial result
A controlled fixed-camera experiment measured a 15 cm
object displacement with approximately 3.84 mm
displacement-vector error.

This is a single experimental scenario, not a general
performance benchmark.

## Next steps
- Automatic object detection and segmentation
- Multi-view 3D object association
- Occlusion-aware persistent memory
- Moving-camera and mobile-robot experiments

## Assets
USD scenes reference external NVIDIA Isaac Sim assets.
Third-party models and checkpoints are not included.
