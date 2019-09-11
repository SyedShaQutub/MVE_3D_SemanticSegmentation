# MVE_3D
A 3D reconstruction pipeline using multi view stereo algorithms

https://www.gcc.tu-darmstadt.de/home/proj/mve/

An existing pipeline is extended for 3D semantic segmentation. Semantic segmentation from 2D is extended to 3D voxels.

commands:
export PATH=/qutub/models/MVE_FSSR/mve/apps/__module__:$PATH


ESSBT module:

1.  Limiting the size of the ply file (~2%)
essbt -f0 /path/to/scene/dir/surface-L2-clean.ply /path/to/scene/dir/mesh.ply /path/to/scene/dir/

2. Process the limited size ply file
essbt -f1 /path/to/scene/dir/mesh.ply /qutub/models/MVE_FSSR/datasets/mve_rosselerstr_2/mesh.obj /path/to/scene/dir/
