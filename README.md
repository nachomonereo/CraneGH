# CraneGH — Discrete Differential Geometry & Digital Fabrication for Grasshopper

**CraneGH** is a self-contained Discrete Differential Geometry (DDG) and Digital Fabrication toolkit for **Rhino / Grasshopper** inspired by Keenan Crane's research (Heat Method, Vector Heat Method, Globally Optimal Direction Fields, Stripe Patterns, Spin Transformations, Boundary First Flattening, Deployable Auxetics, FlipOut Geodesic Loops, Variational Surface Cuts, Repulsive Tangent-Point Surfaces, and Signed Heat Isosurfaces).

## Streamlined 5-Panel Architecture (Minimal Inputs, Maximum Control)

1. **`01. Operators & Display`**
   - **`Curvatures`**: Discrete Gaussian ($K$), Mean ($H$), Principal Curvature Direction, and Dual Area.
   - **`MCFlow`**: Implicit Mean Curvature Flow mesh smoothing.
   - **`SmoothScalar`**: Crease-preserving Cotan-Laplace scalar smoothing & gradient extraction.
   - **`WalkOnMesh`**: Intrinsic scalar gradient evaluation and Steepest Descent / Ascent / Iso-Contour / Stochastic walks on meshes.
   - **`ScalarView`**: Interactive scientific colormap evaluator (Crane Heat, CoolWarm, Viridis, Turbo, Zebra) with live 3D Min/Max callouts.

2. **`02. Geodesics & Iso-Cut`**
   - **`HeatDist`**: Geodesic distance and redistributed gradient from **any external geometry** (Points, Curves, Meshes, or Breps).
   - **`IsoCut`**: Exact sub-triangle Iso-Contour extraction and topological mesh band slicing in real physical units (e.g. `4.55`) or normalized `[0..1]`.
   - **`GeoPath`**: Exact shortest geodesic paths traced along the negative heat gradient.
   - **`FlipOut`**: Intrinsic geodesic curve & network shortening with sliding junctions.

3. **`03. Vector & Direction Fields`**
   - **`SmoothField`**: Multi-method Vector & Scalar Gradient smoothing:
     - `0`: Connection Laplacian (Vector Heat parallel transport)
     - `1`: Helmholtz-Hodge / Eikonal Gradient Redistribution (uniform non-bunching flow across the mesh)
     - `2`: Crease-Anisotropic Feature-Preserving Vector Diffusion
     - `3`: 4-RoSy Symmetric Cross Field Smoothing
   - **`OptField`**: Globally Optimal $N$-Direction Fields ($N = 1, 2, 4, 6$).
   - **`VecHeat`**: Parallel transport of tangent vectors via the Vector Heat Method.
   - **`VecFamily`**: Quantizes continuous tangent vector fields into a discrete Family of Vectors and segments faces by vector family.
   - **`Streamlines`** & **`VecView`**: Streamline integration and 3D viewport vector/cross glyph display.

4. **`04. Stripes & Conformal`**
   - **`Stripes`** & **`CrossQuads`**: Equidistant stripe patterns with topological bifurcations and orthogonal quad grids at exact physical spacing $D$.
   - **`BFF`**, **`Auxetics`**, **`SpinXForm`**: Boundary First Flattening, Deployable Auxetic kirigami mechanisms, and Quaternion Dirac Spin conformal deformations.

5. **`05. Mold, Segmentation & CAM`**
   - **`MultiMold` (`Multi-Piece Mold & Draft Cut`)**: Partitions complex 3D meshes into $K$ extractable mold pieces combining **Multiple Pull Vectors** (or automatic **Curvature-driven pull discovery**) + **Heat Geodesic Distance** (preventing disconnected islands) + **Curvature Ridge Parting Line snapping** + **Undercut detection**.
   - **`KMeansSeg`** & **`VarCuts`**: Variational K-Means++ normal/vector/crease clustering and Sharp & Crane (2018) Variational Surface Cuts.
   - **`RollSphere`**: Tangent-Point Rolling Spheres (Repulsive Surfaces 2021) for CNC Ball-Mill gouge detection (`+R`), interior medial wall thickness (`-R`), and collision-free mesh offsetting.
   - **`Skeleton`**: 3D Medial Axis Skeleton Curves and Shrinking-Ball Spine via Implicit Mean Curvature Contraction + Inscribed Spheres.
   - **`HeatIsoSurf`**: Watertight 3D Heat Offset Mesh, Smooth Boolean Union Isosurface of multiple geometries, and 3D Vector Field Isosurface via Marching Tetrahedra.
   - **`FabPaths`**: Concentric & Spiral CNC Milling / Curved-Layer 3D Printing toolpaths with Collision-Smoothed 5/6-Axis Planes.
   - **`RobotPlanes`**: Geodesic Traveling Salesperson (TSP) trajectory optimizer and $SO(3)$ Collision-Avoiding Robot Plane Stabilizer (eliminates drastic wrist turns and tilts away from mesh concavities).
