# CraneGH — Discrete Differential Geometry, Field Design & Mesh Segmentation for Grasshopper

**CraneGH** is a self-contained, pure C# **Discrete Differential Geometry (DDG)**, **Vector/Cross Field Design**, **Surface Segmentation**, and **Digital Fabrication** plugin for **Rhino / Grasshopper**.

Built directly on top of cotangent Laplacians, Connection Laplacians, Wavefront Fast-Marching Eikonal solvers, and sparse linear algebra (with **zero external native DLL dependencies**), CraneGH brings modern geometry-processing research into Grasshopper with **minimal inputs (2–3 per component)**, **full `Graft` / multi-mesh DataTree compatibility**, and **dedicated GPU-batched visualizers**.

---

## Installation

1. Download **`CraneGH.gha`** from the repository root (`CraneGH.gha`).
2. Right-click `CraneGH.gha` $\rightarrow$ **Properties** $\rightarrow$ check **Unblock** (if downloaded from the web) $\rightarrow$ **OK**.
3. Copy `CraneGH.gha` into your Grasshopper Libraries folder:
   ```text
   %APPDATA%\Grasshopper\Libraries\CraneGH.gha
   ```
4. Restart Rhino 8 and open Grasshopper. The **`CraneGH`** tab will appear with 5 streamlined panels.

> **Tip:** Many visualizers and segmenters include a built-in **Area-Normalized Test Mesh (`Area = 1000.0`)** when the `Mesh (M)` input is left empty, so you can test and compare algorithms immediately.

---

## Plugin Architecture (5 Panels)

### `01. Visualizers` — Field, Curvature & Mesh Inspection
Pure visual and numerical analyzers (no extraneous segmentation/iso-curve polylines) with lightweight GPU-batched rendering and 3D viewport Min/Max tags.

| Component | Nickname | Main Function & Key Inputs |
| :--- | :---: | :--- |
| **Area-Normalized Test Mesh** | `TestMesh` | Generates built-in 3D benchmark meshes (`0` Saddle-Dome + Crease, `1` 3-Lobe Organic, `2` CAD Fandisk Block, `3` Torus) or normalizes any custom mesh to an exact **Surface Area `A`** (default `1000.0`) for fair comparison, outputting a surface `Test Curve` and `Test Points`. |
| **Scalar Field Visualizer** | `ScalarView` | Colors any mesh by a scalar field `S` using scientific palettes (`0` Crane Heat, `1` CoolWarm Diverging, `2` Viridis, `3` Turbo, `4` Zebra) with live 3D `Min` / `Max` tags and domain `[Min, Max]`. |
| **Scalar Gradient Visualizer** | `GradView` | Computes and displays the intrinsic surface gradient $\nabla S$ and orthogonal iso-wavefront ticks, coloring the mesh by gradient slope $\|\nabla S\|$. |
| **Vector Field Visualizer** | `VecView` | Dedicated **1-direction** vector field visualizer (`M`, `V`, `S`) using batched GPU directional segments, coloring the mesh by magnitude $\|V\|$ and outputting Divergence ($\nabla \cdot V$) and Curl. |
| **X-Cross & Div/Curl Visualizer** | `CrossView` | Dedicated **4-way X-Cross** field visualizer (`M`, `V`, `L`) showing orthogonal $U$ (Cyan) and $V$ (Coral) cross segments, Divergence, and Curl/Rotor intensity. |
| **Gaussian Curvature Visualizer** | `GaussView` | Separate visualizer for **Gaussian Curvature $K = \kappa_1 \kappa_2$** (Blue = Hyperbolic Saddle $K < 0$, White = Developable $K = 0$, Red = Elliptic Dome $K > 0$). |
| **Mean Curvature Visualizer** | `MeanView` | Separate visualizer for **Mean Curvature $H = (\kappa_1 + \kappa_2)/2$** (Blue = Concave Valley $H < 0$, White = Minimal/Flat $H = 0$, Red = Convex Ridge $H > 0$) + Mean Curvature Normals $H\mathbf{N}$. |
| **Principal Curvatures (K1 & K2) Visualizer** | `PrinCurvView` | Separate visualizer for **Max ($\kappa_1$) & Min ($\kappa_2$) Principal Curvatures**, **Curvedness**, **Koenderink Shape Index**, and combed principal directions $(\mathbf{d}_1, \mathbf{d}_2)$. |
| **Mesh Area, Quality & Topology Analyzer** | `MeshAnalyze` | Reports Surface Area, Edge Length Domain, Euler Characteristic, Naked Edges, Sharp Creases, Triangle Aspect Quality `[0..1]`, and Vertex Valence. |
| **Mesh Crease, Concavity & Anisotropy Analyzer** | `CreaseAnalyze` | Analyzes Signed Dihedral Angles (`-` Concave Valley / `+` Convex Ridge), Crease Intensity `[0..1]`, and Curvature Anisotropy $\tau \in [0..1]$. |
| **Mesh Mold Draft & Undercut Analyzer** | `DraftAnalyze` | Evaluates per-vertex Mold Draft Angles `[-90°..+90°]` and Undercut Severity `[0..1]` relative to a Pull Direction `D`. |

---

### `02. Scalar & Geodesics` — Distances, Iso-Curves & Geodesic Paths
Core scalar field solvers and exact geodesic computations on triangle meshes.

| Component | Nickname | Main Function & Key Inputs |
| :--- | :---: | :--- |
| **Heat Geodesic & Signed Distance** | `HeatDist` | Computes smooth, gradual **Geodesic Distance (`d >= 0`)**, **Signed Geodesic Distance (`SD`)**, **Smooth Gradient (`G`)**, and **Farthest Point (`FP`)** from **Points, Curves, Meshes, Breps, or default Naked Edges**. Uses global-scale Heat Diffusion + Wavefront-Ordered Planar-Triangle Eikonal + Screened Poisson integration so `d = 0.0` at the source and grows linearly to `Max` across the entire mesh (even on narrow-neck flaring surfaces). |
| **Iso-Splitting Curves** | `IsoCut` | Extracts exact sub-triangle iso-curves and slices the mesh into bands via either **(1) Subdivisions Count `N`** (e.g., `12` evenly spaced iso-curves) OR **(2) Exact Scalar Iso-Value(s) `V`** (e.g., `3.4` in real geodesic/scalar units). |
| **Geodesic Path A to B** | `GeoAB` | Classic direct **Point A to Point B** shortest geodesic curve on a mesh (requires no precomputed scalar field). |
| **Geodesic Field Path** | `GeoPath` | Traces shortest geodesic paths from target points down any Distance Field `d`. |
| **Geodesic Curve Shorten** | `FlipOut` | Shortens curves, loops, or multi-curve networks directly on a mesh into taut intrinsic geodesics (*FlipOut* algorithm). |
| **Scalar & Gradient Stripes** | `VarStripes` | Generates variable-spacing stripes driven by a Scalar Field or Scalar Gradient (high values = denser stripes, low values = sparser stripes) with **`Touch Ends (T)`** control (`False` by default). |
| **Discrete Curvatures** | `Curvatures` | Outputs raw per-vertex Gaussian Curvature `K`, Mean Curvature `H`, Principal Direction `D`, Dual Area `A`, and Welded Mesh `WM`. |
| **Smooth Scalar Field** | `SmoothScalar` | Range-preserving Cotan-Laplace diffusion smoothing for any scalar field, with optional **Crease Barrier `B`** to stop smoothing across sharp edges. |

---

### `03. Vector Fields & Stripes` — Tangent Fields, Cross Fields & Stripes
Tools to design, smooth, rotate, mix, discretize, and turn tangent vector/cross fields into equidistant stripe patterns.

| Component | Nickname | Main Function & Key Inputs |
| :--- | :---: | :--- |
| **Cross Field (U & V)** | `CrossField` | Computes a scale-invariant, **globally combed 4-RoSy Cross Field (`U` and `V = N × U`)** aligned with naked boundaries, sharp creases, principal curvatures, or optional **Guide Vectors/Curves `G`**, plus singularity points `P`. |
| **Cross Field Quad Stripes** | `CrossQuads` | Generates two orthogonal families of equidistant Quad Stripe curves (**`U Stripes`** and **`V Stripes`**) at spacing `D` aligned to a combed Cross Field, with **`Touch Ends (T)`** control (`False` by default). |
| **Vector Field Stripes** | `Stripes` | Generates stripe curves aligned to a Vector Field `V`. Pass a single number in `D` for strictly equidistant stripes, or a **Scalar Field list in `D`** to weight local stripe density (`Touch Ends T` default `False`). |
| **Vector Field & Stripes from Curves** | `CrvField` | Builds a smooth tangent Vector Field `V` (and orthogonal field `O`) parallel to input **Guide Curves `G`**, plus equidistant Stripe Curves `C`. |
| **Smooth Vector Field** | `SmoothField` | Smooths and redistributes any tangent Vector Field or Scalar Gradient (`0` Connection Laplacian / Vector Heat, `1` Helmholtz-Hodge Redistribution, `2` Crease-Anisotropic Smooth, `3` 4-RoSy). |
| **Rotate Tangent Vectors** | `RotateVec` | Rotates a tangent Vector Field `V` around each vertex normal by a uniform angle `A` (e.g., `90°`) or a per-vertex Scalar Field of angles. |
| **Scalar + Vector Field Mixer** | `FieldMix` | Combines a Scalar Field `S` and Vector Field `V` into Scalar-Weighted Vectors, Gradient-Blended Vectors, Scalar-Rotated Vectors, and Normalized `[0..1]` Weights for `Stripes` or `QuadRemesh`. |
| **Scalar-Attracted Vector Field** | `ScalarVec` | Deflects a Vector Field `V` toward high-scalar zones (`S = 1`) and away from low-scalar zones (`S = 0`), outputting the attracted vectors `VA` and flow curves `C`. |
| **Discretize Vector Field** | `VecFamily` | Quantizes a continuous Vector Field into `K` discrete canonical vector directions (or a custom Vector Family `VF`) and segments the mesh by vector family. |
| **Vector Parallel Transport** | `VecHeat` | Parallel-transports tangent vectors from seed points across curved meshes using the **Vector Heat Method**. |
| **Walk on Mesh & Gradient** | `WalkOnMesh` | Computes intrinsic scalar gradients and traces paths along Steepest Descent (`0`), Ascent (`1`), Iso-Contour (`2`), or Stochastic Walk (`3`). |

---

### `04. Segmentation & Fabrication` — Crease, Curvature, Mold & Geometry Processing
Surface segmentation methods (both user-controlled `K` and geometry-driven automatic) plus fabrication geometry utilities.

| Component | Nickname | Main Function & Key Inputs |
| :--- | :---: | :--- |
| **CAD Crease & Curvature Tensor Segmenter** | `TensorRectSeg` | Decomposes a mesh into **`K` constant-curvature and crease-bounded mold/CAD patches** (`M`, `Segments K`, `Crease Angle A`) via Principal Curvature Tensor superfacets, Region Adjacency Graph (RAG) merging, and Boundary Rectification (*Lavoué, Dupont, Baskurt LIRIS-1815*). Set `K = 0` for automatic patch count. |
| **CGAL SDF & Crease Graph-Cut Segmenter** | `SDFSeg` | Automatically segments a 3D mesh by geometry using the **Shape Diameter Function (SDF)** local thickness modes and Dihedral Crease Graph-Cut (no `K` needed). |
| **Salient Spectral & Concavity Segmenter** | `SpectralSeg` | Automatically segments organic/branching meshes via Recursive Salient Spectral Cuts and the Concavity Minima Rule (*Zhang & Liu VMV 2005 / Liu & Zhang EG 2007*). |
| **K-Means Normal & Vector Segmenter** | `KMeansSeg` | Segments a mesh into `K` contiguous pieces using K-Means clustering on Face Normals and an optional Vector Field, with taut geodesic seam boundaries. |
| **Geodesic Seam Mesh Smooth** | `GeoSeam` | Projects and smooths the boundary vertices of segmented Sub-Meshes directly onto taut **Geodesic Seams (*costuras geodésicas*)** while keeping adjacent pieces watertight. |
| **Concave / Convex Curvature Segmenter** | `CurvSeg` | Segments a mesh into Concave and Convex regions at Inflection $H = 0$ (`0`), Curvature Maxima Ridges (`1`), or Curvature Minima Valleys (`2`). |
| **Scalar & Geodesic Band Segmenter** | `ScalarSeg` | Slices a mesh into `K` clean sub-mesh bands across the level-sets of any Scalar Field (`HeatDist`, Curvature, Height, etc.). |
| **Multi-Piece Mold Segmenter** | `MultiMold` | Partitions a 3D mesh into `K` extractable Mold Pieces with Pull Vectors, Parting Lines, and Draft/Undercut analysis. |
| **Milling & Print Iso-Paths** | `FabPaths` | Generates concentric Milling or Curved-Layer Printing Iso-Paths at Stepover `D` plus Rolling-Sphere Offset Mesh & Toolpaths. |
| **Rolling-Sphere Offset & Gouge** | `RollSphere` | Computes Tangent-Point Rolling Spheres (`+R` exterior clearance / `-R` interior wall thickness) and collision-free offset meshes. |
| **3D Heat Offset & Isosurface** | `HeatIsoSurf` | Reconstructs watertight 3D Offset Meshes, Smooth Boolean Union Isosurfaces, and Subtractive Iso-Boxes from Meshes, Curves, or Vector Fields. |
| **Medial Axis Skeleton** | `Skeleton` | Extracts 3D Medial Axis Skeleton Curves, Contracted Spine Mesh, and inscribed wall-thickness radii via Laplacian contraction. |
| **Point Cloud Clean & Signed Distance** | `CleanCloud` | Denoises raw 3D Point Clouds, estimates consistent normals, evaluates Fast Pointwise Signed Distance, and reconstructs watertight meshes. |
| **Mean Curvature Mesh Smooth** | `MCFlow` | Smooths 3D mesh geometry via implicit Cotan-Laplace Mean Curvature Flow with optional boundary pinning. |

---

### `05. Remesh & Advanced DDG` — Quad Remeshing, Developability & Flows

| Component | Nickname | Main Function & Key Inputs |
| :--- | :---: | :--- |
| **Field & Distance Quad Remesh** | `QuadRemesh` | Quad-remeshes a 3D surface controlling exact **Quad Face Distance `D`** (single number = equidistant quads, or Scalar Field = weighted quad density) aligned with a Vector/Cross Field. |
| **Developable Mesh & Rulings** | `Developable` | Optimizes a mesh toward piecewise developable strips ($K \to 0$) and extracts straight ruling lines (*Stein, Grinspun, Crane SIGGRAPH 2018*). |
| **Optimal Cone Singularities & Cuts** | `OptimalCones` | Places optimal cone singularities and minimal geodesic seam cuts to minimize conformal flattening distortion (*Soliman, Sorkine-Hornung, Crane SIGGRAPH 2018*). |
| **Surface Winding Number & Trim** | `SurfWinding` | Computes exact Piecewise-Constant Winding Numbers on a mesh from open/broken curves and trims Inside/Outside regions (*Feng, Gillespie, Crane SIGGRAPH 2023*). |
| **Conformal Willmore Bending Flow** | `WillmoreFlow` | Fairs 3D surfaces by minimizing Willmore bending energy ($\int H^2 \, dA$) with optional Scalar Field stiffness weights (*Crane, Pinkall, Schröder SIGGRAPH 2013*). |

---

## Typical Workflows

1. **Geodesic Distance $\rightarrow$ Iso-Splitting Curves / Variable Stripes**
   - `Mesh` + `Curve / Points` $\rightarrow$ **`HeatDist`** (`d`, `G`) $\rightarrow$ **`ScalarView`** (inspect gradient) $\rightarrow$ **`IsoCut`** (extract `N` subdivisions or exact distance `V = 3.4`) or **`VarStripes`**.
2. **Cross Field $\rightarrow$ Orthogonal Quad Stripes / Controlled Quad Remesh**
   - `Mesh` (+ optional `Guide Curves`) $\rightarrow$ **`CrossField`** (`U`, `V`) $\rightarrow$ **`CrossQuads`** (`UC`, `VC` at spacing `D`) or **`QuadRemesh`** (with optional `FieldMix` scalar weights).
3. **CAD / Mold Crease Segmentation $\rightarrow$ Geodesic Seam Smoothing**
   - `Mesh` $\rightarrow$ **`CreaseAnalyze`** / **`GaussView`** (inspect creases & curvature) $\rightarrow$ **`TensorRectSeg`** (`Segments K`, `Crease Angle A`) $\rightarrow$ **`GeoSeam`** (project & smooth seam vertices onto taut geodesic seams).

---

## Research References

CraneGH implements algorithms inspired by the following foundational papers in Discrete Differential Geometry and Geometry Processing:
- **The Heat Method for Distance Computation** — Crane, Weischedel, Wardetzky (*ACM TOG 2013 / CACM 2017*)
- **The Vector Heat Method** — Sharp, Soliman, Crane (*ACM TOG 2019*)
- **The Signed Heat Method / Surface Winding Numbers** — Feng, Gillespie, Crane (*SIGGRAPH 2023 / 2024*)
- **Globally Optimal Direction Fields** — Knöppel, Crane, Pinkall, Schröder (*SIGGRAPH 2013*)
- **Stripe Patterns on Surfaces** — Knöppel, Crane, Pinkall, Schröder (*SIGGRAPH 2015*)
- **Mixed-Integer Quadrangulation** — Bommes, Zimmer, Kobbelt (*SIGGRAPH 2009*)
- **Curvature Tensor Based Triangle Mesh Segmentation with Boundary Rectification** — Lavoué, Dupont, Baskurt (*LIRIS-1815*)
- **Mesh Segmentation via Shape Diameter Function (CGAL)** — Shapira, Shamir, Cohen-Or (*Visual Computer 2008*)
- **Salient Spectral Mesh Segmentation** — Zhang & Liu (*VMV 2005*) / Liu & Zhang (*Eurographics 2007*)
- **You Can Find Geodesic Paths in Triangle Meshes by Just Flipping Edges (FlipOut)** — Sharp & Crane (*SIGGRAPH Asia 2020*)
- **Developability of Triangle Meshes** — Stein, Grinspun, Crane (*SIGGRAPH 2018*)
- **Robust Fairing via Conformal Curvature Flow (Willmore Flow)** — Crane, Pinkall, Schröder (*SIGGRAPH 2013*)
