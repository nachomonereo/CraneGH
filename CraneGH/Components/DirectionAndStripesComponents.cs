using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_OptimalDirectionField : GH_Component
    {
        public GH_OptimalDirectionField()
          : base("Optimal Direction Field", "OptField",
              "Computes Globally Optimal N-Direction Fields (1-Vector, 2-Line, 4-Cross, 6-Hex) via Keenan Crane's complex eigenvalue formulation (ACM TOG 2013).",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("OptField");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Symmetry", "N", "Symmetry degree N (1=Vector, 2=Line, 4=Cross, 6=Hexagonal)", GH_ParamAccess.item, 4);
            pManager.AddNumberParameter("Curvature Align", "A", "Alignment weight to principal curvature directions (0 = Globally Smoothest)", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Primary Field", "V", "Primary unit tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Orthogonal Field", "W", "90-degree rotated unit tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "S", "Detected singularity face centers", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            int nSym = 4;
            double align = 0.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref nSym);
            DA.GetData(2, ref align);

            var ddg = new DdgMesh(m);
            Vector3d[] vField = DdgAlgorithms.ComputeOptimalDirectionField(ddg, nSym, align, out Vector3d[] wField, out List<Point3d> sings);

            DA.SetDataList(0, vField);
            DA.SetDataList(1, wField);
            DA.SetDataList(2, sings);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000011");
    }

    public class GH_TrivialConnections : GH_Component
    {
        public GH_TrivialConnections()
          : base("Trivial Connections", "TrivConn",
              "Designs smooth tangent direction fields with user-prescribed cone/vortex singularities (Crane, Desbrun, Schröder 2010).",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("TrivConn");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Singularity Points", "P", "Points on the mesh where singularities (vortices/saddles) are placed", GH_ParamAccess.list);
            pManager.AddNumberParameter("Indices", "I", "Topological index at each singularity (+1 = vortex/source, -1 = saddle, +0.5, etc.)", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vertex Field", "VF", "Parallel-transported tangent vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Face Field", "FF", "Parallel-transported tangent vector per triangle face", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var pts = new List<Point3d>();
            var indices = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, pts);
            DA.GetDataList(2, indices);

            var ddg = new DdgMesh(m);
            var singVerts = new List<int>();
            foreach (var p in pts)
                singVerts.Add(ddg.FindClosestVertex(p));

            Vector3d[] faceField = DdgAlgorithms.ComputeTrivialConnections(ddg, singVerts, indices, 0.0, out Vector3d[] vertField);

            DA.SetDataList(0, vertField);
            DA.SetDataList(1, faceField);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000012");
    }

    public class GH_MeshFieldStreamlines : GH_Component
    {
        public GH_MeshFieldStreamlines()
          : base("Mesh Field Streamlines", "Streamlines",
              "Integrates smooth streamline curves across a mesh following any per-vertex tangent vector field.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("Streamlines");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector Field", "V", "Tangent vector per mesh vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Seeds", "S", "Seed points on the mesh to start streamlines", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Streamlines", "C", "Integrated streamline polylines on the mesh", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var vecsInput = new List<Vector3d>();
            var seeds = new List<Point3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, vecsInput) || vecsInput.Count == 0) return;
            if (!DA.GetDataList(2, seeds) || seeds.Count == 0) return;

            var ddg = new DdgMesh(m);
            Vector3d[] vecs = ddg.MapVectorInputToVertices(vecsInput);
            double stepLen = 0.5 * ddg.MeanEdgeLength;
            int maxSteps = 120;

            var curves = new List<Polyline>();
            foreach (Point3d seed in seeds)
            {
                MeshPoint mp = ddg.RhinoMesh.ClosestMeshPoint(seed, 0.0);
                if (mp == null) continue;

                var pts = new List<Point3d> { mp.Point };
                Point3d curr = mp.Point;
                Vector3d prevDir = Vector3d.Zero;

                for (int step = 0; step < maxSteps; step++)
                {
                    MeshFace f = ddg.RhinoMesh.Faces[mp.FaceIndex];
                    Vector3d vA = vecs[f.A];
                    Vector3d vB = vecs[f.B];
                    Vector3d vC = vecs[f.C];

                    if (prevDir.Length > 1e-12)
                    {
                        if (vA * prevDir < 0) vA = -vA;
                        if (vB * prevDir < 0) vB = -vB;
                        if (vC * prevDir < 0) vC = -vC;
                    }

                    Vector3d dir = mp.T[0] * vA + mp.T[1] * vB + mp.T[2] * vC;
                    if (dir.Length < 1e-12) break;
                    dir.Unitize();
                    prevDir = dir;

                    Point3d nextPt = curr + stepLen * dir;
                    mp = ddg.RhinoMesh.ClosestMeshPoint(nextPt, 0.0);
                    if (mp == null || mp.Point.DistanceToSquared(curr) < 1e-14)
                        break;

                    curr = mp.Point;
                    pts.Add(curr);
                }

                if (pts.Count >= 2)
                    curves.Add(new Polyline(pts));
            }

            DA.SetDataList(0, curves);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000013");
    }

    public class GH_StripePatterns : GH_Component
    {
        public GH_StripePatterns()
          : base("Field Stripe Patterns", "Stripes",
              "Computes equidistant stripe curves separated by exact physical Distance D aligned to a direction field, with automatic topological bifurcations (Knöppel, Crane et al., ACM TOG 2015).",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("Stripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction Field", "V", "Optional tangent direction vector per vertex (leave empty for automatic smoothest field)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distance", "D", "Target physical distance (spacing in Rhino units) between equidistant stripe curves", GH_ParamAccess.item, 0.0);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Equidistant Curves", "C", "Extracted equidistant stripe polylines separated by Distance D", GH_ParamAccess.list);
            pManager.AddNumberParameter("Stripe Wave", "S", "Cosine stripe wave value in [-1, 1] per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Branch Points", "B", "Locations of topological stripe bifurcations (Y-junctions)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Striped Mesh", "CM", "Mesh vertex-colored with the zebra stripe pattern", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var vecs = new List<Vector3d>();
            double dist = 0.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, vecs);
            DA.GetData(2, ref dist);

            var ddg = new DdgMesh(m);
            double[] wave = DdgAlgorithms.ComputeStripePatternsByDistance(
                ddg, vecs, null, dist, 2, out List<Polyline> stripeCurves, out List<Point3d> branches);
            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, wave, true);

            DA.SetDataList(0, stripeCurves);
            DA.SetDataList(1, wave);
            DA.SetDataList(2, branches);
            DA.SetData(3, colored);
        }

        public override Guid ComponentGuid => new Guid("A105B205-C305-4D05-9E05-000000000014");
    }

    public class GH_CurveGuidedStripes : GH_Component
    {
        public GH_CurveGuidedStripes()
          : base("Curve Guided Stripes", "CrvStripes",
              "Generates equidistant stripe curves separated by exact physical Distance D that follow and align with input Guide Curves on the mesh.",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("CrvStripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Guide Curves", "G", "Input curves on or near the mesh that guide the stripe orientation and phase", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distance", "D", "Target physical distance (spacing in Rhino units) between equidistant stripe curves", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Equidistant Curves", "C", "Extracted equidistant stripe polylines separated by Distance D", GH_ParamAccess.list);
            pManager.AddNumberParameter("Stripe Wave", "S", "Cosine stripe wave value in [-1, 1] per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Branch Points", "B", "Locations of topological stripe bifurcations (Y-junctions)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Striped Mesh", "CM", "Mesh vertex-colored with the zebra stripe pattern", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var guides = new List<Curve>();
            double dist = 0.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, guides) || guides.Count == 0) return;
            DA.GetData(2, ref dist);

            var ddg = new DdgMesh(m);
            double[] wave = DdgAlgorithms.ComputeStripePatternsByDistance(
                ddg, null, guides, dist, 1, out List<Polyline> stripeCurves, out List<Point3d> branches);
            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, wave, true);

            DA.SetDataList(0, stripeCurves);
            DA.SetDataList(1, wave);
            DA.SetDataList(2, branches);
            DA.SetData(3, colored);
        }

        public override Guid ComponentGuid => new Guid("A105B205-C305-4D05-9E05-000000000022");
    }

    public class GH_PrincipalCrossField : GH_Component
    {
        public GH_PrincipalCrossField()
          : base("Principal Cross Field", "CrossField",
              "Computes a smooth 4-way symmetric Cross Field (4-RoSy) on a mesh aligned with principal curvatures and sharp feature creases, outputting U/V directions, visual 3D Cross Lines, and singularities.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("CrossField");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Curvature Align", "A", "Alignment weight to principal curvatures and sharp creases (0 = globally smoothest, 1 = strictly curvature-aligned)", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Cross Scale", "S", "Visual scale factor for the output 3D cross lines (relative to edge length)", GH_ParamAccess.item, 0.65);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("U Direction", "U", "First cross tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("V Direction", "V", "Orthogonal second cross tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddLineParameter("Cross Lines", "Crx", "Visual 4-way cross lines at every vertex on the mesh", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "P", "Detected cross-field singularity points (3-way or 5-way quad vertices)", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double align = 0.5;
            double scale = 0.65;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref align);
            DA.GetData(2, ref scale);

            var ddg = new DdgMesh(m);
            Vector3d[] uDir = DdgAlgorithms.ComputeOptimalDirectionField(ddg, 4, align, out Vector3d[] vDir, out List<Point3d> sings);

            double arm = 0.5 * Math.Max(0.05, scale) * ddg.MeanEdgeLength;
            var crossLines = new List<Line>(ddg.VertexCount * 2);
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                Point3d p = ddg.Positions[i];
                crossLines.Add(new Line(p - arm * uDir[i], p + arm * uDir[i]));
                crossLines.Add(new Line(p - arm * vDir[i], p + arm * vDir[i]));
            }

            DA.SetDataList(0, uDir);
            DA.SetDataList(1, vDir);
            DA.SetDataList(2, crossLines);
            DA.SetDataList(3, sings);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000026");
    }

    public class GH_CrossFieldQuadGrid : GH_Component
    {
        public GH_CrossFieldQuadGrid()
          : base("Cross Field Quad Grid", "CrossQuads",
              "Generates two orthogonal sets of equidistant curves (U and V) separated by exact Distance D aligned to a Cross Field.",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("CrossQuads");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Cross U Field", "U", "First cross direction vector per vertex (from Principal Cross Field, or leave empty for automatic)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Quad Distance", "D", "Target physical spacing distance (in Rhino units) between orthogonal quad grid curves", GH_ParamAccess.item, 0.0);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("U Curves", "UC", "Equidistant curves along U cross direction", GH_ParamAccess.list);
            pManager.AddCurveParameter("V Curves", "VC", "Equidistant orthogonal curves along V cross direction", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "B", "Branch / singularity points", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uInput = new List<Vector3d>();
            double dist = 0.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, uInput);
            DA.GetData(2, ref dist);

            var ddg = new DdgMesh(m);
            Vector3d[] uDir;
            Vector3d[] vDir;

            if (uInput.Count > 0)
            {
                uDir = ddg.MapVectorInputToVertices(uInput);
                vDir = new Vector3d[ddg.VertexCount];
                for (int i = 0; i < ddg.VertexCount; i++)
                    vDir[i] = Vector3d.CrossProduct(ddg.VertexNormals[i], uDir[i]);
            }
            else
            {
                uDir = DdgAlgorithms.ComputeOptimalDirectionField(ddg, 4, 0.5, out vDir, out _);
            }

            DdgAlgorithms.ComputeStripePatternsByDistance(ddg, uDir, null, dist, 2, out List<Polyline> uCrvs, out List<Point3d> bU);
            DdgAlgorithms.ComputeStripePatternsByDistance(ddg, vDir, null, dist, 2, out List<Polyline> vCrvs, out List<Point3d> bV);

            var allBranches = new List<Point3d>(bU);
            allBranches.AddRange(bV);

            DA.SetDataList(0, uCrvs);
            DA.SetDataList(1, vCrvs);
            DA.SetDataList(2, allBranches);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000027");
    }

    public class GH_VectorFieldSmooth : GH_Component
    {
        private readonly List<Line> _previewArrows = new List<Line>();

        public GH_VectorFieldSmooth()
          : base("Vector & Gradient Multi-Smooth", "SmoothField",
              "Smooths and redistributes any tangent Vector Field or Scalar Gradient on a mesh using 4 mathematical methods: (0) Connection Laplacian Vector Heat, (1) Helmholtz-Hodge Eikonal Gradient Redistribution, (2) Crease-Anisotropic Feature Smooth, or (3) 4-RoSy Symmetric Frame.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("SmoothField");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxF = pManager.AddVectorParameter("Vectors", "V", "Input vector or gradient field per vertex or face (leave empty for principal curvature directions)", GH_ParamAccess.list);
            pManager[idxF].Optional = true;
            pManager.AddNumberParameter("Smoothness", "T", "Diffusion / redistribution smoothing strength", GH_ParamAccess.item, 2.5);
            pManager.AddIntegerParameter("Method", "Mode", "0 = Connection Laplacian (Vector Heat), 1 = Helmholtz-Hodge Gradient Redistribution (uniform flow), 2 = Crease-Anisotropic Smooth, 3 = 4-RoSy Cross Field", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vertex Field", "V", "Smoothed & redistributed unit tangent vectors per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Face Field", "FF", "Smoothed unit tangent vectors per triangle face", GH_ParamAccess.list);
            pManager.AddVectorParameter("Orthogonal Field", "O", "90-degree rotated tangent vectors (N x V) per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Integrated Scalar", "U", "Redistributed scalar potential whose gradient matches the smoothed vector field", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the redistributed scalar potential", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _previewArrows.Clear();
            Mesh m = null;
            var inVecs = new List<Vector3d>();
            double smoothness = 2.5;
            int mode = 1;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, inVecs);
            DA.GetData(2, ref smoothness);
            DA.GetData(3, ref mode);

            var ddg = new DdgMesh(m);
            Vector3d[] vSmooth = DdgAlgorithms.SmoothVectorAndGradientMultiMode(
                ddg,
                inVecs.Count > 0 ? inVecs : null,
                null,
                smoothness,
                mode,
                out Vector3d[] fSmooth,
                out Vector3d[] orthoVecs,
                out double[] pot,
                out Mesh colored);

            double arrowScale = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 450);
            for (int i = 0; i < ddg.VertexCount; i += stride)
            {
                if (vSmooth[i].Length > 1e-12)
                    _previewArrows.Add(new Line(ddg.Positions[i], ddg.Positions[i] + arrowScale * vSmooth[i]));
            }

            DA.SetDataList(0, vSmooth);
            DA.SetDataList(1, fSmooth);
            DA.SetDataList(2, orthoVecs);
            DA.SetDataList(3, pot);
            DA.SetData(4, colored);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _previewArrows.Count == 0) return;

            Color cArrow = Attributes.Selected ? Color.FromArgb(255, 235, 60) : Color.FromArgb(0, 225, 255);
            for (int i = 0; i < _previewArrows.Count; i++)
                args.Display.DrawArrow(_previewArrows[i], cArrow);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000030");
    }

    public class GH_MeshFieldDisplay : GH_Component
    {
        private readonly List<Line> _primaryLines = new List<Line>();
        private readonly List<Line> _secondaryLines = new List<Line>();
        private readonly List<Point3d> _anchors = new List<Point3d>();
        private readonly List<Vector3d> _arrowDirs = new List<Vector3d>();
        private int _displayMode = 1;

        public GH_MeshFieldDisplay()
          : base("Mesh Field Display", "VecView",
              "Displays any per-vertex or per-face vector, line (2-RoSy), or cross (4-RoSy) field directly on the mesh without needing to extract anchor points manually.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VecView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Field Vectors", "F", "Vectors per vertex or per face (automatically matches vertex or face centers)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Scale", "S", "Visual length multiplier relative to average mesh edge length", GH_ParamAccess.item, 0.75);
            pManager.AddIntegerParameter("Style", "N", "1 = Directed Arrows, 2 = Centered Line Ticks (2-RoSy), 4 = Orthogonal Crosses (4-RoSy)", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Field Lines", "L", "3D line segments representing the vector/cross field on the mesh", GH_ParamAccess.list);
            pManager.AddPointParameter("Anchor Points", "P", "3D anchor points (vertex or face centers) where each vector is placed", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _primaryLines.Clear();
            _secondaryLines.Clear();
            _anchors.Clear();
            _arrowDirs.Clear();

            Mesh m = null;
            var vecs = new List<Vector3d>();
            double scale = 0.75;
            int mode = 1;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, vecs) || vecs.Count == 0) return;
            DA.GetData(2, ref scale);
            DA.GetData(3, ref mode);

            _displayMode = mode <= 1 ? 1 : (mode >= 4 ? 4 : 2);
            var ddg = new DdgMesh(m);
            double drawLen = Math.Max(1e-5, scale * ddg.MeanEdgeLength);

            var outLines = new List<Line>();

            // Automatically detect if vectors correspond to Faces or Vertices
            bool isFaceField = (vecs.Count == ddg.FaceCount && ddg.FaceCount != ddg.VertexCount)
                            || (vecs.Count == m.Faces.Count && m.Faces.Count != m.Vertices.Count);

            if (isFaceField)
            {
                int count = Math.Min(ddg.FaceCount, vecs.Count);
                for (int f = 0; f < count; f++)
                {
                    int[] fv = ddg.Faces[f];
                    Point3d center = (ddg.Positions[fv[0]] + ddg.Positions[fv[1]] + ddg.Positions[fv[2]]) / 3.0;
                    Vector3d n = ddg.FaceNormals[f];
                    AddFieldGlyph(center, vecs[f], n, drawLen, _displayMode, outLines);
                }
            }
            else
            {
                Vector3d[] mapped = ddg.MapVectorInputToVertices(vecs);
                for (int v = 0; v < ddg.VertexCount; v++)
                {
                    AddFieldGlyph(ddg.Positions[v], mapped[v], ddg.VertexNormals[v], drawLen, _displayMode, outLines);
                }
            }

            DA.SetDataList(0, outLines);
            DA.SetDataList(1, _anchors);
        }

        private void AddFieldGlyph(Point3d p, Vector3d v, Vector3d normal, double drawLen, int mode, List<Line> outLines)
        {
            if (!v.IsValid || v.Length < 1e-12) return;
            Vector3d dir = v;
            dir.Unitize();

            _anchors.Add(p);

            if (mode == 1)
            {
                Line ln = new Line(p, p + dir * drawLen);
                _primaryLines.Add(ln);
                _arrowDirs.Add(dir);
                outLines.Add(ln);
            }
            else if (mode == 2)
            {
                double half = 0.5 * drawLen;
                Line ln = new Line(p - dir * half, p + dir * half);
                _primaryLines.Add(ln);
                outLines.Add(ln);
            }
            else
            {
                double half = 0.5 * drawLen;
                Vector3d ortho = Vector3d.CrossProduct(normal, dir);
                if (ortho.Length > 1e-12) ortho.Unitize();
                Line lnU = new Line(p - dir * half, p + dir * half);
                Line lnV = new Line(p - ortho * half, p + ortho * half);
                _primaryLines.Add(lnU);
                _secondaryLines.Add(lnV);
                outLines.Add(lnU);
                outLines.Add(lnV);
            }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _primaryLines.Count == 0) return;

            Color cPrimary = Attributes.Selected ? Color.FromArgb(255, 230, 50) : Color.FromArgb(0, 215, 255);
            Color cSecondary = Attributes.Selected ? Color.FromArgb(255, 140, 60) : Color.FromArgb(245, 85, 70);

            if (_displayMode == 1)
            {
                for (int i = 0; i < _primaryLines.Count; i++)
                {
                    args.Display.DrawArrow(_primaryLines[i], cPrimary);
                }
            }
            else
            {
                args.Display.DrawLines(_primaryLines, cPrimary, 2);
                if (_secondaryLines.Count > 0)
                    args.Display.DrawLines(_secondaryLines, cSecondary, 2);
            }
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000031");
    }

    public class GH_VectorFamilyDiscretizer : GH_Component
    {
        private readonly List<Line> _familyArrows = new List<Line>();
        private readonly List<Color> _arrowColors = new List<Color>();

        public GH_VectorFamilyDiscretizer()
          : base("Vector Family Discretizer", "VecFamily",
              "Discretizes a continuous tangent vector field on a mesh into a finite Family of Vectors (either user-supplied reference vectors or K clustered directions), grouping faces by discrete vector family and displaying color-coded arrows.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VecFamily");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Continuous Field", "F", "Input continuous vector field per vertex or face (leave empty for principal curvature directions)", GH_ParamAccess.list);
            pManager[1].Optional = true;
            int idxFam = pManager.AddVectorParameter("Vector Family", "VF", "Optional custom family of vectors to snap/quantize onto (if empty, automatically generates K optimal directions)", GH_ParamAccess.list);
            pManager[idxFam].Optional = true;
            pManager.AddIntegerParameter("Family Count", "K", "Number of discrete vector directions if Vector Family VF is not provided", GH_ParamAccess.item, 6);
            pManager.AddBooleanParameter("Bidirectional (2-RoSy)", "B", "True = treat +V and -V as the same line family; False = strictly directed vectors", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Discretized Face Field", "DF", "Quantized tangent vector per face snapped to the Vector Family", GH_ParamAccess.list);
            pManager.AddVectorParameter("Discretized Vertex Field", "DV", "Quantized tangent vector per vertex snapped to the Vector Family", GH_ParamAccess.list);
            pManager.AddVectorParameter("Canonical Family", "CF", "The K discrete vectors of the family", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Family Index", "ID", "Assigned vector family index (0..K-1) per triangle face", GH_ParamAccess.list);
            pManager.AddMeshParameter("Family Sub-Meshes", "SM", "Mesh segmented into pieces sharing the same discrete vector family member", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Combined mesh colored by discrete vector family", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _familyArrows.Clear();
            _arrowColors.Clear();

            Mesh m = null;
            var contField = new List<Vector3d>();
            var customFam = new List<Vector3d>();
            int kCount = 6;
            bool biDir = false;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, contField);
            DA.GetDataList(2, customFam);
            DA.GetData(3, ref kCount);
            DA.GetData(4, ref biDir);

            var ddg = new DdgMesh(m);
            Vector3d[] discFace = DdgAlgorithms.DiscretizeVectorFieldByFamily(
                ddg,
                contField.Count > 0 ? contField : null,
                customFam.Count > 0 ? customFam : null,
                kCount,
                biDir,
                out Vector3d[] discVert,
                out Vector3d[] canonFam,
                out int[] faceIds,
                out List<Mesh> subMeshes,
                out Mesh coloredMesh);

            double arrowLen = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.FaceCount / 600);
            for (int f = 0; f < ddg.FaceCount; f += stride)
            {
                Vector3d v = discFace[f];
                if (v.Length > 1e-12)
                {
                    Point3d c = ddg.FaceCenters[f];
                    _familyArrows.Add(new Line(c, c + arrowLen * v));
                    double t = canonFam.Length > 1 ? (double)faceIds[f] / (canonFam.Length - 1) : 0.5;
                    _arrowColors.Add(DdgAlgorithms.EvaluateScientificColor(t, 3));
                }
            }

            DA.SetDataList(0, discFace);
            DA.SetDataList(1, discVert);
            DA.SetDataList(2, canonFam);
            DA.SetDataList(3, faceIds);
            DA.SetDataList(4, subMeshes);
            DA.SetData(5, coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _familyArrows.Count == 0) return;

            for (int i = 0; i < _familyArrows.Count; i++)
                args.Display.DrawArrow(_familyArrows[i], _arrowColors[i]);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000035");
    }
}
