using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_OptimalDirectionField : GH_Component
    {
        private readonly List<Line> _previewSegs = new List<Line>();

        public GH_OptimalDirectionField()
          : base("Smooth Optimal Direction Field", "OptField",
              "Computes a globally smooth N-Direction Field (1=Vector, 2=Line, 4=Cross, 6=Hex) on a mesh aligned with principal curvatures.",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override Bitmap Icon => CraneIcons.GetIcon("OptField");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Symmetry", "N", "Symmetry degree N (1=Vector, 2=Line, 4=Cross, 6=Hexagonal)", GH_ParamAccess.item, 4);
            pManager.AddNumberParameter("Curvature Align", "A", "Alignment weight to principal curvature directions (0 = Globally Smoothest)", GH_ParamAccess.item, 0.5);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Primary Field", "V", "Primary unit tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Orthogonal Field", "W", "90-degree rotated unit tangent direction vector per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "S", "Detected singularity face centers", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _previewSegs.Clear();
            Mesh m = null;
            int nSym = 4;
            double align = 0.5;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref nSym);
            DA.GetData(2, ref align);

            var ddg = new DdgMesh(m);
            Vector3d[] vField = DdgAlgorithms.ComputeOptimalDirectionField(ddg, nSym, align, out Vector3d[] wField, out List<Point3d> sings);

            double segLen = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 700);
            for (int i = 0; i < ddg.VertexCount; i += stride)
            {
                Point3d p = ddg.Positions[i];
                _previewSegs.Add(new Line(p - 0.5 * segLen * vField[i], p + 0.5 * segLen * vField[i]));
            }

            DA.SetDataList(0, vField);
            DA.SetDataList(1, wField);
            DA.SetDataList(2, sings);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _previewSegs.Count == 0) return;

            Color cSeg = Attributes.Selected ? Color.FromArgb(255, 240, 70) : Color.FromArgb(80, 225, 255);
            args.Display.DrawLines(_previewSegs, cSeg, 2);
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
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

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
          : base("Vector Field Stripes", "Stripes",
              "Generates equidistant stripe curves separated by Distance D from a Vector Field V, with control over whether stripe ends touch (default = False).",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("Stripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector Field", "V", "Tangent direction vector per vertex or face (leave empty for automatic principal curvature field)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distance", "D", "Target physical spacing distance (in Rhino units) between stripe curves", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Touch Ends", "T", "False (default) = stripe ends stay separated at bifurcations/loops; True = stripe ends touch and connect", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Stripe Curves", "C", "Extracted equidistant stripe curves", GH_ParamAccess.list);
            pManager.AddNumberParameter("Stripe Wave", "S", "Cosine stripe wave value in [-1, 1] per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Branch Points", "B", "Locations of topological stripe bifurcations (Y-junctions)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Striped Mesh", "CM", "Mesh vertex-colored with the stripe pattern", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var vecs = new List<Vector3d>();
            double dist = 0.0;
            bool touchEnds = false;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, vecs);
            DA.GetData(2, ref dist);
            DA.GetData(3, ref touchEnds);

            var ddg = new DdgMesh(m);
            double[] wave = DdgAlgorithms.ComputeStripePatternsByDistance(
                ddg, vecs, null, dist, 2, out List<Polyline> stripeCurves, out List<Point3d> branches, null, touchEnds);
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
        private readonly List<Line> _vecSegs = new List<Line>();
        private readonly List<Polyline> _stripes = new List<Polyline>();

        public GH_CurveGuidedStripes()
          : base("Vector Field & Stripes from Curves", "CrvField",
              "Generates a smooth Tangent Vector Field V on a mesh aligned with input Guide Curves G, plus equidistant Stripe Curves C (with Touch Ends control, default = False).",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override Bitmap Icon => CraneIcons.GetIcon("CrvStripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Guide Curves", "G", "Input curves on or near the mesh that generate the tangent vector field and guide stripes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Stripe Distance", "D", "Target physical spacing distance (in Rhino units) between stripe curves (0 = automatic)", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Touch Ends", "T", "False (default) = stripe ends do NOT touch; True = stripe ends touch / connect", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Curve Vector Field", "V", "Smooth tangent vector field on the mesh aligned parallel to the Guide Curves G", GH_ParamAccess.list);
            pManager.AddVectorParameter("Ortho Vector Field", "O", "Orthogonal tangent vector field (N x V) perpendicular to Guide Curves G", GH_ParamAccess.list);
            pManager.AddCurveParameter("Stripe Curves", "C", "Equidistant stripe curves aligned with the Guide Curves G", GH_ParamAccess.list);
            pManager.AddMeshParameter("Striped Mesh", "CM", "Mesh vertex-colored with the stripe pattern", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _vecSegs.Clear();
            _stripes.Clear();

            Mesh m = null;
            var guides = new List<Curve>();
            double dist = 0.0;
            bool touchEnds = false;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, guides) || guides.Count == 0) return;
            DA.GetData(2, ref dist);
            DA.GetData(3, ref touchEnds);

            var ddg = new DdgMesh(m);

            // Sample tangent vectors along guide curves and parallel-transport across the mesh
            var srcVerts = new List<int>();
            var srcVecs = new List<Vector3d>();
            foreach (Curve c in guides)
            {
                if (c == null) continue;
                int sampleCount = Math.Max(16, Math.Min(250, (int)(c.GetLength() / Math.Max(1e-5, 0.6 * ddg.MeanEdgeLength))));
                c.DivideByCount(sampleCount, true, out Point3d[] pts);
                if (pts == null) continue;

                foreach (Point3d p in pts)
                {
                    int vIdx = ddg.FindClosestVertex(p);
                    if (c.ClosestPoint(p, out double tParam))
                    {
                        Vector3d tan = c.TangentAt(tParam);
                        Vector3d n = ddg.VertexNormals[vIdx];
                        tan -= (tan * n) * n;
                        if (tan.Unitize())
                        {
                            srcVerts.Add(vIdx);
                            srcVecs.Add(tan);
                        }
                    }
                }
            }

            Vector3d[] vField = srcVerts.Count > 0
                ? DdgAlgorithms.ComputeVectorHeatTransport(ddg, srcVerts, srcVecs, 2.0)
                : ddg.PrincipalDir;

            var orthoField = new Vector3d[ddg.VertexCount];
            double segLen = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 650);
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                Vector3d v = vField[i];
                if (v.Unitize()) vField[i] = v;
                Vector3d o = Vector3d.CrossProduct(ddg.VertexNormals[i], v);
                if (o.Unitize()) orthoField[i] = o;

                if (i % stride == 0)
                {
                    Point3d p = ddg.Positions[i];
                    _vecSegs.Add(new Line(p - 0.5 * segLen * vField[i], p + 0.5 * segLen * vField[i]));
                }
            }

            double[] wave = DdgAlgorithms.ComputeStripePatternsByDistance(
                ddg, null, guides, dist, 1, out List<Polyline> stripeCurves, out _, null, touchEnds);
            _stripes.AddRange(stripeCurves);
            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, wave, true);

            DA.SetDataList(0, vField);
            DA.SetDataList(1, orthoField);
            DA.SetDataList(2, stripeCurves);
            DA.SetData(3, colored);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            if (_vecSegs.Count > 0)
                args.Display.DrawLines(_vecSegs, Color.FromArgb(180, 255, 195, 50), 2);

            foreach (var pl in _stripes)
                args.Display.DrawPolyline(pl, Color.FromArgb(0, 235, 255), 2);
        }

        public override Guid ComponentGuid => new Guid("A105B205-C305-4D05-9E05-000000000022");
    }

    public class GH_PrincipalCrossField : GH_Component
    {
        private readonly List<Line> _uLines = new List<Line>();
        private readonly List<Line> _vLines = new List<Line>();
        private readonly List<Point3d> _singularities = new List<Point3d>();

        public GH_PrincipalCrossField()
          : base("Cross Field (U & V)", "CrossField",
              "Builds orthogonal U and V tangent vector fields and lightweight X-Cross segments from an Input Vector Field V (or from Principal Curvatures if V is empty).",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override Bitmap Icon => CraneIcons.GetIcon("CrossField");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxV = pManager.AddVectorParameter("Input Field", "V", "Input tangent vector field (e.g. from SmoothField or WalkOnMesh) to decompose into U and V cross directions (leave empty for Principal Curvature Cross Field)", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
            pManager.AddNumberParameter("Align / Fidelity", "A", "If V is provided: 1.0 = exact U & V from input field V, <1.0 = 4-RoSy cross relaxation. If V is empty: alignment to principal curvatures (0..1)", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Cross Scale", "S", "Visual scale factor for the output 3D X-cross line segments (relative to edge length)", GH_ParamAccess.item, 0.65);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("U Field", "U", "Primary tangent direction vector U per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("V Field", "V", "Orthogonal second cross tangent direction vector V (N x U) per vertex", GH_ParamAccess.list);
            pManager.AddLineParameter("X-Cross Lines", "Crx", "Lightweight 4-way X-cross line segments at every vertex on the mesh", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "P", "Detected cross-field singularity points (3-way or 5-way quad vertices)", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _uLines.Clear();
            _vLines.Clear();
            _singularities.Clear();

            Mesh m = null;
            var inField = new List<Vector3d>();
            double align = 1.0;
            double scale = 0.65;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, inField);
            DA.GetData(2, ref align);
            DA.GetData(3, ref scale);

            var ddg = new DdgMesh(m);
            double effAlign = (inField.Count == 0 && Params.Input[2].SourceCount == 0) ? 0.5 : align;
            Vector3d[] uDir = DdgAlgorithms.ComputeOptimalDirectionField(
                ddg,
                4,
                effAlign,
                out Vector3d[] vDir,
                out List<Point3d> sings,
                inField.Count > 0 ? inField : null);
            _singularities.AddRange(sings);

            double arm = 0.5 * Math.Max(0.05, scale) * ddg.MeanEdgeLength;
            var crossLines = new List<Line>(ddg.VertexCount * 2);
            int stride = Math.Max(1, ddg.VertexCount / 1200);
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                Point3d p = ddg.Positions[i];
                Line lu = new Line(p - arm * uDir[i], p + arm * uDir[i]);
                Line lv = new Line(p - arm * vDir[i], p + arm * vDir[i]);
                crossLines.Add(lu);
                crossLines.Add(lv);
                if (i % stride == 0)
                {
                    _uLines.Add(lu);
                    _vLines.Add(lv);
                }
            }

            DA.SetDataList(0, uDir);
            DA.SetDataList(1, vDir);
            DA.SetDataList(2, crossLines);
            DA.SetDataList(3, sings);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _uLines.Count == 0) return;

            Color cU = Attributes.Selected ? Color.FromArgb(255, 240, 80) : Color.FromArgb(0, 225, 255);
            Color cV = Attributes.Selected ? Color.FromArgb(255, 160, 60) : Color.FromArgb(255, 95, 80);
            args.Display.DrawLines(_uLines, cU, 2);
            args.Display.DrawLines(_vLines, cV, 2);
            foreach (Point3d sp in _singularities)
                args.Display.DrawPoint(sp, Rhino.Display.PointStyle.RoundSimple, 6, Color.FromArgb(255, 230, 40));
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000026");
    }

    public class GH_CrossFieldQuadGrid : GH_Component
    {
        private readonly List<Polyline> _uCurves = new List<Polyline>();
        private readonly List<Polyline> _vCurves = new List<Polyline>();
        private readonly List<Point3d> _sings = new List<Point3d>();

        public GH_CrossFieldQuadGrid()
          : base("Cross Field Quad Stripes", "CrossQuads",
              "Generates and displays two orthogonal sets of equidistant Quad Stripe curves (U and V) separated by Distance D aligned to a Cross Field (with Touch Ends control, default = False).",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override Bitmap Icon => CraneIcons.GetIcon("CrossQuads");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Cross U Field", "U", "First cross direction vector per vertex (from CrossField, or leave empty for automatic Principal Cross Field)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Quad Distance", "D", "Target physical spacing distance (in Rhino units) between orthogonal U and V quad stripe curves", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Touch Ends", "T", "False (default) = stripe ends stay separated at bifurcations/loops; True = stripe ends touch", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("U Stripes", "UC", "Equidistant stripe curves along U cross direction", GH_ParamAccess.list);
            pManager.AddCurveParameter("V Stripes", "VC", "Equidistant orthogonal stripe curves along V cross direction", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "B", "Branch / singularity points", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _uCurves.Clear();
            _vCurves.Clear();
            _sings.Clear();

            Mesh m = null;
            var uInput = new List<Vector3d>();
            double dist = 0.0;
            bool touchEnds = false;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, uInput);
            DA.GetData(2, ref dist);
            DA.GetData(3, ref touchEnds);

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

            DdgAlgorithms.ComputeStripePatternsByDistance(ddg, uDir, null, dist, 2, out List<Polyline> uCrvs, out List<Point3d> bU, null, touchEnds);
            DdgAlgorithms.ComputeStripePatternsByDistance(ddg, vDir, null, dist, 2, out List<Polyline> vCrvs, out List<Point3d> bV, null, touchEnds);

            _uCurves.AddRange(uCrvs);
            _vCurves.AddRange(vCrvs);
            _sings.AddRange(bU);
            _sings.AddRange(bV);

            DA.SetDataList(0, uCrvs);
            DA.SetDataList(1, vCrvs);
            DA.SetDataList(2, _sings);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            Color cU = Attributes.Selected ? Color.FromArgb(255, 240, 80) : Color.FromArgb(0, 215, 255);
            Color cV = Attributes.Selected ? Color.FromArgb(255, 165, 60) : Color.FromArgb(255, 95, 80);

            foreach (var pl in _uCurves)
                args.Display.DrawPolyline(pl, cU, 2);
            foreach (var pl in _vCurves)
                args.Display.DrawPolyline(pl, cV, 2);

            if (_sings.Count > 0)
                args.Display.DrawPoints(_sings, Rhino.Display.PointStyle.RoundSimple, 4, Color.Gold);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000027");
    }

    public class GH_VectorFieldSmooth : GH_Component
    {
        private readonly List<Line> _previewArrows = new List<Line>();

        public GH_VectorFieldSmooth()
          : base("Smooth Vector Field", "SmoothField",
              "Smooths and redistributes any tangent Vector Field or Scalar Gradient on a mesh (0=Connection Laplacian, 1=Helmholtz-Hodge Redistribution, 2=Crease-Anisotropic, 3=4-RoSy).",
              "CraneGH", "03. Vector Fields & Stripes")
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
            args.Display.DrawLines(_previewArrows, cArrow, 2);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000030");
    }

    public class GH_MeshFieldDisplay : GH_Component
    {
        private readonly List<Line> _primaryLines = new List<Line>();
        private readonly List<Line> _secondaryLines = new List<Line>();
        private readonly List<Color> _glyphColors = new List<Color>();
        private Mesh _coloredMesh;
        private int _displayMode = 0;

        public GH_MeshFieldDisplay()
          : base("Vector Field Visualizer", "VecView",
              "Visualizes any Vector Field (0), X-Cross Field (1), Scalar Gradient (2), or Vector Gradient/Div/Curl (3) on a mesh using fast batched GPU line segments.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VecView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxF = pManager.AddGenericParameter("Vectors or Scalars", "F", "Tangent Vectors (Vector3d) or Scalar Field (double) to compute Geodesic Gradient (leave empty for Principal Curvature X-Cross Field)", GH_ParamAccess.list);
            pManager[idxF].Optional = true;
            pManager.AddNumberParameter("Scale", "S", "Visual segment length multiplier relative to average mesh edge length", GH_ParamAccess.item, 0.75);
            pManager.AddIntegerParameter("Mode", "Mode", "0 = Vector Segments + Cross Ticks, 1 = 4-RoSy Symmetric X-Crosses, 2 = Geodesic Gradient Segments + Wavefront Ticks, 3 = Covariant Vector Gradient + Divergence/Curl", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Primary Lines", "L", "Primary 3D vector segments or U-cross lines", GH_ParamAccess.list);
            pManager.AddLineParameter("X-Cross / Ortho Lines", "Crx", "Orthogonal 4-RoSy V-cross lines or iso-wavefront ticks", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vector Gradient", "VG", "Covariant Vector Gradient (direction & rate of spatial change of the vector field)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Divergence", "Div", "Discrete Divergence (div V) per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Curl / Rotor", "Curl", "Discrete Curl ((nabla x V) . N) per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Analysis Mesh", "CM", "Mesh colored by Magnitude (0), Curl (1), Geodesic Slope (2), or Vector Gradient Norm (3)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _primaryLines.Clear();
            _secondaryLines.Clear();
            _glyphColors.Clear();
            _coloredMesh = null;

            Mesh m = null;
            var rawItems = new List<object>();
            double scale = 0.75;
            int mode = 1;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawItems);
            DA.GetData(2, ref scale);
            DA.GetData(3, ref mode);

            // Map legacy style values (4 -> 1 for 4-RoSy Crosses)
            if (mode == 4) mode = 1;
            _displayMode = Math.Max(0, Math.Min(3, mode));

            var inVecs = new List<Vector3d>();
            var inScalars = new List<double>();

            foreach (object obj in rawItems)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is Vector3d v) inVecs.Add(v);
                else if (val is Point3d pt) inVecs.Add(new Vector3d(pt.X, pt.Y, pt.Z));
                else if (val is double d) inScalars.Add(d);
                else if (val is int iv) inScalars.Add(iv);
            }

            if (inScalars.Count > 0 && inVecs.Count == 0 && Params.Input[3].SourceCount == 0)
            {
                // Auto-switch to Geodesic Gradient + Wavefront Ticks if user connected a scalar field
                _displayMode = 2;
            }

            var ddg = new DdgMesh(m);
            DdgAlgorithms.ComputeVectorFieldGradientsAndGlyphs(
                ddg,
                inVecs.Count > 0 ? inVecs : null,
                inScalars.Count > 0 ? inScalars : null,
                scale,
                _displayMode,
                out List<Line> primLines,
                out List<Line> orthoLines,
                out Vector3d[] covGrad,
                out double[] div,
                out double[] curl,
                out _coloredMesh,
                out List<Color> cols);

            _primaryLines.AddRange(primLines);
            _secondaryLines.AddRange(orthoLines);
            _glyphColors.AddRange(cols);

            DA.SetDataList(0, primLines);
            DA.SetDataList(1, orthoLines);
            DA.SetDataList(2, covGrad);
            DA.SetDataList(3, div);
            DA.SetDataList(4, curl);
            DA.SetData(5, _coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _primaryLines.Count == 0) return;

            Color cPrim = Attributes.Selected ? Color.FromArgb(255, 235, 60) : Color.FromArgb(0, 225, 255);
            Color cOrtho = Attributes.Selected ? Color.FromArgb(255, 160, 70) : Color.FromArgb(245, 95, 80);

            // Fast batched GPU line segment rendering (no heavy 3D cone arrows)
            args.Display.DrawLines(_primaryLines, cPrim, 2);
            if (_secondaryLines.Count > 0)
            {
                Color cSec = _displayMode == 1 ? cOrtho : Color.FromArgb(190, 255, 180, 90);
                args.Display.DrawLines(_secondaryLines, cSec, _displayMode == 1 ? 2 : 1);
            }
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000031");
    }

    public class GH_ScalarWeightedVectorFlow : GH_Component
    {
        private readonly List<Polyline> _flowPaths = new List<Polyline>();
        private readonly List<Line> _guidedArrows = new List<Line>();
        private Mesh _relMesh;

        public GH_ScalarWeightedVectorFlow()
          : base("Scalar-Attracted Vector Field", "ScalarVec",
              "Combines a Vector Field (V) with a Scalar Weight Field (S in [0..1]) so vectors bend toward high-scalar zones (S=1) and avoid low-scalar zones (S=0). Outputs the new Attracted Vector Field (VA) and Flow Curves.",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GuidedFlow");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxV = pManager.AddVectorParameter("Vector Field", "V", "Base tangent vector field per vertex or face", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
            int idxW = pManager.AddNumberParameter("Scalar Weight", "S", "Scalar field where low values (0) are avoided and high values (1) attract the vectors", GH_ParamAccess.list);
            pManager[idxW].Optional = true;
            pManager.AddNumberParameter("Attract Strength", "A", "How strongly vectors bend toward the gradient of high scalar values (0 = only scale by S, 1 = strong attraction to S=1)", GH_ParamAccess.item, 0.75);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Attracted Vectors", "VA", "Combined tangent vector field deflected toward high-scalar regions (plug into Stripes, CrossField, or VecFamily!)", GH_ParamAccess.list);
            pManager.AddCurveParameter("Flow Curves", "C", "Streamline curves traveling along the attracted vector field through S=1 zones", GH_ParamAccess.list);
            pManager.AddMeshParameter("Weight Mesh", "CM", "Mesh colored by normalized Scalar Weight S in [0..1] (Blue = 0, Gold/Red = 1)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _flowPaths.Clear();
            _guidedArrows.Clear();
            _relMesh = null;

            Mesh m = null;
            var inVecs = new List<Vector3d>();
            var inWeights = new List<double>();
            double attract = 0.75;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, inVecs);
            DA.GetDataList(2, inWeights);
            DA.GetData(3, ref attract);

            var ddg = new DdgMesh(m);
            List<Polyline> paths = DdgAlgorithms.ComputeScalarWeightedVectorGuidedToolpaths(
                ddg,
                inVecs.Count > 0 ? inVecs : null,
                inWeights.Count > 0 ? inWeights : null,
                null,
                out _,
                out Vector3d[] guidedVecs,
                out _,
                out _relMesh,
                out double[] normW);

            // Blend base vectors and guided drift by user's Attract Strength A
            Vector3d[] baseV = inVecs.Count > 0 ? ddg.MapVectorInputToVertices(inVecs) : ddg.PrincipalDir;
            ddg.ComputeGradient(normW, out _, out Vector3d[] gradW);
            double alpha = Math.Max(0.0, Math.Min(5.0, attract));
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                Vector3d bv = baseV[i];
                if (bv.Length > 1e-12) bv.Unitize();
                Vector3d gw = gradW[i];
                if (gw.Length > 1e-12) gw.Unitize();
                Vector3d comb = bv + alpha * gw;
                comb -= (comb * ddg.VertexNormals[i]) * ddg.VertexNormals[i];
                if (comb.Unitize())
                    guidedVecs[i] = comb * (0.15 + 0.85 * normW[i]);
            }

            _flowPaths.AddRange(paths);

            double arrowLen = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 500);
            for (int i = 0; i < ddg.VertexCount; i += stride)
            {
                if (guidedVecs[i].Length > 1e-12 && normW[i] > 0.08)
                {
                    _guidedArrows.Add(new Line(ddg.Positions[i], ddg.Positions[i] + arrowLen * guidedVecs[i]));
                }
            }

            DA.SetDataList(0, guidedVecs);
            DA.SetDataList(1, paths);
            DA.SetData(2, _relMesh);
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            base.DrawViewportMeshes(args);
            if (Hidden || Locked || _relMesh == null) return;
            args.Display.DrawMeshFalseColors(_relMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            if (_guidedArrows.Count > 0)
                args.Display.DrawLines(_guidedArrows, Color.FromArgb(190, 255, 240, 120), 2);

            foreach (var pl in _flowPaths)
                args.Display.DrawPolyline(pl, Color.FromArgb(0, 255, 215), 2);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000044");
    }

    public class GH_VectorFamilyDiscretizer : GH_Component
    {
        private readonly List<Line> _familyArrows = new List<Line>();
        private readonly List<Color> _arrowColors = new List<Color>();

        public GH_VectorFamilyDiscretizer()
          : base("Discretize Vector Field", "VecFamily",
              "Quantizes a continuous Vector Field into K discrete vector directions and segments the mesh into pieces sharing the same vector family.",
              "CraneGH", "03. Vector Fields & Stripes")
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
                    _familyArrows.Add(new Line(c - 0.5 * arrowLen * v, c + 0.5 * arrowLen * v));
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

            args.Display.DrawLines(_familyArrows, Color.FromArgb(230, 255, 255, 255), 2);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000035");
    }

    public class GH_SpiralIsoConnector : GH_Component
    {
        private Curve _spiralCurve;
        private readonly List<Curve> _isoCurves = new List<Curve>();
        private readonly List<Line> _crossLines = new List<Line>();

        public GH_SpiralIsoConnector()
          : base("Smooth Spiral Iso-Curve", "SpiralIso",
              "Dedicated component that extracts a smooth, non-polygonal Degree-3 NURBS Helical Spiral Iso-Curve (or Fermat Spiral) on a mesh from any Scalar Field, Smoothed Scalar Mesh, or Scalar Gradient.",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("Stripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (or Colored Mesh from SmoothScalar)", GH_ParamAccess.item);
            int idxS = pManager.AddGenericParameter("Scalar or Gradient", "S", "Scalar Field (double), Vector/Gradient Field (Vector3d), or External Geometry (leave empty for automatic Geodesic Height)", GH_ParamAccess.list);
            pManager[idxS].Optional = true;
            pManager.AddNumberParameter("Turn Pitch / Distance", "D", "Physical step-over spacing distance (in Rhino units) between spiral turns (<= 0 for 24 automatic turns)", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("Spiral Mode", "Mode", "0 = Smooth Continuous Helical Iso-Spiral, 1 = Smooth Fermat Out-And-In Single-Track Spiral", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Smooth Spiral Iso-Curve", "SP", "Smooth, non-polygonal Degree-3 NURBS spiral curve across the mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Smooth Iso-Curves", "C", "Smooth Degree-3 NURBS iso-contour curves per turn", GH_ParamAccess.list);
            pManager.AddLineParameter("X-Cross Field", "Crx", "Lightweight X-cross tangent and gradient line segments", GH_ParamAccess.list);
            pManager.AddNumberParameter("Potential Scalar", "U", "Active scalar field driving the spiral", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _spiralCurve = null;
            _isoCurves.Clear();
            _crossLines.Clear();

            Mesh m = null;
            var rawS = new List<object>();
            double stepDist = 0.0;
            int mode = 0;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawS);
            DA.GetData(2, ref stepDist);
            DA.GetData(3, ref mode);

            var ddg = new DdgMesh(m);
            var scalars = new List<double>();
            var gradVecs = new List<Vector3d>();
            var extGeo = new List<GeometryBase>();

            foreach (object obj in rawS)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is double d) scalars.Add(d);
                else if (val is int iv) scalars.Add(iv);
                else if (val is Vector3d v) gradVecs.Add(v);
                else if (val is GeometryBase g) extGeo.Add(g);
            }

            double[] uField;
            if (gradVecs.Count > 0)
            {
                DdgAlgorithms.SmoothVectorAndGradientMultiMode(ddg, gradVecs, null, 1.5, 1, out _, out _, out uField, out _);
            }
            else if (extGeo.Count > 0)
            {
                uField = DdgAlgorithms.ComputeExternalGeometryHeatDistance(ddg, extGeo, 1.0, out _, out _, out _, out _);
            }
            else if (scalars.Count > 0)
            {
                uField = ddg.MapScalarInputToVertices(scalars);
            }
            else if (m.VertexColors != null && m.VertexColors.Count == m.Vertices.Count)
            {
                double[] rawCol = ddg.MapScalarInputToVertices(null);
                uField = DdgAlgorithms.SmoothScalarField(ddg, rawCol, 1.5, 0.0, out _, out _);
            }
            else
            {
                uField = DdgAlgorithms.ComputeHeatDistance(ddg, new int[] { 0 }, 1.0, out _, out _);
            }

            _spiralCurve = DdgAlgorithms.ComputeSmoothSpiralIsoCurveOnMesh(
                ddg,
                uField,
                stepDist,
                mode,
                out List<Curve> smoothIsoCrvs);

            _isoCurves.AddRange(smoothIsoCrvs);

            DdgAlgorithms.ComputeWalkOnMesh(ddg, uField, null, 0.0, 2, 0, out Vector3d[] vGrad, out _, out _);
            double arm = 0.45 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 600);
            for (int i = 0; i < ddg.VertexCount; i += stride)
            {
                Vector3d g = vGrad[i];
                if (g.Unitize())
                {
                    Vector3d t = Vector3d.CrossProduct(ddg.VertexNormals[i], g);
                    t.Unitize();
                    Point3d p = ddg.Positions[i];
                    _crossLines.Add(new Line(p - arm * t, p + arm * t));
                    _crossLines.Add(new Line(p - arm * g, p + arm * g));
                }
            }

            DA.SetData(0, _spiralCurve);
            DA.SetDataList(1, _isoCurves);
            DA.SetDataList(2, _crossLines);
            DA.SetDataList(3, uField);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            if (_crossLines.Count > 0)
                args.Display.DrawLines(_crossLines, Color.FromArgb(120, 0, 215, 255), 1);

            if (_spiralCurve != null && _spiralCurve.IsValid)
                args.Display.DrawCurve(_spiralCurve, Color.FromArgb(255, 220, 40), 3);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000048");
    }

    public class GH_VariableDensityStripes : GH_Component
    {
        private readonly List<Polyline> _stripeCurves = new List<Polyline>();
        private readonly List<Line> _xCrossSegments = new List<Line>();
        private Mesh _stripedMesh;

        public GH_VariableDensityStripes()
          : base("Scalar & Gradient Stripes", "VarStripes",
              "Generates variable-spacing stripes from a Scalar Field or Scalar Gradient (high values = MORE stripes / closer spacing, low values = FEWER stripes / wider spacing), with control over whether stripe ends touch (default = False).",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VarStripes");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxS = pManager.AddGenericParameter("Scalar or Gradient", "S", "Scalar Field (double) or Scalar Gradient Vectors (Vector3d): high values = MORE stripes, low values = FEWER stripes", GH_ParamAccess.list);
            pManager[idxS].Optional = true;
            pManager.AddNumberParameter("Base Spacing", "D", "Base physical spacing distance (in Rhino units) between stripes (0 = automatic)", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Touch Ends", "T", "False (default) = stripe ends do NOT touch; True = stripe ends touch / close", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Stripe Curves", "C", "Variable-density stripe curves (dense where S/Gradient is high, sparse where S/Gradient is low)", GH_ParamAccess.list);
            pManager.AddLineParameter("X-Cross Segments", "Crx", "Lightweight 4-way X-cross tangent segments scaled by local stripe density", GH_ParamAccess.list);
            pManager.AddPointParameter("Singularities", "B", "Stripe bifurcation / branch singularity points", GH_ParamAccess.list);
            pManager.AddMeshParameter("Striped Mesh", "SM", "Mesh colored with the variable-density stripe pattern", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _stripeCurves.Clear();
            _xCrossSegments.Clear();
            _stripedMesh = null;

            Mesh m = null;
            var rawS = new List<object>();
            double baseDist = 0.0;
            bool touchEnds = false;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawS);
            DA.GetData(2, ref baseDist);
            DA.GetData(3, ref touchEnds);

            var ddg = new DdgMesh(m);
            int nV = ddg.VertexCount;

            var sScalars = new List<double>();
            var sGrads = new List<Vector3d>();
            foreach (object obj in rawS)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is double d) sScalars.Add(d);
                else if (val is int iv) sScalars.Add(iv);
                else if (val is Vector3d v) sGrads.Add(v);
            }

            double[] uPotential;
            double[] densityField = new double[nV];

            if (sScalars.Count > 0)
            {
                uPotential = ddg.MapScalarInputToVertices(sScalars);
                DdgAlgorithms.ComputeWalkOnMesh(ddg, uPotential, null, 0.0, 2, 0, out Vector3d[] gVec, out _, out _);

                double uMin = double.MaxValue, uMax = double.MinValue;
                double gMin = double.MaxValue, gMax = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    if (uPotential[i] < uMin) uMin = uPotential[i];
                    if (uPotential[i] > uMax) uMax = uPotential[i];
                    double gm = gVec[i].Length;
                    if (gm < gMin) gMin = gm;
                    if (gm > gMax) gMax = gm;
                }
                double uSpan = Math.Max(1e-12, uMax - uMin);
                double gSpan = Math.Max(1e-12, gMax - gMin);

                for (int i = 0; i < nV; i++)
                {
                    double tU = (uPotential[i] - uMin) / uSpan;
                    double tG = (gVec[i].Length - gMin) / gSpan;
                    densityField[i] = 0.70 * tU + 0.30 * tG;
                }
            }
            else if (sGrads.Count > 0)
            {
                Vector3d[] mappedG = ddg.MapVectorInputToVertices(sGrads);
                DdgAlgorithms.SmoothVectorAndGradientMultiMode(ddg, sGrads, null, 1.2, 1, out _, out _, out uPotential, out _);

                double gMin = double.MaxValue, gMax = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    double gm = mappedG[i].Length;
                    if (gm < gMin) gMin = gm;
                    if (gm > gMax) gMax = gm;
                }

                if (gMax - gMin > 1e-4 * gMax)
                {
                    double uMin = double.MaxValue, uMax = double.MinValue;
                    for (int i = 0; i < nV; i++)
                    {
                        if (uPotential[i] < uMin) uMin = uPotential[i];
                        if (uPotential[i] > uMax) uMax = uPotential[i];
                    }
                    double uSpan = Math.Max(1e-12, uMax - uMin);
                    double gSpan = Math.Max(1e-12, gMax - gMin);
                    for (int i = 0; i < nV; i++)
                    {
                        double tG = (mappedG[i].Length - gMin) / gSpan;
                        double tU = (uPotential[i] - uMin) / uSpan;
                        densityField[i] = 0.65 * tG + 0.35 * tU;
                    }
                }
                else
                {
                    for (int i = 0; i < nV; i++) densityField[i] = uPotential[i];
                }
            }
            else
            {
                uPotential = DdgAlgorithms.ComputeHeatDistance(ddg, new int[] { 0 }, 1.0, out _, out _);
                for (int i = 0; i < nV; i++) densityField[i] = uPotential[i];
            }

            DdgAlgorithms.ComputeWalkOnMesh(ddg, uPotential, null, 0.0, 2, 0, out Vector3d[] gradU, out _, out _);
            var vDir = new Vector3d[nV];
            for (int i = 0; i < nV; i++)
            {
                Vector3d t = Vector3d.CrossProduct(ddg.VertexNormals[i], gradU[i]);
                vDir[i] = t.Unitize() ? t : ddg.PrincipalDir[i];
            }

            // Use variable-wavenumber Knöppel stripes with densityField and touchEnds control!
            double[] wave = DdgAlgorithms.ComputeStripePatternsByDistance(
                ddg,
                vDir,
                null,
                baseDist,
                2,
                out List<Polyline> crvs,
                out List<Point3d> branches,
                densityField,
                touchEnds);

            _stripeCurves.AddRange(crvs);
            _stripedMesh = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, wave, true);

            double dMin = double.MaxValue, dMax = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                if (densityField[i] < dMin) dMin = densityField[i];
                if (densityField[i] > dMax) dMax = densityField[i];
            }
            double dSpan = Math.Max(1e-12, dMax - dMin);

            double baseArm = 0.45 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 700);
            for (int i = 0; i < ddg.VertexCount; i += stride)
            {
                Vector3d u = vDir[i];
                if (u.Unitize())
                {
                    Vector3d v = Vector3d.CrossProduct(ddg.VertexNormals[i], u);
                    v.Unitize();
                    double tD = (densityField[i] - dMin) / dSpan;
                    double arm = baseArm * (0.45 + 0.90 * tD);
                    Point3d p = ddg.Positions[i];
                    _xCrossSegments.Add(new Line(p - arm * u, p + arm * u));
                    _xCrossSegments.Add(new Line(p - arm * v, p + arm * v));
                }
            }

            DA.SetDataList(0, _stripeCurves);
            DA.SetDataList(1, _xCrossSegments);
            DA.SetDataList(2, branches);
            DA.SetData(3, _stripedMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            if (_xCrossSegments.Count > 0)
                args.Display.DrawLines(_xCrossSegments, Color.FromArgb(140, 255, 120, 80), 1);

            foreach (var pl in _stripeCurves)
                args.Display.DrawPolyline(pl, Color.FromArgb(0, 235, 255), 2);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000049");
    }

    public class GH_RotateTangentVectors : GH_Component
    {
        private readonly List<Line> _previewLines = new List<Line>();

        public GH_RotateTangentVectors()
          : base("Rotate Tangent Vectors", "RotateVec",
              "Rotates a Tangent Vector Field V on the mesh around each vertex normal by a uniform Angle (e.g. 90 deg) or a per-vertex Scalar Field of angles.",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("RotateVec");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxV = pManager.AddVectorParameter("Vectors", "V", "Input tangent vector field per vertex or face (leave empty for principal curvature directions)", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
            pManager.AddNumberParameter("Angle (Deg)", "A", "Rotation angle in degrees in the surface tangent plane (single angle like 90.0, or per-vertex scalar list)", GH_ParamAccess.list, 90.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Rotated Vectors", "VR", "Tangent vectors rotated by Angle A around each vertex normal", GH_ParamAccess.list);
            pManager.AddVectorParameter("Ortho Vectors", "VO", "Orthogonal tangent vectors (90 deg from VR)", GH_ParamAccess.list);
            pManager.AddLineParameter("Preview Segments", "L", "Lightweight 3D line segments of the rotated vector field", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _previewLines.Clear();

            Mesh m = null;
            var inVecs = new List<Vector3d>();
            var angles = new List<double>();

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, inVecs);
            if (!DA.GetDataList(2, angles) || angles.Count == 0) angles.Add(90.0);

            var ddg = new DdgMesh(m);
            Vector3d[] rotVecs = DdgAlgorithms.RotateTangentVectorsOnMesh(
                ddg,
                inVecs.Count > 0 ? inVecs : null,
                angles,
                out Vector3d[] orthoVecs,
                out List<Line> lines);

            _previewLines.AddRange(lines);

            DA.SetDataList(0, rotVecs);
            DA.SetDataList(1, orthoVecs);
            DA.SetDataList(2, lines);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _previewLines.Count == 0) return;

            Color cSeg = Attributes.Selected ? Color.FromArgb(255, 240, 70) : Color.FromArgb(255, 215, 50);
            args.Display.DrawLines(_previewLines, cSeg, 2);
        }

        public override Guid ComponentGuid => new Guid("A104B204-C304-4D04-9E04-000000000050");
    }
}
