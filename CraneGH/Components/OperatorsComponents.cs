using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_DiscreteCurvatures : GH_Component
    {
        public GH_DiscreteCurvatures()
          : base("Discrete Curvatures", "Curvatures",
              "Computes Gaussian Curvature (K), Mean Curvature (H), and Principal Curvature Direction (D) per mesh vertex.",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("Curvatures");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Gaussian Curvature", "K", "Integrated discrete Gaussian curvature (angle defect) per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Mean Curvature", "H", "Discrete Mean Curvature per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Principal Direction", "D", "Estimated principal curvature tangent direction per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Dual Area", "A", "Barycentric dual cell area per vertex (Hodge Star 0)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Welded Mesh", "WM", "Topologically welded triangle mesh matching output vertex count", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            if (!DA.GetData(0, ref m) || m == null) return;

            var ddg = new DdgMesh(m);
            DA.SetDataList(0, ddg.GaussianCurvature);
            DA.SetDataList(1, ddg.MeanCurvature);
            DA.SetDataList(2, ddg.PrincipalDir);
            DA.SetDataList(3, ddg.DualAreas);
            DA.SetData(4, ddg.RhinoMesh);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000001");
    }

    public class GH_MeanCurvatureFlow : GH_Component
    {
        public GH_MeanCurvatureFlow()
          : base("Mean Curvature Mesh Smooth", "MCFlow",
              "Smooths 3D mesh geometry using implicit Cotan-Laplace Mean Curvature Flow.",
              "CraneGH", "04. Segmentation & Fabrication")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("MCFlow");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step", "t", "Implicit flow time step (relative to squared edge length)", GH_ParamAccess.item, 1.0);
            pManager.AddBooleanParameter("Fix Boundary", "B", "Pin naked boundary vertices", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Smoothed Mesh", "M", "Mesh after implicit Mean Curvature Flow", GH_ParamAccess.item);
            pManager.AddVectorParameter("Displacement", "V", "Vertex displacement vectors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double tStep = 1.0;
            bool fixB = true;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref tStep);
            DA.GetData(2, ref fixB);

            var ddg = new DdgMesh(m);
            int n = ddg.VertexCount;
            double dt = Math.Max(0.0, tStep) * ddg.MeanEdgeLength * ddg.MeanEdgeLength;

            var builder = ddg.BuildCotanLaplacianBuilder(dt);
            double[] rhsX = new double[n];
            double[] rhsY = new double[n];
            double[] rhsZ = new double[n];

            for (int i = 0; i < n; i++)
            {
                if (fixB && ddg.VertexOnBoundary[i])
                {
                    builder.PinRow(i, 1.0);
                    rhsX[i] = ddg.Positions[i].X;
                    rhsY[i] = ddg.Positions[i].Y;
                    rhsZ[i] = ddg.Positions[i].Z;
                }
                else
                {
                    double mass = ddg.DualAreas[i];
                    builder.Add(i, i, mass);
                    rhsX[i] = mass * ddg.Positions[i].X;
                    rhsY[i] = mass * ddg.Positions[i].Y;
                    rhsZ[i] = mass * ddg.Positions[i].Z;
                }
            }

            CsrMatrix mat = builder.Build();
            double[] nx = SparseLinearSolver.SolvePCG(mat, rhsX, 800, 1e-7);
            double[] ny = SparseLinearSolver.SolvePCG(mat, rhsY, 800, 1e-7);
            double[] nz = SparseLinearSolver.SolvePCG(mat, rhsZ, 800, 1e-7);

            Mesh outMesh = ddg.RhinoMesh.DuplicateMesh();
            var disp = new Vector3d[n];
            for (int i = 0; i < n; i++)
            {
                var newPt = new Point3d(nx[i], ny[i], nz[i]);
                disp[i] = newPt - ddg.Positions[i];
                outMesh.Vertices.SetVertex(i, newPt);
            }
            outMesh.Normals.ComputeNormals();

            DA.SetData(0, outMesh);
            DA.SetDataList(1, disp);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000002");
    }

    public class GH_ScalarFieldCalculus : GH_Component
    {
        public GH_ScalarFieldCalculus()
          : base("Scalar Field Calculus", "GradLap",
              "Computes Intrinsic Surface Gradient (per vertex and per face) and Cotan-Laplacian of any scalar field u defined on mesh vertices.",
              "CraneGH", "01. Operators & Display")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("GradLap");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Scalar value per mesh vertex", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vertex Gradient", "GV", "Intrinsic tangent gradient vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Face Gradient", "GF", "Intrinsic tangent gradient vector per triangle face", GH_ParamAccess.list);
            pManager.AddNumberParameter("Laplacian", "Lu", "Cotan-Laplacian (Lc * u) per vertex", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;

            var ddg = new DdgMesh(m);
            double[] u = ddg.MapScalarInputToVertices(uList);

            ddg.ComputeGradient(u, out Vector3d[] faceGrad, out Vector3d[] vertGrad);
            CsrMatrix lap = ddg.BuildCotanLaplacianBuilder(1.0).Build();
            double[] lu = new double[ddg.VertexCount];
            lap.Multiply(u, lu);

            DA.SetDataList(0, vertGrad);
            DA.SetDataList(1, faceGrad);
            DA.SetDataList(2, lu);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000003");
    }

    public class GH_HarmonicScalarSolver : GH_Component
    {
        public GH_HarmonicScalarSolver()
          : base("Harmonic Scalar Field", "Harmonic",
              "Solves the Laplace Dirichlet problem (Lc * u = 0) on a mesh to smoothly interpolate scalar values pinned at constraint points.",
              "CraneGH", "01. Operators & Display")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("Harmonic");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Constraint Points", "P", "Points on the mesh where values are fixed", GH_ParamAccess.list);
            pManager.AddNumberParameter("Values", "V", "Scalar values at the constraint points", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Scalar Field", "u", "Harmonic interpolated scalar per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by harmonic field", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var pts = new List<Point3d>();
            var vals = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, pts) || pts.Count == 0) return;
            if (!DA.GetDataList(2, vals) || vals.Count == 0) return;

            var ddg = new DdgMesh(m);
            int n = ddg.VertexCount;
            var builder = ddg.BuildCotanLaplacianBuilder(1.0);
            double[] rhs = new double[n];

            const double penalty = 1e7;
            for (int k = 0; k < pts.Count; k++)
            {
                int vIdx = ddg.FindClosestVertex(pts[k]);
                double val = vals[Math.Min(k, vals.Count - 1)];
                builder.Add(vIdx, vIdx, penalty);
                rhs[vIdx] += penalty * val;
            }

            double[] u = SparseLinearSolver.SolvePCG(builder.Build(), rhs, 1500, 1e-8);
            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, u, false);

            DA.SetDataList(0, u);
            DA.SetData(1, colored);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000004");
    }

    public class GH_ScalarFieldSmooth : GH_Component
    {
        public GH_ScalarFieldSmooth()
          : base("Smooth Scalar Field", "SmoothScalar",
              "Smooths any scalar field on a mesh via macro-scale Cotan-Laplace diffusion while preserving its original [Min, Max] range.",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("SmoothScalar");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "S", "Input scalar values per vertex or per face (e.g. K or H from Curvatures)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Smoothness", "T", "Diffusion smoothing strength (0 = raw, 1..10 = smooth)", GH_ParamAccess.item, 2.5);
            int idxB = pManager.AddNumberParameter("Crease Barrier", "B", "Optional sharp-edge preservation weight (0 = isotropic smooth, >0 = stops smoothing across sharp edges)", GH_ParamAccess.item, 0.0);
            pManager[idxB].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Smoothed Scalar", "S", "Clean, smoothed scalar field per vertex (ready for MeshSeg, VarCuts, IsoCrvs, SpinXForm)", GH_ParamAccess.list);
            pManager.AddVectorParameter("Face Gradients", "G", "Smoothed tangent gradient vectors per face", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the smoothed scalar field", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var rawScalars = new List<double>();
            double smoothness = 2.5;
            double barrier = 0.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, rawScalars) || rawScalars.Count == 0) return;
            DA.GetData(2, ref smoothness);
            DA.GetData(3, ref barrier);

            var ddg = new DdgMesh(m);
            double[] uSmooth = DdgAlgorithms.SmoothScalarField(
                ddg, rawScalars, smoothness, barrier, out Vector3d[] faceGrads, out Mesh colored);

            DA.SetDataList(0, uSmooth);
            DA.SetDataList(1, faceGrads);
            DA.SetData(2, colored);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000029");
    }

    public class GH_WalkOnMeshGradient : GH_Component
    {
        private readonly List<Polyline> _walkPaths = new List<Polyline>();
        private readonly List<Line> _gradArrows = new List<Line>();

        public GH_WalkOnMeshGradient()
          : base("Walk on Mesh & Gradient", "WalkOnMesh",
              "Computes the intrinsic surface gradient of a scalar field and traces paths along Steepest Descent (0), Ascent (1), Iso-Contour (2), or Stochastic Walk (3).",
              "CraneGH", "03. Vector Fields & Stripes")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("WalkOnMesh");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Scalar value per vertex (custom numbers, real distances, curvature, etc.)", GH_ParamAccess.list);
            int idxP = pManager.AddPointParameter("Start Points", "P", "Optional seed points on the mesh to start the walk (if empty, samples automatically)", GH_ParamAccess.list);
            pManager[idxP].Optional = true;
            pManager.AddIntegerParameter("Walk Mode", "W", "0 = Steepest Descent (-grad u), 1 = Steepest Ascent (+grad u), 2 = Iso-Contour Walk (orthogonal), 3 = Stochastic Walk on Mesh", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Walk Paths", "C", "Walk trajectory polylines on the mesh", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vertex Gradient", "GV", "Intrinsic scalar gradient vector per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Face Gradient", "GF", "Intrinsic scalar gradient vector per triangle face", GH_ParamAccess.list);
            pManager.AddNumberParameter("Gradient Magnitude", "GM", "Slope / magnitude ||grad u|| per vertex", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _walkPaths.Clear();
            _gradArrows.Clear();
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 }, new int[] { 2 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            var seeds = new List<Point3d>();
            int mode = 0;

            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;
            DA.GetDataList(2, seeds);
            DA.GetData(3, ref mode);

            var ddg = new DdgMesh(m);
            List<Polyline> paths = DdgAlgorithms.ComputeWalkOnMesh(
                ddg, uList, seeds, 0.0, 150, mode,
                out Vector3d[] vGrad, out Vector3d[] fGrad, out _);

            _walkPaths.AddRange(paths);

            double[] gradMag = new double[ddg.VertexCount];
            double arrowScale = 0.65 * ddg.MeanEdgeLength;
            int stride = Math.Max(1, ddg.VertexCount / 400);
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                gradMag[i] = vGrad[i].Length;
                if (i % stride == 0 && gradMag[i] > 1e-12)
                {
                    Vector3d dir = vGrad[i] / gradMag[i];
                    _gradArrows.Add(new Line(ddg.Positions[i], ddg.Positions[i] + arrowScale * dir));
                }
            }

            DA.SetDataList(0, paths);
            DA.SetDataList(1, vGrad);
            DA.SetDataList(2, fGrad);
            DA.SetDataList(3, gradMag);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            Color cGrad = Color.FromArgb(140, 0, 215, 255);
            if (_gradArrows.Count > 0)
                args.Display.DrawLines(_gradArrows, cGrad, 2);

            Color cPath = Attributes.Selected ? Color.FromArgb(255, 255, 80) : Color.FromArgb(255, 195, 40);
            foreach (var pl in _walkPaths)
                args.Display.DrawPolyline(pl, cPath, 3);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000033");
    }

    public class GH_MeshScalarDisplay : GH_Component
    {
        private Mesh _coloredMesh;
        private Point3d _minPt;
        private Point3d _maxPt;
        private double _minVal;
        private double _maxVal;

        public GH_MeshScalarDisplay()
          : base("Scalar Field Visualizer", "ScalarView",
              "Visualizes any scalar field on a mesh with scientific colormaps (0=Heat, 1=CoolWarm, 2=Viridis, 3=Turbo, 4=Zebra) and 3D Min/Max value tags (no segmentation/iso-polylines).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("ScalarView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "S", "Scalar values per vertex or face", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Palette", "P", "Colormap: 0 = Crane Heat, 1 = CoolWarm Diverging, 2 = Viridis, 3 = Turbo Rainbow, 4 = Zebra Bands", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Colored Mesh", "CM", "Vertex-colored mesh with selected scientific palette", GH_ParamAccess.item);
            pManager.AddIntervalParameter("Scalar Domain", "D", "Domain [Min, Max] of the scalar field in real units", GH_ParamAccess.item);
            pManager.AddPointParameter("Min / Max Points", "P", "[MinPoint, MaxPoint] locations on the mesh", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _coloredMesh = null;
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            int palette = 0;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, uList);
            DA.GetData(2, ref palette);

            var ddg = new DdgMesh(m);
            double[] u = ddg.MapScalarInputToVertices(uList);

            _minVal = double.MaxValue;
            _maxVal = double.MinValue;
            int minIdx = 0, maxIdx = 0;
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                if (u[i] < _minVal) { _minVal = u[i]; minIdx = i; }
                if (u[i] > _maxVal) { _maxVal = u[i]; maxIdx = i; }
            }
            _minPt = ddg.Positions[minIdx];
            _maxPt = ddg.Positions[maxIdx];

            _coloredMesh = DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, u, palette);

            DA.SetData(0, _coloredMesh);
            DA.SetData(1, new Interval(_minVal, _maxVal));
            DA.SetDataList(2, new Point3d[] { _minPt, _maxPt });
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _coloredMesh == null) return;

            args.Display.DrawDot(_minPt, $"Min: {_minVal:0.###}", Color.FromArgb(40, 90, 200), Color.White);
            args.Display.DrawDot(_maxPt, $"Max: {_maxVal:0.###}", Color.FromArgb(215, 50, 40), Color.White);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000032");
    }

    public class GH_ScalarGradientVisualizer : GH_Component
    {
        private readonly List<Line> _gradLines = new List<Line>();
        private readonly List<Line> _wavefrontTicks = new List<Line>();
        private Mesh _slopeMesh;

        public GH_ScalarGradientVisualizer()
          : base("Scalar Gradient Visualizer", "GradView",
              "Computes and visualizes the intrinsic Geodesic / Scalar Gradient (grad S) and orthogonal iso-wavefront ticks using lightweight GPU segments, coloring the mesh by slope ||grad S||.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GradView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "S", "Scalar values per vertex or face", GH_ParamAccess.list);
            pManager.AddNumberParameter("Segment Scale", "L", "Visual segment length multiplier relative to average mesh edge length", GH_ParamAccess.item, 0.75);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Gradient Vectors", "G", "Intrinsic scalar gradient vector per vertex", GH_ParamAccess.list);
            pManager.AddLineParameter("Gradient Segments", "L", "3D directional gradient line segments", GH_ParamAccess.list);
            pManager.AddMeshParameter("Slope Mesh", "CM", "Mesh colored by gradient slope magnitude ||grad S||", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _gradLines.Clear();
            _wavefrontTicks.Clear();
            _slopeMesh = null;
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var sList = new List<double>();
            double scale = 0.75;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, sList) || sList.Count == 0) return;
            DA.GetData(2, ref scale);

            var ddg = new DdgMesh(m);
            double[] u = ddg.MapScalarInputToVertices(sList);
            ddg.ComputeGradient(u, out _, out Vector3d[] vGrad);

            DdgAlgorithms.ComputeVectorFieldGradientsAndGlyphs(
                ddg, null, u, scale, 2,
                out List<Line> primLines,
                out List<Line> orthoLines,
                out _, out _, out _,
                out _slopeMesh,
                out _);

            _gradLines.AddRange(primLines);
            _wavefrontTicks.AddRange(orthoLines);

            DA.SetDataList(0, vGrad);
            DA.SetDataList(1, primLines);
            DA.SetData(2, _slopeMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _gradLines.Count == 0) return;

            Color cPrim = Attributes.Selected ? Color.FromArgb(255, 240, 70) : Color.FromArgb(0, 235, 255);
            args.Display.DrawLines(_gradLines, cPrim, 2);
            if (_wavefrontTicks.Count > 0)
                args.Display.DrawLines(_wavefrontTicks, Color.FromArgb(180, 255, 230, 140), 1);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000051");
    }

    public class GH_CrossFieldVisualizer : GH_Component
    {
        private readonly List<Line> _uLines = new List<Line>();
        private readonly List<Line> _vLines = new List<Line>();
        private Mesh _analysisMesh;

        public GH_CrossFieldVisualizer()
          : base("X-Cross & Div/Curl Visualizer", "CrossView",
              "Visualizes any Vector Field V as lightweight 4-RoSy X-Crosses and computes its Divergence (div V), Rotor/Curl (curl V), and Covariant Vector Gradient.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("CrossView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxV = pManager.AddVectorParameter("Vectors", "V", "Tangent vector field per vertex or face (leave empty for Principal Curvature X-Crosses)", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
            pManager.AddNumberParameter("Cross Scale", "L", "Visual scale multiplier for the X-Cross segments", GH_ParamAccess.item, 0.70);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("X-Cross Lines", "Crx", "Lightweight 4-way X-cross line segments", GH_ParamAccess.list);
            pManager.AddNumberParameter("Divergence", "Div", "Discrete Divergence (div V) per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Curl / Rotor", "Curl", "Discrete Curl ((nabla x V) . N) per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Analysis Mesh", "CM", "Mesh colored by local Curl / Rotation intensity", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _uLines.Clear();
            _vLines.Clear();
            _analysisMesh = null;
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var inVecs = new List<Vector3d>();
            double scale = 0.70;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, inVecs);
            DA.GetData(2, ref scale);

            var ddg = new DdgMesh(m);
            DdgAlgorithms.ComputeVectorFieldGradientsAndGlyphs(
                ddg,
                inVecs.Count > 0 ? inVecs : null,
                null,
                scale,
                1,
                out List<Line> primLines,
                out List<Line> orthoLines,
                out _,
                out double[] div,
                out double[] curl,
                out _analysisMesh,
                out _);

            _uLines.AddRange(primLines);
            _vLines.AddRange(orthoLines);

            var allCross = new List<Line>(primLines.Count + orthoLines.Count);
            allCross.AddRange(primLines);
            allCross.AddRange(orthoLines);

            DA.SetDataList(0, allCross);
            DA.SetDataList(1, div);
            DA.SetDataList(2, curl);
            DA.SetData(3, _analysisMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _uLines.Count == 0) return;

            Color cU = Attributes.Selected ? Color.FromArgb(255, 240, 70) : Color.FromArgb(0, 230, 255);
            Color cV = Attributes.Selected ? Color.FromArgb(255, 160, 60) : Color.FromArgb(255, 95, 85);
            args.Display.DrawLines(_uLines, cU, 2);
            if (_vLines.Count > 0)
                args.Display.DrawLines(_vLines, cV, 2);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000052");
    }

    public class GH_StandardTestMesh : GH_Component
    {
        public GH_StandardTestMesh()
          : base("Area-Normalized Test Mesh", "TestMesh",
              "Generates built-in benchmark 3D test meshes OR normalizes any custom mesh so all meshes have the exact same Surface Area (default = 1000.0) for fair side-by-side comparison, plus a Test Curve and Test Points on the surface.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("TestMesh");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Custom Mesh", "M", "Optional custom mesh to normalize to Target Area A (leave empty to generate built-in Preset Shape P)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddIntegerParameter("Preset Shape", "P", "Built-in test mesh when M is empty: 0 = Saddle-Dome + Crease + Naked Edge, 1 = 3-Lobe Branching Surface, 2 = CAD Fandisk Crease Block, 3 = Genus-1 Torus", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Target Area", "A", "Exact target surface area to normalize the mesh to (so all test meshes have identical area!)", GH_ParamAccess.item, 1000.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Normalized Mesh", "M", "3D mesh centered at origin with exact Surface Area = A", GH_ParamAccess.item);
            pManager.AddCurveParameter("Test Curve", "TC", "Smooth curve lying directly on the test mesh (ready to plug into HeatDist, CrvField, SurfWinding!)", GH_ParamAccess.item);
            pManager.AddPointParameter("Test Points", "TP", "Pair of test points [A, B] on the mesh (ready to plug into HeatDist or GeoAB!)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Surface Area", "A", "Verified total surface area of the output mesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh customM = null;
            int preset = 0;
            double targetArea = 1000.0;

            DA.GetData(0, ref customM);
            DA.GetData(1, ref preset);
            DA.GetData(2, ref targetArea);

            Mesh normMesh = DdgAlgorithms.CreateAreaNormalizedTestMesh(
                customM, preset, targetArea,
                out Curve testCrv,
                out List<Point3d> testPts,
                out double finalArea);

            DA.SetData(0, normMesh);
            DA.SetData(1, testCrv);
            DA.SetDataList(2, testPts);
            DA.SetData(3, finalArea);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000069");
    }

    public class GH_MeshQualityTopologyAnalyzer : GH_Component
    {
        public GH_MeshQualityTopologyAnalyzer()
          : base("Mesh Area, Quality & Topology Analyzer", "MeshAnalyze",
              "Analyzes mesh Surface Area, Edge Lengths, Aspect Ratio Quality, Dihedral Creases, Valence, and Topology (pure visual + numerical analyzer without iso-curves).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("MeshAnalyze");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddIntegerParameter("Metric Mode", "Mode", "0 = Vertex Dual Area, 1 = Triangle Aspect Ratio Quality [0..1], 2 = Max Dihedral Crease Angle (Deg), 3 = Vertex Valence", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Total Area", "A", "Exact total surface area of the mesh", GH_ParamAccess.item);
            pManager.AddIntervalParameter("Edge Domain", "E", "Domain [MinEdge, MaxEdge] of mesh edge lengths", GH_ParamAccess.item);
            pManager.AddTextParameter("Topology Info", "I", "Detailed summary of Vertices, Faces, Area, Naked Edges, Sharp Creases, Shells, and Euler Characteristic", GH_ParamAccess.item);
            pManager.AddNumberParameter("Metric Scalar", "S", "Per-vertex scalar values of the selected Metric Mode", GH_ParamAccess.list);
            pManager.AddMeshParameter("Analysis Mesh", "CM", "False-color analysis mesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            int mode = 1;
            DA.GetData(0, ref m);
            DA.GetData(1, ref mode);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 0, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            double[] metric = DdgAlgorithms.ComputeMeshQualityAndTopologyAnalysis(
                ddg, mode,
                out double area,
                out Interval edgeDom,
                out string report,
                out Mesh coloredMesh);

            DA.SetData(0, area);
            DA.SetData(1, edgeDom);
            DA.SetData(2, report);
            DA.SetDataList(3, metric);
            DA.SetData(4, coloredMesh);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000070");
    }

    public class GH_MeshCreaseConcavityAnalyzer : GH_Component
    {
        public GH_MeshCreaseConcavityAnalyzer()
          : base("Mesh Crease, Concavity & Anisotropy Analyzer", "CreaseAnalyze",
              "Analyzes signed Dihedral Angles (- = Concave Valley, + = Convex Ridge), Crease Intensity [0..1], and Curvature Anisotropy Tau [0..1] per vertex.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("CreaseAnalyze");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized CAD Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddNumberParameter("Crease Threshold (Deg)", "A", "Reference dihedral angle in degrees for crease intensity normalization", GH_ParamAccess.item, 25.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Signed Dihedral (Deg)", "SD", "Signed dihedral angle in degrees per vertex (Negative = Concave Valley, Positive = Convex Ridge)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Crease Intensity", "CI", "Normalized [0..1] crease salience per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Curvature Anisotropy", "Tau", "Relative principal curvature anisotropy (|Kmax| - |Kmin|) / |Kmax| in [0..1]", GH_ParamAccess.list);
            pManager.AddMeshParameter("Analysis Mesh", "CM", "False-color mesh (Blue = Concave Valley, White = Smooth, Red = Convex Crease Ridge)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double thresh = 25.0;
            DA.GetData(0, ref m);
            DA.GetData(1, ref thresh);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 2, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            double[] signedDihedral = DdgAlgorithms.ComputeMeshCreaseAndConcavityAnalysis(
                ddg, thresh,
                out double[] creaseSalience,
                out double[] anisotropyTau,
                out Mesh coloredMesh);

            DA.SetDataList(0, signedDihedral);
            DA.SetDataList(1, creaseSalience);
            DA.SetDataList(2, anisotropyTau);
            DA.SetData(3, coloredMesh);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000071");
    }

    public class GH_MeshDraftUndercutAnalyzer : GH_Component
    {
        public GH_MeshDraftUndercutAnalyzer()
          : base("Mesh Mold Draft & Undercut Analyzer", "DraftAnalyze",
              "Analyzes per-vertex Mold Draft Angles [-90..+90 deg] and Undercut Severity [0..1] relative to a Pull Direction D (pure visual + scalar analyzer without cutting polylines).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("DraftAnalyze");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddVectorParameter("Pull Direction", "D", "Mold extraction / pull direction vector", GH_ParamAccess.item, Vector3d.ZAxis);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Draft Angle (Deg)", "Deg", "Draft angle in degrees [-90..+90] per vertex relative to Pull Direction D", GH_ParamAccess.list);
            pManager.AddNumberParameter("Undercut Severity", "U", "Undercut severity [0..1] per vertex (0 = safe draft >= 3 deg, 1 = severe undercut)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Analysis Mesh", "CM", "False-color draft angle mesh (Blue = Undercut, White = 0 deg Vertical Wall, Red = Positive Draft)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            Vector3d pullDir = Vector3d.ZAxis;
            DA.GetData(0, ref m);
            DA.GetData(1, ref pullDir);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 0, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            double[] draftDeg = DdgAlgorithms.ComputeMeshDraftAndUndercutAnalysis(
                ddg, pullDir,
                out double[] undercutSev,
                out Mesh coloredMesh);

            DA.SetDataList(0, draftDeg);
            DA.SetDataList(1, undercutSev);
            DA.SetData(2, coloredMesh);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000072");
    }

    public class GH_GaussianCurvatureVisualizer : GH_Component
    {
        private Mesh _coloredMesh;
        private Point3d _minPt;
        private Point3d _maxPt;
        private double _minVal;
        private double _maxVal;

        public GH_GaussianCurvatureVisualizer()
          : base("Gaussian Curvature Visualizer", "GaussView",
              "Dedicated visualizer for Discrete Gaussian Curvature K = K1 * K2 (Blue = Hyperbolic Saddle K < 0, White = Developable K = 0, Red = Elliptic Dome/Bowl K > 0).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GaussView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddNumberParameter("Smoothness", "S", "Cotan-Laplace smoothing strength for Gaussian curvature density", GH_ParamAccess.item, 1.5);
            pManager.AddIntegerParameter("Palette", "P", "Colormap: 0 = Heat, 1 = CoolWarm Diverging (K=0 White), 2 = Viridis, 3 = Turbo", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Gaussian Curvature", "K", "Area-normalized Gaussian curvature K per vertex", GH_ParamAccess.list);
            pManager.AddIntervalParameter("Curvature Domain", "D", "Domain [Min K, Max K]", GH_ParamAccess.item);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by Gaussian Curvature K", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double smoothness = 1.5;
            int palette = 1;
            DA.GetData(0, ref m);
            DA.GetData(1, ref smoothness);
            DA.GetData(2, ref palette);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 0, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            DdgAlgorithms.ComputeSeparateCurvatureFields(
                ddg, smoothness,
                out double[] K, out _, out _, out _, out _, out _, out _, out _);

            _minVal = double.MaxValue;
            _maxVal = double.MinValue;
            int minIdx = 0, maxIdx = 0;
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                if (K[i] < _minVal) { _minVal = K[i]; minIdx = i; }
                if (K[i] > _maxVal) { _maxVal = K[i]; maxIdx = i; }
            }
            _minPt = ddg.Positions[minIdx];
            _maxPt = ddg.Positions[maxIdx];

            double maxAbs = Math.Max(1e-9, Math.Max(Math.Abs(_minVal), Math.Abs(_maxVal)));
            _coloredMesh = (palette == 1)
                ? DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, K, 1, -maxAbs, maxAbs)
                : DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, K, palette);

            DA.SetDataList(0, K);
            DA.SetData(1, new Interval(_minVal, _maxVal));
            DA.SetData(2, _coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _coloredMesh == null) return;

            args.Display.DrawDot(_minPt, $"K Min: {_minVal:0.####}", Color.FromArgb(40, 90, 200), Color.White);
            args.Display.DrawDot(_maxPt, $"K Max: {_maxVal:0.####}", Color.FromArgb(215, 50, 40), Color.White);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000073");
    }

    public class GH_MeanCurvatureVisualizer : GH_Component
    {
        private Mesh _coloredMesh;
        private Point3d _minPt;
        private Point3d _maxPt;
        private double _minVal;
        private double _maxVal;

        public GH_MeanCurvatureVisualizer()
          : base("Mean Curvature Visualizer", "MeanView",
              "Dedicated visualizer for Discrete Mean Curvature H = (K1 + K2)/2 (Blue = Concave Valley H < 0, White = Minimal/Flat H = 0, Red = Convex Ridge H > 0).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("MeanView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddNumberParameter("Smoothness", "S", "Cotan-Laplace smoothing strength for Mean curvature H", GH_ParamAccess.item, 1.5);
            pManager.AddIntegerParameter("Palette", "P", "Colormap: 0 = Heat, 1 = CoolWarm Diverging (H=0 White), 2 = Viridis, 3 = Turbo", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Mean Curvature", "H", "Discrete Mean Curvature H per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Mean Curvature Normals", "HN", "Mean curvature normal vector H * N per vertex", GH_ParamAccess.list);
            pManager.AddIntervalParameter("Curvature Domain", "D", "Domain [Min H, Max H]", GH_ParamAccess.item);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by Mean Curvature H", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double smoothness = 1.5;
            int palette = 1;
            DA.GetData(0, ref m);
            DA.GetData(1, ref smoothness);
            DA.GetData(2, ref palette);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 0, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            DdgAlgorithms.ComputeSeparateCurvatureFields(
                ddg, smoothness,
                out _, out double[] H, out _, out _, out _, out _, out _, out _);

            _minVal = double.MaxValue;
            _maxVal = double.MinValue;
            int minIdx = 0, maxIdx = 0;
            var hnVecs = new Vector3d[ddg.VertexCount];
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                if (H[i] < _minVal) { _minVal = H[i]; minIdx = i; }
                if (H[i] > _maxVal) { _maxVal = H[i]; maxIdx = i; }
                hnVecs[i] = H[i] * ddg.VertexNormals[i];
            }
            _minPt = ddg.Positions[minIdx];
            _maxPt = ddg.Positions[maxIdx];

            double maxAbs = Math.Max(1e-9, Math.Max(Math.Abs(_minVal), Math.Abs(_maxVal)));
            _coloredMesh = (palette == 1)
                ? DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, H, 1, -maxAbs, maxAbs)
                : DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, H, palette);

            DA.SetDataList(0, H);
            DA.SetDataList(1, hnVecs);
            DA.SetData(2, new Interval(_minVal, _maxVal));
            DA.SetData(3, _coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _coloredMesh == null) return;

            args.Display.DrawDot(_minPt, $"H Min: {_minVal:0.####}", Color.FromArgb(40, 90, 200), Color.White);
            args.Display.DrawDot(_maxPt, $"H Max: {_maxVal:0.####}", Color.FromArgb(215, 50, 40), Color.White);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000074");
    }

    public class GH_PrincipalCurvatureVisualizer : GH_Component
    {
        private readonly List<Line> _d1Lines = new List<Line>();
        private readonly List<Line> _d2Lines = new List<Line>();

        public GH_PrincipalCurvatureVisualizer()
          : base("Principal Curvatures (K1 & K2) Visualizer", "PrinCurvView",
              "Dedicated visualizer for Maximum (K1) and Minimum (K2) Principal Curvatures, Curvedness, Shape Index, and Principal Curvature Directions (D1, D2).",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("PrinCurvView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            int idxM = pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (if empty, uses Area-Normalized Test Mesh)", GH_ParamAccess.item);
            pManager[idxM].Optional = true;
            pManager.AddIntegerParameter("Metric Mode", "Mode", "0 = Max Principal Curvature K1, 1 = Min Principal Curvature K2, 2 = Curvedness sqrt((K1^2+K2^2)/2), 3 = Shape Index [-1..+1]", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Direction Scale", "L", "Visual scale of Principal Direction segments D1 (Cyan) and D2 (Coral) (set 0 to hide ticks)", GH_ParamAccess.item, 0.55);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Max Curvature", "K1", "Maximum principal curvature K1 per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Min Curvature", "K2", "Minimum principal curvature K2 per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Max Direction", "D1", "Combed maximum principal curvature tangent vector D1 per vertex", GH_ParamAccess.list);
            pManager.AddVectorParameter("Min Direction", "D2", "Combed minimum principal curvature tangent vector D2 per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the selected Principal Curvature metric", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            if (DA.Iteration == 0)
            {
                _d1Lines.Clear();
                _d2Lines.Clear();
            }

            Mesh m = null;
            int mode = 0;
            double scale = 0.55;
            DA.GetData(0, ref m);
            DA.GetData(1, ref mode);
            DA.GetData(2, ref scale);
            if (m == null || m.Vertices.Count < 3)
                m = DdgAlgorithms.CreateAreaNormalizedTestMesh(null, 0, 1000.0, out _, out _, out _);

            var ddg = new DdgMesh(m);
            DdgAlgorithms.ComputeSeparateCurvatureFields(
                ddg, 1.5,
                out _, out _,
                out double[] k1,
                out double[] k2,
                out double[] curvedness,
                out double[] shapeIndex,
                out Vector3d[] d1,
                out Vector3d[] d2);

            double[] activeMetric = mode == 1 ? k2 : (mode == 2 ? curvedness : (mode == 3 ? shapeIndex : k1));
            Mesh coloredMesh = DdgAlgorithms.ColorMeshByPalette(ddg.RhinoMesh, activeMetric, mode == 2 ? 0 : 1);

            if (scale > 1e-3)
            {
                double arm = 0.45 * scale * ddg.MeanEdgeLength;
                int stride = Math.Max(1, ddg.VertexCount / 900);
                for (int i = 0; i < ddg.VertexCount; i += stride)
                {
                    Point3d p = ddg.Positions[i];
                    _d1Lines.Add(new Line(p - arm * d1[i], p + arm * d1[i]));
                    _d2Lines.Add(new Line(p - arm * d2[i], p + arm * d2[i]));
                }
            }

            DA.SetDataList(0, k1);
            DA.SetDataList(1, k2);
            DA.SetDataList(2, d1);
            DA.SetDataList(3, d2);
            DA.SetData(4, coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _d1Lines.Count == 0) return;

            Color c1 = Attributes.Selected ? Color.FromArgb(255, 240, 70) : Color.FromArgb(0, 225, 255);
            Color c2 = Attributes.Selected ? Color.FromArgb(255, 160, 60) : Color.FromArgb(255, 95, 80);
            args.Display.DrawLines(_d1Lines, c1, 2);
            args.Display.DrawLines(_d2Lines, c2, 1);
        }

        public override Guid ComponentGuid => new Guid("A101B201-C301-4D01-9E01-000000000075");
    }
}
