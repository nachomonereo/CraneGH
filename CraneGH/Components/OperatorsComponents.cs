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
        private readonly List<Polyline> _isoLines = new List<Polyline>();
        private Point3d _minPt;
        private Point3d _maxPt;
        private double _minVal;
        private double _maxVal;
        private bool _showTags = true;

        public GH_MeshScalarDisplay()
          : base("Scalar Field Visualizer", "ScalarView",
              "Visualizes any scalar field on a mesh with scientific colormaps (0=Heat, 1=CoolWarm, 2=Viridis, 3=Turbo, 4=Zebra), iso-lines, and 3D Min/Max value tags.",
              "CraneGH", "01. Visualizers")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("ScalarView");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Scalar values per vertex or face", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Palette", "P", "Colormap: 0 = Crane Heat, 1 = CoolWarm Diverging, 2 = Viridis, 3 = Turbo Rainbow, 4 = Zebra Bands", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Iso-Lines", "N", "Number of live viewport iso-contour lines to overlay (0 = none)", GH_ParamAccess.item, 12);
            pManager.AddBooleanParameter("Show Value Tags", "T", "Display 3D Min / Max scalar value callouts in the viewport", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Colored Mesh", "CM", "Vertex-colored mesh with selected scientific palette", GH_ParamAccess.item);
            pManager.AddCurveParameter("Iso-Contours", "C", "Extracted iso-contour polylines", GH_ParamAccess.list);
            pManager.AddIntervalParameter("Scalar Domain", "D", "Domain [Min, Max] of the scalar field in real units", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _coloredMesh = null;
            _isoLines.Clear();
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            int palette = 0;
            int isoCount = 12;
            bool showTags = true;

            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;
            DA.GetData(2, ref palette);
            DA.GetData(3, ref isoCount);
            DA.GetData(4, ref showTags);

            _showTags = showTags;
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

            if (isoCount > 0)
            {
                var levels = new List<double>(isoCount);
                for (int k = 1; k <= isoCount; k++)
                {
                    double frac = (double)k / (isoCount + 1);
                    levels.Add(_minVal + frac * (_maxVal - _minVal));
                }
                _isoLines.AddRange(ddg.ExtractIsoContours(u, levels));
            }

            DA.SetData(0, _coloredMesh);
            DA.SetDataList(1, _isoLines);
            DA.SetData(2, new Interval(_minVal, _maxVal));
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _coloredMesh == null) return;

            Color isoCol = Color.FromArgb(200, 25, 25, 30);
            foreach (var pl in _isoLines)
                args.Display.DrawPolyline(pl, isoCol, 1);

            if (_showTags)
            {
                args.Display.DrawDot(_minPt, $"Min: {_minVal:0.###}", Color.FromArgb(40, 90, 200), Color.White);
                args.Display.DrawDot(_maxPt, $"Max: {_maxVal:0.###}", Color.FromArgb(215, 50, 40), Color.White);
            }
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
}
