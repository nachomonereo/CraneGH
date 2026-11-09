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
              "Computes Discrete Gaussian Curvature (angle defect), Mean Curvature (Cotan-Laplace), Principal Curvature Direction, and Dual Area per mesh vertex.",
              "CraneGH", "01. Operators & Display")
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
          : base("Mean Curvature Smooth", "MCFlow",
              "Implicit Mean Curvature Flow (M + t*Lc) X = M*X0 using the Discrete Cotan-Laplacian.",
              "CraneGH", "01. Operators & Display")
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
          : base("Scalar Field Smooth", "SmoothScalar",
              "Smooths any noisy scalar field (such as Gaussian Curvature K, Mean Curvature H, or custom weights) using implicit Cotan-Laplace diffusion with optional sharp-crease preservation.",
              "CraneGH", "01. Operators & Display")
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
          : base("Walk on Mesh & Scalar Gradient", "WalkOnMesh",
              "Computes the intrinsic surface gradient of any scalar field and walks on the mesh from seed points along Steepest Descent (-grad u), Ascent (+grad u), Iso-Contour (ortho), or Stochastic Geodesic Walk.",
              "CraneGH", "01. Operators & Display")
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

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _walkPaths.Clear();
            _gradArrows.Clear();

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
          : base("Mesh Scalar Inspector & Display", "ScalarView",
              "Evaluates and visualizes any scalar field on a mesh with scientific colormaps (Heat, CoolWarm, Viridis, Turbo, Zebra), live iso-contours, and 3D Min/Max value tags.",
              "CraneGH", "01. Operators & Display")
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

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _coloredMesh = null;
            _isoLines.Clear();

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
}
