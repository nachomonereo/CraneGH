using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_HeatDistance : GH_Component
    {
        private readonly List<Point3d> _sourcePts = new List<Point3d>();
        private readonly List<Point3d> _farthestPts = new List<Point3d>();

        public GH_HeatDistance()
          : base("Heat Geodesic & Signed Distance", "HeatDist",
              "Computes exact Geodesic Distance (d >= 0), Signed Geodesic Distance (SD, Feng & Crane 2024), Smooth Gradient (G), and Farthest Point (FP) on a mesh from any Curve/Geometry or default Naked Edges (full Graft support).",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("HeatDist");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (supports Grafted multi-mesh trees)", GH_ParamAccess.item);
            int idxG = pManager.AddGeometryParameter("Source Geometry", "G", "Curves, Points, Meshes, or Breps on/near the mesh (leave empty to compute geodesic distance from Naked Boundary Edges!)", GH_ParamAccess.list);
            pManager[idxG].Optional = true;
            pManager.AddNumberParameter("Smooth Time", "t", "Heat diffusion & gradient smoothing multiplier (default = 1.0)", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "d", "Exact unsigned Geodesic Distance per vertex (0.0 on the curve/naked edges up to the farthest vertex on the mesh)", GH_ParamAccess.list);
            pManager.AddVectorParameter("Smooth Gradient", "G", "Redistributed smooth unit geodesic gradient vector per vertex pointing along shortest geodesic paths", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by Keenan Crane's Heat Method colormap (also embeds exact scalar field)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Signed Distance", "SD", "Signed Geodesic Distance d_+- per vertex (The Signed Heat Method, Feng & Crane 2024: negative on one side of curve, 0 on curve, positive on other side)", GH_ParamAccess.list);
            pManager.AddPointParameter("Farthest Point", "FP", "Vertex on the mesh at maximum geodesic distance from the source curve / naked edges", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _sourcePts.Clear();
            _farthestPts.Clear();
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, null, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var geoms = new List<GeometryBase>();
            double tFactor = 1.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, geoms);
            DA.GetData(2, ref tFactor);

            var ddg = new DdgMesh(m);
            double[] dist = DdgAlgorithms.ComputeExternalGeometryHeatDistance(
                ddg, geoms, tFactor,
                out Vector3d[] vertGrad,
                out _,
                out List<Point3d> srcPts,
                out Mesh colored,
                out double[] signedDist,
                out Point3d farthestPt);

            _sourcePts.AddRange(srcPts);
            if (farthestPt.IsValid)
                _farthestPts.Add(farthestPt);

            DA.SetDataList(0, dist);
            DA.SetDataList(1, vertGrad);
            DA.SetData(2, colored);
            DA.SetDataList(3, signedDist);
            DA.SetData(4, farthestPt);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;
            if (_sourcePts.Count > 0)
                args.Display.DrawPoints(_sourcePts, Rhino.Display.PointStyle.RoundSimple, 4, Color.Gold);
            if (_farthestPts.Count > 0)
                args.Display.DrawPoints(_farthestPts, Rhino.Display.PointStyle.RoundControlPoint, 6, Color.Cyan);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000005");
    }

    public class GH_MeshIsoContours : GH_Component
    {
        public GH_MeshIsoContours()
          : base("Mesh Iso-Contours", "IsoCrvs",
              "Extracts smooth iso-contour polylines across a mesh from any vertex scalar field (e.g., Heat Distance, Feature Heatmap, Harmonic Field, Stripe Wave, Curvature).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoCrvs");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Scalar value per mesh vertex (can be real distances, coordinates, curvature, or any custom numbers)", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Count", "N", "Number of uniformly spaced iso-contour levels (ignored if Exact Iso-Values V are provided)", GH_ParamAccess.item, 20);
            int idxV = pManager.AddNumberParameter("Exact Iso-Values", "V", "Optional exact scalar value(s) to extract (e.g. 4.55 in real distance units, or 0.5 if scalar is 0..1)", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Iso-Curves", "C", "Extracted iso-contour polylines on the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("Iso-Levels", "L", "Scalar iso-values at which contours were extracted", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            int count = 20;
            var exactVals = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;
            DA.GetData(2, ref count);
            DA.GetDataList(3, exactVals);

            var ddg = new DdgMesh(m);
            double[] u = ddg.MapScalarInputToVertices(uList);
            int n = ddg.VertexCount;

            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                if (u[i] < minU) minU = u[i];
                if (u[i] > maxU) maxU = u[i];
            }

            var levels = new List<double>();
            if (exactVals.Count > 0)
            {
                levels.AddRange(exactVals);
            }
            else if (count <= 1)
            {
                levels.Add(minU < 0.0 && maxU > 0.0 ? 0.0 : 0.5 * (minU + maxU));
            }
            else
            {
                for (int k = 1; k <= count; k++)
                {
                    double frac = (double)k / (count + 1);
                    levels.Add(minU + frac * (maxU - minU));
                }
            }

            List<Polyline> contours = ddg.ExtractIsoContours(u, levels);
            DA.SetDataList(0, contours);
            DA.SetDataList(1, levels);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000006");
    }

    public class GH_GeodesicPointAToB : GH_Component
    {
        private readonly List<Polyline> _geoPaths = new List<Polyline>();
        private readonly List<Point3d> _endPts = new List<Point3d>();

        public GH_GeodesicPointAToB()
          : base("Geodesic Path A to B", "GeoAB",
              "Computes the exact shortest Geodesic Curve on a mesh directly from Point A to Point B (no Scalar Field needed).",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GeoPath");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Point A", "A", "Start point(s) A on or near the mesh", GH_ParamAccess.list);
            pManager.AddPointParameter("Point B", "B", "End point(s) B on or near the mesh", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Curve", "G", "Shortest geodesic curve(s) on the mesh from Point A to Point B", GH_ParamAccess.list);
            pManager.AddNumberParameter("Geodesic Length", "L", "Exact length of each geodesic curve along the mesh", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _geoPaths.Clear();
            _endPts.Clear();

            Mesh m = null;
            var ptsA = new List<Point3d>();
            var ptsB = new List<Point3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, ptsA) || ptsA.Count == 0) return;
            if (!DA.GetDataList(2, ptsB) || ptsB.Count == 0) return;

            var ddg = new DdgMesh(m);
            List<Polyline> paths = DdgAlgorithms.ComputeGeodesicPathAToB(ddg, ptsA, ptsB, out List<double> lengths);

            _geoPaths.AddRange(paths);
            foreach (var pl in paths)
            {
                if (pl != null && pl.Count >= 2)
                {
                    _endPts.Add(pl[0]);
                    _endPts.Add(pl[pl.Count - 1]);
                }
            }

            DA.SetDataList(0, paths);
            DA.SetDataList(1, lengths);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _geoPaths.Count == 0) return;

            Color cPath = Attributes.Selected ? Color.FromArgb(255, 245, 70) : Color.FromArgb(0, 255, 215);
            foreach (var pl in _geoPaths)
                args.Display.DrawPolyline(pl, cPath, 3);

            if (_endPts.Count > 0)
                args.Display.DrawPoints(_endPts, Rhino.Display.PointStyle.RoundSimple, 5, Color.Gold);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000054");
    }

    public class GH_GeodesicShortestPath : GH_Component
    {
        public GH_GeodesicShortestPath()
          : base("Geodesic Field Path", "GeoPath",
              "Traces shortest geodesic paths on a mesh from target points down a Distance Field d (or directly between points if d is empty).",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("GeoPath");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxD = pManager.AddNumberParameter("Distance Field", "d", "Optional geodesic distance per vertex (if empty, connects Targets sequentially as A-to-B geodesics)", GH_ParamAccess.list);
            pManager[idxD].Optional = true;
            pManager.AddPointParameter("Targets", "T", "Target points on the mesh", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Paths", "P", "Traced geodesic shortest path polylines on the mesh", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var dList = new List<double>();
            var targets = new List<Point3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, dList);
            if (!DA.GetDataList(2, targets) || targets.Count == 0) return;

            var ddg = new DdgMesh(m);
            if (dList.Count == 0 && targets.Count >= 2)
            {
                var listA = new List<Point3d>();
                var listB = new List<Point3d>();
                for (int i = 0; i < targets.Count - 1; i++)
                {
                    listA.Add(targets[i]);
                    listB.Add(targets[i + 1]);
                }
                List<Polyline> abPaths = DdgAlgorithms.ComputeGeodesicPathAToB(ddg, listA, listB, out _);
                DA.SetDataList(0, abPaths);
                return;
            }
            if (dList.Count == 0) return;

            double[] d = ddg.MapScalarInputToVertices(dList);
            int n = ddg.VertexCount;

            ddg.ComputeGradient(d, out _, out Vector3d[] gradV);
            double stepSize = 0.45 * ddg.MeanEdgeLength;
            int maxSteps = Math.Max(200, n * 2);

            var paths = new List<Polyline>();
            foreach (Point3d tPt in targets)
            {
                var pts = new List<Point3d>();
                MeshPoint mp = ddg.RhinoMesh.ClosestMeshPoint(tPt, 0.0);
                if (mp == null) continue;

                Point3d curr = mp.Point;
                pts.Add(curr);

                for (int step = 0; step < maxSteps; step++)
                {
                    int vNear = ddg.FindClosestVertex(curr);
                    if (d[vNear] < stepSize * 0.8)
                    {
                        pts.Add(ddg.Positions[vNear]);
                        break;
                    }

                    MeshFace f = ddg.RhinoMesh.Faces[mp.FaceIndex];
                    Vector3d g = mp.T[0] * gradV[f.A] + mp.T[1] * gradV[f.B] + mp.T[2] * gradV[f.C];
                    if (g.Length < 1e-12)
                    {
                        int bestNb = vNear;
                        foreach (int heIdx in ddg.VertexOutgoingHalfEdges[vNear])
                        {
                            int nb = ddg.HalfEdges[heIdx].TargetVertex;
                            if (d[nb] < d[bestNb]) bestNb = nb;
                        }
                        if (bestNb == vNear) break;
                        curr = ddg.Positions[bestNb];
                        pts.Add(curr);
                        mp = ddg.RhinoMesh.ClosestMeshPoint(curr, 0.0);
                        continue;
                    }

                    g.Unitize();
                    Point3d nextCandidate = curr - stepSize * g;
                    mp = ddg.RhinoMesh.ClosestMeshPoint(nextCandidate, 0.0);
                    if (mp == null || mp.Point.DistanceToSquared(curr) < 1e-14)
                        break;

                    curr = mp.Point;
                    pts.Add(curr);
                }

                if (pts.Count >= 2)
                    paths.Add(new Polyline(pts));
            }

            DA.SetDataList(0, paths);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000007");
    }

    public class GH_ExactIsoCut : GH_Component
    {
        private readonly List<Polyline> _cutCurves = new List<Polyline>();

        public GH_ExactIsoCut()
          : base("Iso-Splitting Curves", "IsoCut",
              "Extracts Iso-Splitting Curves on a mesh either by (1) Subdivisions Count N (e.g. 12 iso-curves) OR by (2) Exact Scalar Iso-Value(s) V (e.g. 3.4 in real scalar/geodesic units).",
              "CraneGH", "02. Scalar & Geodesics")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoCut");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (or Colored Mesh CM from SmoothScalar)", GH_ParamAccess.item);
            int idxU = pManager.AddGenericParameter("Scalar or Gradient", "S", "Scalar values (double, e.g. geodesic distance to a curve) or Scalar Gradient vectors (Vector3d)", GH_ParamAccess.list);
            pManager[idxU].Optional = true;
            pManager.AddIntegerParameter("Subdivisions", "N", "Option 1: Number of evenly-spaced iso-curves to extract (e.g. 12) when Exact Iso-Values V is empty", GH_ParamAccess.item, 12);
            int idxV = pManager.AddNumberParameter("Exact Iso-Values", "V", "Option 2: Specific scalar iso-value(s) to extract (e.g. 3.4 in real distance/scalar units). Overrides N when connected!", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Iso-Splitting Curves", "C", "Extracted iso-contour curves on the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("Extracted Iso-Levels", "L", "Exact scalar value(s) at which the curves were extracted", GH_ParamAccess.list);
            pManager.AddNumberParameter("Resolved Scalar", "S", "Active scalar values per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the active scalar field", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
            _cutCurves.Clear();
            CraneTreeHelper.NormalizeMeshAndFieldTrees(this, 0, new int[] { 1 });
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var rawU = new List<object>();
            int subCount = 12;
            var exactIsoVals = new List<double>();

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawU);
            DA.GetData(2, ref subCount);
            DA.GetDataList(3, exactIsoVals);

            var ddg = new DdgMesh(m);
            var inScalars = new List<double>();
            var inVecs = new List<Vector3d>();

            foreach (object obj in rawU)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is double d) inScalars.Add(d);
                else if (val is int iv) inScalars.Add(iv);
                else if (val is Vector3d vec) inVecs.Add(vec);
            }

            double[] u;
            if (inScalars.Count > 0)
            {
                u = ddg.MapScalarInputToVertices(inScalars);
            }
            else if (inVecs.Count > 0)
            {
                DdgAlgorithms.SmoothVectorAndGradientMultiMode(
                    ddg, inVecs, null, 2.0, 1, out _, out _, out u, out _);
            }
            else if (m.VertexColors.Count == m.Vertices.Count)
            {
                double[] rawCol = ddg.MapScalarInputToVertices(null);
                u = DdgAlgorithms.SmoothScalarField(ddg, rawCol, 1.5, 0.0, out _, out _);
            }
            else
            {
                var seeds = new List<int>();
                for (int i = 0; i < ddg.VertexCount; i++)
                    if (ddg.VertexOnBoundary[i]) seeds.Add(i);
                if (seeds.Count == 0) seeds.Add(0);
                u = DdgAlgorithms.ComputeHeatDistance(ddg, seeds, 1.0, out _, out _);
            }

            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                if (u[i] < minU) minU = u[i];
                if (u[i] > maxU) maxU = u[i];
            }
            double spanU = Math.Max(1e-12, maxU - minU);

            var levels = new List<double>();
            if (exactIsoVals.Count > 0)
            {
                // Option 2: User provided exact scalar value(s) (e.g. 3.4) -> extract at those exact scalar values!
                foreach (double val in exactIsoVals)
                    levels.Add(val);
            }
            else
            {
                // Option 1: Extract N evenly-spaced subdivisions across [minU, maxU]
                int count = Math.Max(1, Math.Min(250, subCount));
                for (int k = 1; k <= count; k++)
                {
                    double frac = k / (double)(count + 1);
                    levels.Add(minU + frac * spanU);
                }
            }
            levels.Sort();

            List<Polyline> crvs = ddg.ExtractIsoContours(u, levels);
            _cutCurves.AddRange(crvs);
            Mesh coloredM = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, u, false);

            DA.SetDataList(0, crvs);
            DA.SetDataList(1, levels);
            DA.SetDataList(2, u);
            DA.SetData(3, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _cutCurves.Count == 0) return;

            Color cCurve = Attributes.Selected ? Color.FromArgb(255, 255, 80) : Color.FromArgb(255, 210, 40);
            foreach (var pl in _cutCurves)
                args.Display.DrawPolyline(pl, cCurve, 2);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000034");
    }
}
