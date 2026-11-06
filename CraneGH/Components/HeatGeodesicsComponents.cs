using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_HeatDistance : GH_Component
    {
        private readonly List<Point3d> _sourcePts = new List<Point3d>();

        public GH_HeatDistance()
          : base("Heat Geodesic Distance", "HeatDist",
              "Computes geodesic distance and smoothed gradient on a mesh from ANY external geometry (Points, Curves, Meshes, or Breps) using Keenan Crane's Heat Method (ACM TOG 2013).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("HeatDist");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxG = pManager.AddGeometryParameter("Source Geometry", "G", "External heat sources: Points, Curves, Meshes, or Breps (leave empty to radiate from mesh boundary)", GH_ParamAccess.list);
            pManager[idxG].Optional = true;
            pManager.AddNumberParameter("Smooth Time", "t", "Heat diffusion & gradient smoothing multiplier (default = 1.0)", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "d", "Geodesic distance per vertex in real Rhino units", GH_ParamAccess.list);
            pManager.AddVectorParameter("Smooth Gradient", "G", "Redistributed smooth unit geodesic gradient vector per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by Keenan Crane's Heat Method colormap", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _sourcePts.Clear();
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
                out Mesh colored);

            _sourcePts.AddRange(srcPts);

            DA.SetDataList(0, dist);
            DA.SetDataList(1, vertGrad);
            DA.SetData(2, colored);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _sourcePts.Count == 0) return;
            args.Display.DrawPoints(_sourcePts, Rhino.Display.PointStyle.RoundSimple, 4, Color.Gold);
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

    public class GH_GeodesicShortestPath : GH_Component
    {
        public GH_GeodesicShortestPath()
          : base("Geodesic Shortest Path", "GeoPath",
              "Traces exact shortest geodesic paths on a mesh from target points down the negative gradient of a Heat Distance field.",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GeoPath");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance Field", "d", "Geodesic distance per vertex (from Heat Geodesic Distance)", GH_ParamAccess.list);
            pManager.AddPointParameter("Targets", "T", "Target points from which to trace geodesic paths back to sources", GH_ParamAccess.list);
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
            if (!DA.GetDataList(1, dList) || dList.Count == 0) return;
            if (!DA.GetDataList(2, targets) || targets.Count == 0) return;

            var ddg = new DdgMesh(m);
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
        private readonly List<Point3d> _labelPts = new List<Point3d>();
        private readonly List<string> _labelTexts = new List<string>();

        public GH_ExactIsoCut()
          : base("Exact Iso-Cut & Band Slicer", "IsoCut",
              "Extracts iso-contours and topologically cuts/slices a mesh at exact user-specified Scalar Iso-Values (e.g. 4.55 in real distance units, or 0.5 if scalar is in [0..1]).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoCut");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Custom or computed scalar values per vertex (real distances, coordinates, curvature, or [0..1] values)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Iso-Values", "V", "Exact scalar value(s) at which to cut (e.g. 4.55 for real distance, or 0.5 for midpoint of [0..1])", GH_ParamAccess.list, 0.5);
            pManager.AddBooleanParameter("Normalized [0..1]", "N", "False = V is evaluated in exact real scalar units (e.g. 4.55); True = V is a normalized fraction [0..1] between Min(u) and Max(u)", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Iso-Cut Curves", "C", "Exact iso-contour curves at u = V", GH_ParamAccess.list);
            pManager.AddMeshParameter("Lower Mesh (u <= V)", "ML", "Sub-mesh where scalar u <= first Iso-Value V", GH_ParamAccess.item);
            pManager.AddMeshParameter("Upper Mesh (u >= V)", "MU", "Sub-mesh where scalar u >= last Iso-Value V", GH_ParamAccess.item);
            pManager.AddMeshParameter("Iso-Band Meshes", "MB", "Sliced sub-mesh bands between consecutive Iso-Values", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by scalar field u", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _cutCurves.Clear();
            _labelPts.Clear();
            _labelTexts.Clear();

            Mesh m = null;
            var uList = new List<double>();
            var isoVals = new List<double>();
            bool normalized = false;

            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;
            if (!DA.GetDataList(2, isoVals) || isoVals.Count == 0) isoVals.Add(0.5);
            DA.GetData(3, ref normalized);

            var ddg = new DdgMesh(m);
            List<Polyline> crvs = DdgAlgorithms.ComputeExactIsoCutAndBands(
                ddg, uList, isoVals, normalized,
                out Mesh lowerM, out Mesh upperM, out List<Mesh> bands,
                out Mesh coloredM, out List<double> resolvedIsos);

            _cutCurves.AddRange(crvs);
            for (int i = 0; i < crvs.Count; i++)
            {
                if (crvs[i].Count > 0)
                {
                    _labelPts.Add(crvs[i][crvs[i].Count / 2]);
                    double vTag = resolvedIsos[Math.Min(i, resolvedIsos.Count - 1)];
                    _labelTexts.Add($"u = {vTag:0.###}");
                }
            }

            DA.SetDataList(0, crvs);
            DA.SetData(1, lowerM);
            DA.SetData(2, upperM);
            DA.SetDataList(3, bands);
            DA.SetData(4, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _cutCurves.Count == 0) return;

            Color cCurve = Attributes.Selected ? Color.FromArgb(255, 255, 80) : Color.FromArgb(255, 210, 40);
            foreach (var pl in _cutCurves)
                args.Display.DrawPolyline(pl, cCurve, 3);

            for (int i = 0; i < _labelPts.Count; i++)
                args.Display.DrawDot(_labelPts[i], _labelTexts[i], Color.FromArgb(30, 35, 45), Color.Gold);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000034");
    }
}
